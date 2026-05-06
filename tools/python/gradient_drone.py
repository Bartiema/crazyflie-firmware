"""
gradient_drone.py — cflib wrapper for the CrazyFlie gradient navigation system.

Architecture:
    Photodiodes → FFT → per-channel magnitude at target frequency
                      → BearingAngle (weighted circular mean of sensor angles)
                      → WLS gradient (spatial map, inverse-distance² weighted plane fit)
                      → Heading fusion: weighted_circular_mean(bearing, gradient)
                         (50/50 when map ready, bearing-only otherwise)
                      → Waypoint navigator (frequency-based seek + dwell)
                      → CrazyFlie velocity + yaw-rate setpoint

Quick-start (data gathering):
    with GradientDrone('radio://0/80/2M') as drone:
        drone.start_logging(period_ms=10)
        drone.set_mode(GradientDrone.MODE_DATA_GATHER)
        drone.takeoff(height=1.0)
        input("Press Enter to land")
        drone.land()
        drone.save_log('session.csv')

Quick-start (navigation):
    with GradientDrone('radio://0/80/2M') as drone:
        drone.upload_frequency_waypoints([
            {'frequency_hz': 150.0, 'dwell_ms': 3000},
            {'frequency_hz': 200.0, 'dwell_ms': 3000},
        ])
        drone.takeoff(height=1.0)
        drone.set_mode(GradientDrone.MODE_NAVIGATE)
        drone.wait_for_mission_complete(timeout=180)
        drone.land()
"""

import struct, time, csv, logging
from pathlib import Path
from typing import Callable, Dict, List, Optional

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement

log = logging.getLogger(__name__)


class GradientDrone:
    MODE_MANUAL      = 0
    MODE_DATA_GATHER = 1
    MODE_NAVIGATE    = 2

    _WP_FMT  = '<ffI'
    _WP_SIZE = struct.calcsize(_WP_FMT)

    _LOG_VARS = [
        # ('pd.ch0','float'),('pd.ch1','float'),('pd.ch2','float'),('pd.ch3','float'),
        # ('pd.ch4','float'),('pd.ch5','float'),('pd.ch6','float'),('pd.ch7','float'),
        ('bearingCtrl.bearing','float'),
        # ('bearingCtrl.weight','float'),
        ('bearingCtrl.valid','uint8_t'),
        ('stateEstimate.x','float'),('stateEstimate.y','float'),
        # ,('stateEstimate.z','float'),
        ('stateEstimate.yaw','float'),
        # ('wlsCtrl.iGradX','float'),('wlsCtrl.iGradY','float'),
        # ('wlsCtrl.iGradMag','float'),('wlsCtrl.iGradAng','float'),
        # ('wlsCtrl.mGradX','float'),('wlsCtrl.mGradY','float'),
        # ('wlsCtrl.mGradMag','float'),('wlsCtrl.mGradAng','float'),
        # ('wlsCtrl.mR2','float'),('wlsCtrl.mapSize','int32_t'),
        # ('nav.bearing','float'),('nav.gradAng','float'),('nav.gradMag','float'),
        # ('nav.cmdYaw','float'),('nav.wB','float'),('nav.wG','float'),
        # ('nav.mapSize','int32_t'),('nav.mode','uint8_t'),
        # ('wpNav.state','uint8_t'),('wpNav.wpIdx','uint8_t'),
        # ('wpNav.vx','float'),('wpNav.yawRate','float'),
    ]

    def __init__(self, uri: str, cache_dir: str = './cache'):
        self.uri = uri
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.cf  = Crazyflie(rw_cache=cache_dir)
        self.scf = SyncCrazyflie(uri, cf=self.cf)
        self._active_log:  Optional[LogConfig] = None
        self._log_records: List[Dict]          = []
        self._connected = False

    def connect(self):
        self.scf.open_link(); self._connected = True
        log.info("Connected to %s", self.uri)

    def disconnect(self):
        if self._active_log: self.stop_logging()
        self.scf.close_link(); self._connected = False

    def __enter__(self):  self.connect();    return self
    def __exit__(self, *_): self.disconnect()

    # ── Mode ──────────────────────────────────────────────────────────────────

    def set_mode(self, mode: int):
        assert mode in (0, 1, 2)
        self.cf.param.set_value('nav.mode', str(mode))

    def get_mode(self) -> int:
        return int(self.cf.param.get_value('nav.mode'))

    # ── Parameters ────────────────────────────────────────────────────────────

    def set_nav_params(self, *, alt_target=None, fwd_speed=None, yaw_gain=None,
                       align_tol=None, acq_snr=None, arr_snr=None, nav_yaw_rate=None):
        p = self.cf.param
        if alt_target   is not None: p.set_value('nav.altTarget',  str(alt_target))
        if fwd_speed    is not None: p.set_value('nav.fwdSpeed',   str(fwd_speed))
        if yaw_gain     is not None: p.set_value('nav.yawGain',    str(yaw_gain))
        if align_tol    is not None: p.set_value('wpNav.alignTol', str(align_tol))
        if acq_snr      is not None: p.set_value('wpNav.acqSnr',   str(acq_snr))
        if arr_snr      is not None: p.set_value('wpNav.arrSnr',   str(arr_snr))
        if nav_yaw_rate is not None: p.set_value('wpNav.yawRate',  str(nav_yaw_rate))

    def set_fusion_params(self, *, w_bearing=None, w_gradient=None,
                          smooth_factor=None, grad_threshold=None,
                          min_light=None, min_map_pts=None):
        """
        Tune the BearingAngle + WLS fusion (mirrors blimp.cpp constants).
        w_bearing + w_gradient should sum to 1.0.
        grad_threshold is the minimum gradient magnitude to activate fusion.
        """
        p = self.cf.param
        if w_bearing      is not None: p.set_value('nav.wBearing',   str(w_bearing))
        if w_gradient     is not None: p.set_value('nav.wGradient',  str(w_gradient))
        if smooth_factor  is not None: p.set_value('nav.smoothFact', str(smooth_factor))
        if grad_threshold is not None: p.set_value('nav.gradThresh', str(grad_threshold))
        if min_light      is not None: p.set_value('nav.minLight',   str(min_light))
        if min_map_pts    is not None: p.set_value('nav.minMapPts',  str(min_map_pts))

    def set_wls_params(self, *, map_r2_threshold=None, map_max_dist=None,
                       map_grid_res=None, step_size=None):
        """Tune the WLS spatial map estimator."""
        p = self.cf.param
        if map_r2_threshold is not None: p.set_value('wlsCtrl.mapR2',      str(map_r2_threshold))
        if map_max_dist     is not None: p.set_value('wlsCtrl.mapMaxDist', str(map_max_dist))
        if map_grid_res     is not None: p.set_value('wlsCtrl.mapGridRes', str(map_grid_res))
        if step_size        is not None: p.set_value('wlsCtrl.stepSize',   str(step_size))

    def clear_wls_map(self):
        self.cf.param.set_value('wlsCtrl.clearMap', '1')

    def get_map_size(self) -> int:
        return int(self.cf.param.get_value('wlsCtrl.mapSize'))

    # ── Waypoints ─────────────────────────────────────────────────────────────

    def upload_frequency_waypoints(self, waypoints: List[Dict]):
        """
        Upload frequency-based waypoints.
        Each dict: {'frequency_hz': float, 'dwell_ms': float}
        """
        payload = b''.join(
            struct.pack(self._WP_FMT, float(wp['frequency_hz']),
                        float(wp['dwell_ms']), 0)
            for wp in waypoints
        )
        mems = self.cf.mem.get_mems(MemoryElement.TYPE_APP)
        if not mems:
            raise RuntimeError("No APP memory element found")
        mems[0].write_data(0, payload)
        log.info("Uploaded %d waypoints", len(waypoints))

    def reset_mission(self):
        self.cf.param.set_value('wpNav.reset', '1')

    # ── Calibration ───────────────────────────────────────────────────────────

    def start_calibration(self):
        self.cf.param.set_value('bearingCtrl.calibrate', '1')

    def finalise_calibration(self):
        self.cf.param.set_value('bearingCtrl.calibrate', '2')

    def reset_calibration(self):
        for i in range(8):
            self.cf.param.set_value(f'bearingCtrl.corr{i}', '1.0')

    def get_calibration_factors(self) -> List[float]:
        return [float(self.cf.param.get_value(f'bearingCtrl.corr{i}')) for i in range(8)]

    # ── Logging ───────────────────────────────────────────────────────────────

    def start_logging(self, callback: Optional[Callable] = None, period_ms: int = 10):
        if self._active_log: return
        lg = LogConfig(name='gradient_nav', period_in_ms=period_ms)
        for var, typ in self._LOG_VARS:
            try: lg.add_variable(var, typ)
            except KeyError: pass

        def _cb(ts, data, lc):
            data['timestamp_ms'] = ts
            self._log_records.append(dict(data))
            if callback: callback(ts, data, lc)

        lg.data_received_cb.add_callback(_cb)
        self.cf.log.add_config(lg)
        lg.start()
        self._active_log = lg

    def stop_logging(self):
        if self._active_log:
            self._active_log.stop(); self._active_log = None

    def save_log(self, path: str, clear: bool = True) -> int:
        if not self._log_records: return 0
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self._log_records[0].keys()),
                               extrasaction='ignore')
            w.writeheader(); w.writerows(self._log_records)
        n = len(self._log_records)
        log.info("Saved %d records to %s", n, out)
        if clear: self._log_records.clear()
        return n

    # ── Flight ────────────────────────────────────────────────────────────────

    def takeoff(self, height: float = 0.5, duration: float = 2.0):
        self.cf.high_level_commander.takeoff(height, duration)
        time.sleep(duration + 0.5)

    def land(self, duration: float = 2.0):
        self.cf.high_level_commander.land(0.0, duration)
        time.sleep(duration + 0.5)

    def wait_for_mission_complete(self, timeout: float = 180.0, poll_s: float = 0.5,
                                   stop_event=None) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if stop_event is not None and stop_event.is_set():
                log.info("Aborted by stop_event"); return False
            try:
                if int(self.cf.param.get_value('wpNav.state')) == 6:
                    log.info("Mission complete"); return True
            except Exception: pass
            time.sleep(poll_s)
        log.warning("Timeout after %.0f s", timeout)
        return False
