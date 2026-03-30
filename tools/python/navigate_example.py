"""
navigate_example.py
Autonomous frequency-based waypoint navigation with BearingAngle + WLS fusion.

The drone:
  1. Searches (rotates) until it detects the target frequency light source
  2. Aligns its forward axis toward the source using the bearing angle estimate
  3. Approaches while the fusion controller blends bearing + spatial gradient
     (bearing-only until map has ≥3 points, then 50/50 fusion)
  4. Holds at the source for dwell_ms once SNR exceeds the arrival threshold
  5. Advances to the next waypoint
"""
import logging
from gradient_drone import GradientDrone

logging.basicConfig(level=logging.INFO)

URI = 'radio://0/80/2M'   # TODO: update

WAYPOINTS = [
    {'frequency_hz': 150.0, 'dwell_ms': 3000},
    {'frequency_hz': 200.0, 'dwell_ms': 3000},
    {'frequency_hz': 150.0, 'dwell_ms': 2000},
]

with GradientDrone(URI) as drone:
    drone.start_logging(period_ms=10)

    # Flight parameters
    drone.set_nav_params(
        alt_target   = 1.0,    # hover height (m)
        fwd_speed    = 0.20,   # approach speed (m/s)
        yaw_gain     = 2.0,    # heading error (deg) → yaw rate (deg/s)
        align_tol    = 15.0,   # ±degrees to be considered aligned
        acq_snr      = 5.0,    # minimum SNR to trust bearing
        arr_snr      = 10.0,   # SNR threshold for "arrived at waypoint"
        nav_yaw_rate = 30.0,   # search/align yaw rate (deg/s)
    )

    # Fusion parameters (mirrors blimp.cpp defaults)
    drone.set_fusion_params(
        w_bearing      = 0.5,   # bearing weight (active once map ready)
        w_gradient     = 0.5,   # gradient weight
        smooth_factor  = 0.3,   # bearing low-pass (0=no smoothing, 1=no update)
        grad_threshold = 0.5,   # min gradient magnitude to activate fusion
        min_light      = 1.0,   # min total FFT magnitude to trust bearing
        min_map_pts    = 3,     # map cells needed before gradient used
    )

    # WLS map parameters
    drone.set_wls_params(
        map_r2_threshold = 0.5,   # min weighted R² to accept gradient
        map_max_dist     = 3.0,   # max distance (m) of map point to include
        map_grid_res     = 0.2,   # spatial map cell size (m)
    )

    drone.upload_frequency_waypoints(WAYPOINTS)
    drone.takeoff(height=1.0)
    drone.set_mode(GradientDrone.MODE_NAVIGATE)
    print("NAVIGATE active — watching nav.wB/wG LOG vars to see fusion kick in")

    success = drone.wait_for_mission_complete(timeout=180)
    print("Mission complete" if success else "Timeout — landing")

    drone.set_mode(GradientDrone.MODE_MANUAL)
    drone.land()
    drone.stop_logging()
    drone.save_log('navigate_session.csv')
