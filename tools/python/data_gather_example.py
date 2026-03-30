"""
data_gather_example.py
Fly manually while logging raw PD channels, bearing, WLS gradients,
and fusion diagnostics at 100 Hz.
"""
import logging
from gradient_drone import GradientDrone

logging.basicConfig(level=logging.INFO)
URI = 'radio://0/80/2M'   # TODO: update

with GradientDrone(URI) as drone:
    drone.start_logging(period_ms=10)
    drone.set_mode(GradientDrone.MODE_DATA_GATHER)
    drone.takeoff(height=1.0)
    input("Flying — press Enter to land")
    drone.set_mode(GradientDrone.MODE_MANUAL)
    drone.land()
    drone.stop_logging()
    n = drone.save_log('data_gather_001.csv')
    print(f"Saved {n} records")
