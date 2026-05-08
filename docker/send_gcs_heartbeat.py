#!/usr/bin/env python3
"""Send a MAVLink GCS heartbeat so PX4 SITL passes the GCS preflight check."""

import argparse
import os
import sys
import time


PX4_HOME = os.environ.get("PX4_HOME", "/workspace/local-PX4-Autopilot")
PYMAVLINK_PATHS = (
    os.path.join(PX4_HOME, "src/modules/mavlink/mavlink/pymavlink"),
    "/workspace/PX4-Autopilot/src/modules/mavlink/mavlink/pymavlink",
)


try:
    from pymavlink import mavutil
except ImportError:
    for path in PYMAVLINK_PATHS:
        if os.path.isdir(path):
            sys.path.insert(0, path)
    from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default=os.environ.get("MAVLINK_URL", "udpin:0.0.0.0:14550"))
    parser.add_argument("--rate-hz", type=float, default=1.0)
    args = parser.parse_args()

    print(f"Connecting to PX4 MAVLink on {args.url}...")
    master = mavutil.mavlink_connection(
        args.url,
        source_system=255,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
    )

    heartbeat = master.wait_heartbeat(timeout=20.0)
    if heartbeat is None:
        print("Timed out waiting for PX4 heartbeat.")
        return 1

    print(
        f"PX4 heartbeat received from system={master.target_system}, "
        f"component={master.target_component}."
    )
    print("Sending GCS heartbeat. Leave this running while the mission test runs.")

    delay = 1.0 / max(args.rate_hz, 0.1)
    while True:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        time.sleep(delay)


if __name__ == "__main__":
    raise SystemExit(main())
