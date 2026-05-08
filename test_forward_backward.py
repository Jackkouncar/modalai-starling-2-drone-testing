#!/usr/bin/env python3
"""Guarded forward/backward PX4 offboard test."""

from flight_config import TARGET_DRONE, TRANSIT_DISTANCE_M
from guarded_mission import run_guarded_mission


def main():
    d = TRANSIT_DISTANCE_M
    run_guarded_mission(
        'test_forward_backward',
        f'{TARGET_DRONE} Test: Forward / Backward Transit',
        [
            ('forward position', d, 0.0),
            ('home position', 0.0, 0.0),
        ],
    )


if __name__ == '__main__':
    main()
