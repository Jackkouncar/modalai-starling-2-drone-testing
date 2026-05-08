#!/usr/bin/env python3
"""Guarded square-pattern PX4 offboard test."""

from flight_config import TARGET_DRONE, TRANSIT_DISTANCE_M
from guarded_mission import run_guarded_mission


def main():
    d = TRANSIT_DISTANCE_M
    run_guarded_mission(
        'test_all_directions',
        f'{TARGET_DRONE} Test: All-Directions Square',
        [
            ('forward corner', d, 0.0),
            ('forward-right corner', d, d),
            ('right corner', 0.0, d),
            ('home position', 0.0, 0.0),
        ],
    )


if __name__ == '__main__':
    main()
