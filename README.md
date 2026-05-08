# ModalAI Starling 2 Max Drone Test Scripts

ROS 2 / PX4 offboard test scripts for basic movement checks with the ModalAI Starling 2 Max drone setup.

Repository:

```text
https://github.com/Jackkouncar/modalai-starling-2-drone-testing
```

## Submission Status

The Dockerized Baylands simulation was tested successfully on WSL/Ubuntu with Docker Desktop.

The working demo ran:

- PX4 SITL
- Gazebo Harmonic
- Baylands world
- `gz_x500_depth` vehicle
- Micro XRCE-DDS Agent
- ROS 2 Humble
- this repo's `test_takeoff_land.py`

The successful run completed takeoff, hover, descent, and land, ending with:

```text
=== Test Complete ===
```

For exact submission/demo commands, use [DOCKER_SUBMISSION.md](DOCKER_SUBMISSION.md).

For detailed Windows/WSL run instructions, use [RUN_ON_WINDOWS_WSL.md](RUN_ON_WINDOWS_WSL.md).

## Docker Quick Start

Build the local Baylands image:

```bash
cd "/mnt/c/Users/JackK/Downloads/Drone Testing"
PX4_MSGS_REF=main docker compose -f docker-compose.yml -f docker-compose.local-px4.yml build
```

Start the Baylands sim:

```bash
xhost +local:root

PX4_HOST_DIR="$HOME/ros2_simulation/PX4-Autopilot" \
PX4_GZ_WORLD=baylands \
PX4_SIM_TARGET=gz_x500_depth \
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml up sim
```

Run the tested takeoff/land demo in a second terminal:

```bash
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml exec \
  -e DRONE_SIM_SKIP_STATUS_GATE=1 sim \
  bash /workspace/drone-tests/docker/run_test.sh test_takeoff_land.py
```

`DRONE_SIM_SKIP_STATUS_GATE=1` is for Gazebo only. It handles Windows/WSL/QGroundControl MAVLink status issues during the Docker demo. Do not use it on the real drone.

## Included Scripts

- `test_takeoff_land.py`
- `test_forward_backward.py`
- `test_left_right.py`
- `test_all_directions.py`

Shared settings live in `flight_config.py`.

The movement scripts share `guarded_mission.py`, which provides the same core safety behavior:

- wait for valid PX4 local position
- capture the current local position as home
- fly waypoints relative to home
- command land if local position goes stale, invalid, too high, or too far from the commanded setpoint

## Safety Limits

Current test limits:

- Takeoff height: `1.0 m`
- Horizontal movement: `1.5 m`
- Soft landing descent: `6.0 s` down to about `0.15 m`, then PX4 land command
- Real-drone safety radius: `0.75 m` horizontal error
- Lab-sim safety radius: `2.0 m` horizontal error
- Max altitude: `1.6 m`

Start with `test_takeoff_land.py` only. Do not run movement scripts until takeoff and landing are stable.

For the real Starling 2 Max, do not start with the square test. Run `test_takeoff_land.py` first, then one-axis movement only after takeoff/land is boring and repeatable.

## Manual ROS/Gazebo Run

For the lab setup outside Docker:

```bash
source /opt/ros/humble/setup.bash
source ~/autonomy_ws/install/setup.bash
MicroXRCEAgent udp4 -p 8888
```

In another terminal:

```bash
cd ~/ros2_simulation/PX4-Autopilot
PX4_GZ_WORLD=baylands make px4_sitl gz_x500_depth
```

Then run this repo's first safety test:

```bash
cd ~/modalai-starling-2-drone-testing
python3 test_takeoff_land.py
```

## Notes

These scripts publish PX4 offboard topics such as `/fmu/in/offboard_control_mode` and `/fmu/in/trajectory_setpoint`.

They should run on a companion computer or ground computer that has access to the ROS 2 / PX4 bridge. They usually do not get copied directly onto the flight controller.

This repo supports ROS 2 Foxy for the physical drone side and ROS 2 Humble for the lab Gazebo setup.
