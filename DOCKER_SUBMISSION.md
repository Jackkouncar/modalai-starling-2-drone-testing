# Docker Submission Guide

This repo contains a Dockerized PX4/Gazebo/ROS 2 setup for the ModalAI Starling 2 Max offboard tests.

## What Worked

The tested Docker demo ran:

- PX4 SITL from the local PX4 checkout
- Gazebo Harmonic
- Baylands world
- `gz_x500_depth` vehicle
- Micro XRCE-DDS Agent
- ROS 2 Humble
- this repo's `test_takeoff_land.py`

The successful test ended with:

```text
=== Test Complete ===
```

## One-Time Setup

Install Docker Desktop and enable WSL integration for Ubuntu.

From Ubuntu/WSL:

```bash
docker --version
docker compose version
sudo apt update
sudo apt install -y x11-xserver-utils
```

Build the image:

```bash
cd "/mnt/c/Users/JackK/Downloads/Drone Testing"
PX4_MSGS_REF=main docker compose -f docker-compose.yml -f docker-compose.local-px4.yml build
```

## Run The Baylands Sim

Terminal 1:

```bash
cd "/mnt/c/Users/JackK/Downloads/Drone Testing"
xhost +local:root

PX4_HOST_DIR="$HOME/ros2_simulation/PX4-Autopilot" \
PX4_GZ_WORLD=baylands \
PX4_SIM_TARGET=gz_x500_depth \
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml up sim
```

Wait until PX4 prints lines like:

```text
Gazebo world is ready
Spawning Gazebo model
world: baylands, model: x500_depth_0
```

## Run The Takeoff/Land Demo

Terminal 2:

```bash
cd "/mnt/c/Users/JackK/Downloads/Drone Testing"

docker compose -f docker-compose.yml -f docker-compose.local-px4.yml exec \
  -e DRONE_SIM_SKIP_STATUS_GATE=1 sim \
  bash /workspace/drone-tests/docker/run_test.sh test_takeoff_land.py
```

The `DRONE_SIM_SKIP_STATUS_GATE=1` flag is for Gazebo only. It avoids Windows/WSL/QGroundControl MAVLink issues during the Docker demo. Do not use it on the real drone.

## Optional Movement Test

After takeoff/land works:

```bash
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml exec \
  -e DRONE_SIM_SKIP_STATUS_GATE=1 sim \
  bash /workspace/drone-tests/docker/run_test.sh test_forward_backward.py
```

## Message For The Team

```text
The drone sim is dockerized now. Docker runs PX4 SITL, Gazebo Baylands, Micro XRCE-DDS, ROS 2 Humble, px4_msgs, and our Python offboard scripts. The tested demo command runs test_takeoff_land.py inside the container and completed takeoff, hover, descent, and land successfully.
```

## Notes

The exact Baylands demo uses Jack's local PX4 checkout at:

```text
~/ros2_simulation/PX4-Autopilot
```

For another machine, either clone the same PX4 checkout there or adjust `PX4_HOST_DIR` to point at that machine's PX4 checkout.
