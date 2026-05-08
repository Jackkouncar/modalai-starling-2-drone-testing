# Run The Docker Drone Sim On Windows + WSL

These instructions are for running the tested Docker demo on a Windows PC using Ubuntu/WSL and Docker Desktop.

## What You Need

- Windows with WSL 2 / Ubuntu installed
- Docker Desktop installed and running
- Docker Desktop WSL integration enabled for Ubuntu
- The project repo cloned
- A PX4 checkout at `~/ros2_simulation/PX4-Autopilot`

The tested demo uses:

- PX4 SITL
- Gazebo Harmonic
- Baylands world
- `gz_x500_depth` drone
- Micro XRCE-DDS Agent
- ROS 2 Humble
- `test_takeoff_land.py`

## 1. Check Docker

Open Ubuntu/WSL and run:

```bash
docker --version
docker compose version
docker run hello-world
```

If `docker` is missing, open Docker Desktop on Windows and enable:

```text
Settings > Resources > WSL Integration > Ubuntu
```

Then restart WSL:

```powershell
wsl --shutdown
```

Reopen Ubuntu.

## 2. Install X11 Helper

In Ubuntu/WSL:

```bash
sudo apt update
sudo apt install -y x11-xserver-utils
```

## 3. Clone This Repo

Use the GitHub repo:

```bash
cd ~
git clone https://github.com/Jackkouncar/modalai-starling-2-drone-testing.git
cd modalai-starling-2-drone-testing
```

If you are using Jack's current Windows folder instead, use:

```bash
cd "/mnt/c/Users/JackK/Downloads/Drone Testing"
```

## 4. Prepare PX4 For Baylands

The Docker Baylands run mounts a PX4 checkout from WSL.

If you already have this folder, keep it:

```bash
ls ~/ros2_simulation/PX4-Autopilot
```

If it does not exist, create it:

```bash
mkdir -p ~/ros2_simulation
git clone https://github.com/PX4/PX4-Autopilot.git ~/ros2_simulation/PX4-Autopilot
cd ~/ros2_simulation/PX4-Autopilot
git submodule update --init --recursive
```

## 5. Build The Docker Image

Go back to this project:

```bash
cd ~/modalai-starling-2-drone-testing
```

Or, on Jack's current PC:

```bash
cd "/mnt/c/Users/JackK/Downloads/Drone Testing"
```

Build:

```bash
PX4_MSGS_REF=main docker compose -f docker-compose.yml -f docker-compose.local-px4.yml build
```

The first build can take a long time. If Wi-Fi drops or a package download times out, run the same build command again.

## 6. Start The Baylands Sim

Terminal 1:

```bash
cd ~/modalai-starling-2-drone-testing
xhost +local:root

PX4_HOST_DIR="$HOME/ros2_simulation/PX4-Autopilot" \
PX4_GZ_WORLD=baylands \
PX4_SIM_TARGET=gz_x500_depth \
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml up sim
```

On Jack's current PC, use this first line instead:

```bash
cd "/mnt/c/Users/JackK/Downloads/Drone Testing"
```

Wait until the terminal shows:

```text
Gazebo world is ready
Spawning Gazebo model
world: baylands, model: x500_depth_0
```

The first sim start may compile PX4. That can take several minutes.

## 7. Run The Takeoff/Land Test

Open Terminal 2:

```bash
cd ~/modalai-starling-2-drone-testing

docker compose -f docker-compose.yml -f docker-compose.local-px4.yml exec \
  -e DRONE_SIM_SKIP_STATUS_GATE=1 sim \
  bash /workspace/drone-tests/docker/run_test.sh test_takeoff_land.py
```

On Jack's current PC, use this first line instead:

```bash
cd "/mnt/c/Users/JackK/Downloads/Drone Testing"
```

The successful test ends with:

```text
=== Test Complete ===
```

## 8. Optional Movement Test

Only after takeoff/land works:

```bash
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml exec \
  -e DRONE_SIM_SKIP_STATUS_GATE=1 sim \
  bash /workspace/drone-tests/docker/run_test.sh test_forward_backward.py
```

## Stop Everything

Press `Ctrl+C` in the sim terminal, then run:

```bash
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml down
```

## Important Notes

`DRONE_SIM_SKIP_STATUS_GATE=1` is for Gazebo only. It avoids Windows/WSL/QGroundControl MAVLink status issues during the Docker demo.

Do not use `DRONE_SIM_SKIP_STATUS_GATE=1` on the real drone.

QGroundControl is optional for this Docker demo. The Docker test starts a small MAVLink GCS heartbeat automatically when `DRONE_SIM_SKIP_STATUS_GATE=1` is used.

## Common Problems

If Docker says permission denied:

```bash
sudo groupadd docker 2>/dev/null || true
sudo usermod -aG docker "$USER"
```

Then restart WSL from PowerShell:

```powershell
wsl --shutdown
```

If the Python test prints RTPS payload-size errors, rebuild with matching PX4 messages:

```bash
PX4_MSGS_REF=main docker compose -f docker-compose.yml -f docker-compose.local-px4.yml build
```

If the sim says `unknown target`, make sure the command uses:

```bash
PX4_GZ_WORLD=baylands
PX4_SIM_TARGET=gz_x500_depth
```

not `gz_x500_depth_baylands`.
