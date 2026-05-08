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

### Stuck On `Waiting for valid local position and velocity`

This means the Python test is not seeing a fresh, valid PX4 local-position estimate yet. It is usually one of these:

- the sim is still compiling or Gazebo is not ready yet
- PX4 is running but the Micro XRCE-DDS bridge has not created the local-position topics yet
- the image was built with mismatched `px4_msgs`
- the PX4 build volume is stale from an older run

Do this exact recovery path.

In Terminal 1, make sure the sim is still running and wait until it shows:

```text
Gazebo world is ready
Spawning Gazebo model
world: baylands, model: x500_depth_0
Startup script returned successfully
home set
```

The first sim start can compile PX4 for several minutes. Wait for that to finish before running the test.

If it still gets stuck, stop the sim:

```bash
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml down
```

Check that the PX4 checkout is the WSL checkout and has the Baylands world:

```bash
PX4_HOST_DIR="$HOME/ros2_simulation/PX4-Autopilot"
test -f "$PX4_HOST_DIR/Tools/simulation/gz/worlds/baylands.sdf" && echo "PX4 Baylands world found"
```

If it does not print `PX4 Baylands world found`, create or update the PX4 checkout:

```bash
mkdir -p "$HOME/ros2_simulation"

if [ ! -d "$HOME/ros2_simulation/PX4-Autopilot/.git" ]; then
  git clone https://github.com/PX4/PX4-Autopilot.git "$HOME/ros2_simulation/PX4-Autopilot"
fi

cd "$HOME/ros2_simulation/PX4-Autopilot"
git pull --ff-only
git submodule update --init --recursive
```

Return to this repo:

```bash
cd ~/modalai-starling-2-drone-testing
```

On Jack's current PC, use:

```bash
cd "/mnt/c/Users/JackK/Downloads/Drone Testing"
```

Clear the stale mounted-PX4 build volume:

```bash
docker volume rm modalai-starling2_px4-local-harmonic-build 2>/dev/null || true
```

Rebuild with PX4 main message definitions:

```bash
PX4_MSGS_REF=main docker compose -f docker-compose.yml -f docker-compose.local-px4.yml build
```

Start the sim again in Terminal 1:

```bash
xhost +local:root

PX4_HOST_DIR="$HOME/ros2_simulation/PX4-Autopilot" \
PX4_GZ_WORLD=baylands \
PX4_SIM_TARGET=gz_x500_depth \
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml up --force-recreate sim
```

Wait again for `Startup script returned successfully` and `home set`.

In Terminal 2, verify ROS can see PX4 local-position topics:

```bash
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml exec sim \
  bash -lc 'set +u; source /opt/ros/humble/setup.bash; source /workspace/ros2_ws/install/setup.bash; set -u; ros2 topic list | grep vehicle_local_position'
```

Expected output should include at least one of these:

```text
/fmu/out/vehicle_local_position
/fmu/out/vehicle_local_position_v1
```

Then verify one local-position message arrives:

```bash
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml exec sim \
  bash -lc 'set +u; source /opt/ros/humble/setup.bash; source /workspace/ros2_ws/install/setup.bash; set -u; timeout 5 ros2 topic echo /fmu/out/vehicle_local_position_v1 --once || timeout 5 ros2 topic echo /fmu/out/vehicle_local_position --once'
```

If no message prints, check Terminal 1 for these Micro XRCE-DDS lines:

```text
session established
successfully created rt/fmu/out/vehicle_local_position
successfully created rt/fmu/out/vehicle_local_position_v1
```

Once the topic check works, run the test:

```bash
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml exec \
  -e DRONE_SIM_SKIP_STATUS_GATE=1 sim \
  bash /workspace/drone-tests/docker/run_test.sh test_takeoff_land.py
```

### `No connection to the GCS` / QGroundControl Not Connecting

QGroundControl is not required for this Docker demo. The test command above automatically starts a small MAVLink GCS heartbeat when `DRONE_SIM_SKIP_STATUS_GATE=1` is set.

If PX4 keeps printing this:

```text
Preflight Fail: No connection to the GCS
```

use the manual third-terminal heartbeat path.

Terminal 1: keep the sim running.

Terminal 3: start the heartbeat and leave it running:

```bash
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml exec sim \
  bash -lc 'python3 /workspace/drone-tests/docker/send_gcs_heartbeat.py'
```

Expected output:

```text
PX4 heartbeat received from system=1, component=0.
Sending GCS heartbeat. Leave this running while the mission test runs.
```

Terminal 2: run the test and tell it the heartbeat is already running:

```bash
docker compose -f docker-compose.yml -f docker-compose.local-px4.yml exec \
  -e DRONE_SIM_SKIP_STATUS_GATE=1 \
  -e DRONE_SIM_EXTERNAL_HEARTBEAT=1 \
  sim bash /workspace/drone-tests/docker/run_test.sh test_takeoff_land.py
```

Use this only for the Gazebo/Docker simulation. Do not use these bypass flags on the real drone.

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
