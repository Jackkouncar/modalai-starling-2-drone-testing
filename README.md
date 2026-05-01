# ModalAI Starling 2 Drone Test Scripts

ROS 2 Foxy / PX4 offboard test scripts for basic movement checks with the ModalAI Starling 2 drone setup.

Repository:

`https://github.com/Jackkouncar/modalai-starling-2-drone-testing`

## Quick Start for Teammates

Clone the repo:

```bash
git clone https://github.com/Jackkouncar/modalai-starling-2-drone-testing.git
cd modalai-starling-2-drone-testing
```

Before running anything, source ROS 2 Foxy and the PX4 messages workspace:

```bash
source /opt/ros/foxy/setup.bash
source ~/px4_ros_com_ros2/install/setup.bash
echo $ROS_DISTRO
```

Make sure `echo $ROS_DISTRO` prints:

```bash
foxy
```

Run tests in this order:

```bash
python3 test_takeoff_land.py
python3 test_forward_backward.py
python3 test_left_right.py
python3 test_all_directions.py
```

Start with `test_takeoff_land.py` only. Do not run the movement scripts until takeoff and landing are stable.

Current limits:

- Height: `1.0 m`
- Movement: `1.5 m`
- Square test diagonal reach: about `2.12 m` from the start point
- Soft landing: slow descent to about `0.15 m`, then PX4 land command

Have someone ready to take over or emergency stop during every test.

## Included scripts

- `test_takeoff_land.py`
- `test_forward_backward.py`
- `test_left_right.py`
- `test_all_directions.py`

Shared settings live in `flight_config.py`.

## Current indoor limits

These scripts are currently tuned for indoor testing:

- Takeoff height: `1.0 m`
- Horizontal movement: `1.5 m`
- Soft landing descent: `5.0 s` down to about `0.15 m`, then PX4 land command

Be aware that the square test reaches the corner point `(1.5, 1.5)`, which is about `2.12 m` diagonally from the origin.

To change these limits later, edit `TAKEOFF_HEIGHT_M`, `TRANSIT_DISTANCE_M`, and the soft landing values in `flight_config.py`.

## Important note

These scripts publish PX4 offboard topics such as `/fmu/in/offboard_control_mode` and `/fmu/in/trajectory_setpoint`.

That usually means they should run on:

- a companion computer connected to the drone, or
- a ground computer on the same ROS 2 / PX4 network

They usually do **not** get copied directly onto the flight controller itself unless your ModalAI Starling 2 setup already includes an onboard computer that runs ROS 2 and has access to the PX4 bridge.

## Prerequisites on any new device

Before running these scripts on another device, make sure that device has:

1. Git installed
2. Python 3 installed
3. ROS 2 Foxy installed and sourced
4. `px4_msgs` available in the ROS 2 environment
5. Network or physical connection to the drone or companion computer

These scripts are currently targeted for ROS 2 Foxy. They avoid Humble-specific APIs, and each script logs a warning if `ROS_DISTRO` is not set to `foxy`.

On Ubuntu, source Foxy before running:

```bash
source /opt/ros/foxy/setup.bash
```

If `px4_msgs` is built in a workspace, source that workspace too:

```bash
source ~/px4_ros_com_ros2/install/setup.bash
```

## Clone on another device

Once this folder is pushed to GitHub, GitLab, or another remote, you can download it on another device with:

```powershell
git clone https://github.com/Jackkouncar/modalai-starling-2-drone-testing.git
cd modalai-starling-2-drone-testing
```

## Run a script

Open a terminal where ROS 2 and `px4_msgs` are already available, then run one of:

```powershell
python test_takeoff_land.py
python test_forward_backward.py
python test_left_right.py
python test_all_directions.py
```

## Recommended workflow

For safety and easier recovery:

1. Keep this project in Git
2. Keep the repo private if needed and add collaborators
3. Clone it onto the laptop or companion computer you use with the drone
4. Test `test_takeoff_land.py` first before trying the movement scripts

## Recommended indoor test order

Run the scripts in this order:

1. `test_takeoff_land.py`
2. `test_forward_backward.py`
3. `test_left_right.py`
4. `test_all_directions.py`

Stop after each flight and confirm the drone stayed well inside the room boundaries before moving to the next script.

## Create the repo and first commit

If this is your first time turning the folder into Git, run:

```powershell
git init
git branch -M main
git add .
git commit -m "Initial drone test scripts"
```

## Push to GitHub

After creating an empty repo on GitHub, run:

```powershell
git remote add origin <YOUR-REPO-URL>
git push -u origin main
```

## Extra recommendations

- Keep the repo private if the drone is part of school, work, or proprietary testing.
- Add a short note after each flight test describing what worked and what drifted.
- If you later want a cleaner launch flow, we can convert these into a ROS 2 package so they run with `ros2 run`.
