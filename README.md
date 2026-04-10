# Sterling Max Drone Test Scripts

ROS 2 / PX4 offboard test scripts for basic movement checks with the Sterling Max drone setup.

## Included scripts

- `test_takeoff_land.py`
- `test_forward_backward.py`
- `test_left_right.py`
- `test_all_directions.py`

## Important note

These scripts publish PX4 offboard topics such as `/fmu/in/offboard_control_mode` and `/fmu/in/trajectory_setpoint`.

That usually means they should run on:

- a companion computer connected to the drone, or
- a ground computer on the same ROS 2 / PX4 network

They usually do **not** get copied directly onto the flight controller itself unless your Sterling Max setup already includes an onboard computer that runs ROS 2 and has access to the PX4 bridge.

## Prerequisites on any new device

Before running these scripts on another device, make sure that device has:

1. Git installed
2. Python 3 installed
3. ROS 2 installed and sourced
4. `px4_msgs` available in the ROS 2 environment
5. Network or physical connection to the drone or companion computer

## Clone on another device

Once this folder is pushed to GitHub, GitLab, or another remote, you can download it on another device with:

```powershell
git clone <YOUR-REPO-URL>
cd "Drone Testing"
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
2. Push it to a private GitHub repo
3. Clone it onto the laptop or companion computer you use with the drone
4. Test `test_takeoff_land.py` first before trying the movement scripts

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
