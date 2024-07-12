
# SVAN_CHAMPion
This is a combined repository, with CHAMP planner implemented on SVAN_M2 URDF.
This assumes you have ROS2 and ROS2-control setup with gazebo_ros2_control.
# Setup
```bash
mkdir svan_champ_ws
mkdir svan_champ_ws/src
cd svan_champ_ws
git clone https://github.com/nyannbob/svan_champ.git src
```

# Changes to make before running
- Absolute paths, these files are currently using absolute paths and would need to be changed for this repo to work (look for /home/nyann)
    - svam_sw2/svanm2_description/launch/display.launch.py (although this file has no use as of now)
    - svam_sw2/svanm2_description/urdf/svanm2_cam.urdf (xacro implementation can fix this)

# Steps for running (from src directory)
1. Go back to your workspace
```bash
cd ..
```
2. Build this repository
```bash
colcon build
```
3. Source the setup file
```bash
source install/setup.bash
```
4. Launch the URDF in gazebo
```bash
ros2 launch svanm2_description gazebo.launch.py
```
5. Launch the controller nodes and planner from CHAMP
```bash
ros2 launch champ_config gazebo.launch.py
```
6. Launch the teleop to control the bot
```bash
ros2 launch champ_teleop teleop.launch.py
```

### SLAM and NAV2
7. Launch slam_toolbox 
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/svan_sw2/svanm2_description/config/mapper_params_online_async.yaml use_sim_time:=true
```
8. Launch nav2
```bash
ros2 launch svanm2_description navigation_launch.py use_sim_time:=true
```
# Essential parameters to change in CHAMP code to run other URDFs
- joint_names_, joint_names (contacts_sensor.cpp, quadruped_controller.cpp, message_relay.cpp, state_estimator.cpp)
- gait.yaml (to change gait parameters)
- arm lengths (kinodynamics.hpp)
- odometry/imu values if required (slam.yaml)
