# omniwheels_controller
## Installation

> **ATTENTION:** These commands assume that you have created a workspace called "ros_ws" in your home folder. If you used a different directory or name, please adjust the commands accordingly.

After installing ROS2 and creating the workspace, clone this repository in your workspace:

```
cd ~/ros_ws/src
git clone https://github.com/Daffin-TW/omniwheels_controller.git
```

## Building

Run the following command to build the package:

```
cd ~/ros_ws
colcon build --symlink-install --event-handlers console_direct+
```

After building the package, open a new terminal and navigate to your workspace. Then, source the overlay by running the following command:

```
source /opt/ros/humble/setup.bash
cd ~/ros_ws
. install/local_setup.bash
```

## Run

```
ros2 launch teleop_twist_joy teleop-launch.py joy_vel:=omniwheel/robot_vel joy_config:='xbox'
```
