# omniwheels_controller

## Topics

### Joystick

#### Subscribed Topics
- `joy (sensor_msgs/msg/Joy)`
  - Joystick messages to be translated to velocity commands.

#### Published Topics
- `omniwheel/robot_vel (geometry_msgs/msg/Twist or geometry_msgs/msg/TwistStamped)`
  - Command velocity messages arising from Joystick commands.

### Kinetic Computation

#### Subscribed Topics
- `omniwheel/robot_vel (geometry_msgs/msg/Twist or geometry_msgs/msg/TwistStamped)`
  - Command velocity messages arising from Joystick commands.
#### Published Topics
- `omniwheel/wheels_vel (omniwheels_interfaces/msg/WheelsVelocity3)`
  - Wheels velocity messages from command veloctiy translation.

## Usage

### Installation

> **ATTENTION:** These commands assume that you have created a workspace called "ros_ws" in your home folder. If you used a different directory or name, please adjust the commands accordingly.

After installing ROS2 and creating the workspace, clone this repository in your workspace:

```
cd ~/ros_ws/src
git clone https://github.com/Daffin-TW/omniwheels_controller.git
```

### Building

Run the following command to build the package:

```
cd ~/ros_ws
colcon build --allow-overriding teleop_twist_joy --symlink-install
```

After building the package, open a new terminal and navigate to your workspace. Then, source the overlay by running the following command:

```
source /opt/ros/humble/setup.bash
cd ~/ros_ws
. install/local_setup.bash
```

### Run

#### Joystick

A launch file has been provided which has three arguments which can be changed in the terminal or via your own launch file.
To configure the node to match your joystick a config file can be used. 
There are several common ones provided in this package (atk3, ps3-holonomic, ps3, xbox, xd3), located here: https://github.com/ros2/teleop_twist_joy/tree/eloquent/config.

PS3 is default, to run for another config (e.g. xbox) use this:
```
ros2 launch teleop_twist_joy teleop-launch.py joy_vel:=omniwheel/robot_vel joy_config:='xbox'
```

__Note:__ this launch file also launches the `joy` node so do not run it separately.


##### Arguments
- `joy_config (string, default: 'ps3')`
  - Config file to use
- `joy_dev (string, default: 'dev/input/js0')`
  - Joystick device to use
- `config_filepath (string, default: '/opt/ros/<rosdistro>/share/teleop_twist_joy/config/' + LaunchConfig('joy_config') + '.config.yaml')`
  - Path to config files
- `publish_stamped_twist (bool, default: false)`
  - Whether to publish `geometry_msgs/msg/TwistStamped` for command velocity messages.

#### Kinetic Computation

Run the computation to start publishing wheel's velocity.
```
ros2 run omniwheels_kinetic omniwheels_vel
```
