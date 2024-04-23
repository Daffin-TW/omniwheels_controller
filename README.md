# omniwheels_controller

## Executable Nodes
- `joystick_to_cmd_vel`
  - Control the robot using a joystick.
- `uwb`
  - Calculate the robot position using anchor and tag.

## Topics

### Joystick

#### Subscribed Topics
- `joy (sensor_msgs/msg/Joy)`
  - Joystick messages to be translated to velocity commands.

#### Published Topics
- `cmd_vel (geometry_msgs/msg/Twist)`
  - Command velocity messages arising from Joystick commands.

### UWB

#### Subscribed Topics
- `uwb_topic (omniwheel_interfaces/msg/UWBAnchor)`
  - Range between two anchors and one tag.
#### Published Topics
- `uwb_coordinate (geometry/msg/Point32)`
  - Robot coordinate messages.

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
colcon build --symlink-install
```

After building the package, open a new terminal and navigate to your workspace. Then, source the overlay by running the following command:

```
source /opt/ros/humble/setup.bash
cd ~/ros_ws
. install/local_setup.bash
```

### Run

#### Joystick

```
ros2 run joy joy_node --ros-args -p deadzone:=0.2
```

#### Joystick to cmd_vel

Run the computation in another terminal to start publishing robot velocity.
```
ros2 run omniwheels_kinetic joystick_to_cmd_vel
```

#### UWB

```
ros2 run omniwheels_kinetic uwb
```

### Controls

#### Movement
Hold `RIGHT SHOULDER` button to enable the robot movement.
Hold `LEFT SHOULDER` button instead for slower movement.

#### Shooting
Press `X` button to take a shot.
