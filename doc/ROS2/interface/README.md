# TurtleSim turtle_teleop_key message interface

## turtle_teleop_key

The `turtle_teleop_key` executable is part of the `turtlesim` package, and its used to send commands to the `turtlesim` for it to move.
It is launched using the following command:

```bash
ros2 run turtlesim turtle_teleop_key
```

This will per default spawn an new Node with the name `teleop_turtle`, which has the following interfaces:

```yaml
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
    /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```

This information can be found by running the command while the node is running on a separate terminal:

```bash
ros2 node info /teleop_turtle
```

Relevant for this project is the Publisher to the topic `/turtle1/cmd_vel` which uses the type `geometry_msgs/msg/Twist`. Note that `turtle_teleop_key` publishes to the topic on button press, not periodically.

## geometry_msgs/msg/Twist Topic Type

The `geometry_msgs/msg/Twist` topic type is defined as:

```yaml
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
        float64 x
        float64 y
        float64 z
Vector3  angular
        float64 x
        float64 y
        float64 z
```

This information can be found by running the command:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

## turtlesim/action/RotateAbsolute Action

The `turtlesim` has `RotateAbsolute` action server. To learn more about actions see [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html). This Action has the following interface/type:

```bash
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```

This information can be found by running the command:

```bash
ros2 interface show turtlesim/action/RotateAbsolute
```

The section of this message above the first --- is the structure (data type and name) of the goal request. The next section is the structure of the result. The last section is the structure of the feedback.
