# Using DCS with Webots under ROS2

The page discussed how to integrate DCS with Webots and ROS2.

## Deployment for TurtleSim Example

![turtle_sim_webots](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/feature/ROS2/doc/ROS2/uml/turtle_sim_webots.plantuml)

## Building and Launching TurtleSim

Use the ROS2 Package wrapper `ros2_dcs_turtlesim` to build and launch
the TurtleSim demo with DroidControlShip and RadonUlzer robots.
The `ros2_dcs_turtlesim` package is availabe from it's own GIT repository at
[https://github.com/nhjschulz/ros2_dcs_turtlesim](https://github.com/nhjschulz/ros2_dcs_turtlesim).

Create a Webots ros2 workspace as described in [setup/Webots.md](../setup/Webots.md),
then follow the instructions in `ros2_dcs_turtlesim` readme.md for building and launching.

