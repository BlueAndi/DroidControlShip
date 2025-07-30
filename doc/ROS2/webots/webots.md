# Using DCS with Webots under ROS2

The page discussed how to integrate DCS with Webots and ROS2.

## Deployment for TurtleSim Example

![turtle_sim_webots](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/feature/ROS2/doc/ROS2/uml/turtle_sim_webots.plantuml)

## Building and Launching via Platformio

### RadonUlzer

1. Run the following command in the terminal:

```bash
pio run -e RemoteControlSim -t webots_launcher_zumo_com_system
```

### DroidControlShip

1. Set micro-ROS agent ip address and port in ```data/config.json``` in microROSAgent section.
2. Run the following command in the terminal:

```bash
pio run -e TurtleSim -t webots_launcher
```

## Building and Launching via ROS2 Launcher

Use the ROS2 Package wrapper `ros2_dcs_turtlesim` to build and launch
the TurtleSim demo with DroidControlShip and RadonUlzer robots.
The `ros2_dcs_turtlesim` package is availabe from it's own GIT repository at
[https://github.com/nhjschulz/ros2_dcs_turtlesim](https://github.com/nhjschulz/ros2_dcs_turtlesim).

Create a Webots ros2 workspace as described in [setup/Webots.md](../setup/Webots.md),
then follow the instructions in `ros2_dcs_turtlesim` readme.md for building and launching.

## Notes

If the robot display shows an MCAL error, the calibration for the RadonUlzer is missing. Perform it first or manipulate the settings:

```bash
nano .pio/build/RemoteControlSim/settings.json
```

Set the maxSpeed value to 4200.
