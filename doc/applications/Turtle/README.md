# Turtle

Turtle receives ROS 2 `geometry_msgs/Twist` messages on `cmd_vel` through a micro-ROS agent and forwards movement commands to the [RadonUlzer](https://github.com/BlueAndi/RadonUlzer) `RemoteControl` application using SerialMuxProt.

It requires a ROS 2 environment and a micro-ROS agent, not an MQTT broker. PlatformIO environments: `TurtleTarget` and `TurtleSim`.

## Context

![Turtle context](https://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/applications/Turtle/Context.puml)

[PlantUML source](./Context.puml) | [ROS 2 integration](../../ROS2/README.md)
