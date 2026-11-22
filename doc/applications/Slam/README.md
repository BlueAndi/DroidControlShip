# Slam

Slam connects the robot to an external TCP server intended for ROS 2 integration. It forwards vehicle telemetry as JSON and sends received linear and angular velocity commands to [RadonUlzer](https://github.com/BlueAndi/RadonUlzer). The application is a communication bridge; it does not itself implement a SLAM algorithm.

It uses the [RadonUlzer](https://github.com/BlueAndi/RadonUlzer) `RemoteControl` application over SerialMuxProt and does not require an MQTT broker. PlatformIO environments: `SlamTarget` and `SlamSim`.

## Context

![Slam context](https://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/applications/Slam/Context.puml)

[PlantUML source](./Context.puml) | [Target deployment](../../architecture/uml/PhysicalView/TargetDeployment.plantuml) | [Simulation deployment](../../architecture/uml/PhysicalView/SimulationDeployment.plantuml)
