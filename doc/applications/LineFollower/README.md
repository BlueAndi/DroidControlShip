# LineFollower

LineFollower follows a black track using line-sensor readings and motor control. It calibrates the sensors and stops after passing the start/end line twice.

It exchanges sensor data and motor commands with the [RadonUlzer](https://github.com/BlueAndi/RadonUlzer) `RemoteControl` application over SerialMuxProt; it does not require MQTT. PlatformIO environments: `LineFollowerTarget` and `LineFollowerSim`.

## Context

![LineFollower context](https://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/applications/LineFollower/Context.puml)

[PlantUML source](./Context.puml) | [Target deployment](../../architecture/uml/PhysicalView/TargetDeployment.plantuml) | [Simulation deployment](../../architecture/uml/PhysicalView/SimulationDeployment.plantuml)
