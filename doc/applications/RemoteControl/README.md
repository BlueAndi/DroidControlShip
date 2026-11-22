# RemoteControl

RemoteControl receives commands and motor speed setpoints via MQTT and forwards them to the [RadonUlzer](https://github.com/BlueAndi/RadonUlzer) `RemoteControl` application over SerialMuxProt. It also exchanges status information with the robot.

PlatformIO environments: `RemoteControlTarget` and `RemoteControlSim`.

## Context

![RemoteControl context](https://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/applications/RemoteControl/Context.puml)

[PlantUML source](./Context.puml) | [Target deployment](../../architecture/uml/PhysicalView/TargetDeployment.plantuml) | [Simulation deployment](../../architecture/uml/PhysicalView/SimulationDeployment.plantuml)
