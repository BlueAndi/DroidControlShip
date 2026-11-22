# ConvoyFollower

ConvoyFollower follows waypoints received from its predecessor. It uses vehicle telemetry to control heading and spacing, applies collision avoidance and forwards waypoints to the next follower.

It communicates with the [RadonUlzer](https://github.com/BlueAndi/RadonUlzer) `RemoteControl` application using SerialMuxProt and with the convoy through MQTT. PlatformIO environments: `ConvoyFollowerTarget` and `ConvoyFollowerSim`.

## Context

![ConvoyFollower context](https://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/applications/ConvoyFollower/Context.puml)

[PlantUML source](./Context.puml) | [Convoy deployment](../../architecture/uml/PhysicalView/ConvoyDeployment.plantuml) | [Existing platoon context](../../architecture/uml/LogicalView/Platoon/SystemContextDiagram.puml)
