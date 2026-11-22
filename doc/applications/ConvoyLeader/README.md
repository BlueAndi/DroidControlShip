# ConvoyLeader

ConvoyLeader drives the leading robot and publishes waypoints for following vehicles. It uses vehicle telemetry and platoon feedback to limit speed, including collision avoidance and platoon-length control.

It communicates with the [RadonUlzer](https://github.com/BlueAndi/RadonUlzer) `ConvoyLeader` application using SerialMuxProt and with convoy vehicles through MQTT. PlatformIO environments: `ConvoyLeaderTarget` and `ConvoyLeaderSim`.

## Context

![ConvoyLeader context](https://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/applications/ConvoyLeader/Context.puml)

[PlantUML source](./Context.puml) | [Convoy deployment](../../architecture/uml/PhysicalView/ConvoyDeployment.plantuml) | [Existing platoon context](../../architecture/uml/LogicalView/Platoon/SystemContextDiagram.puml)
