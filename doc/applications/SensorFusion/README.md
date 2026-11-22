# SensorFusion

SensorFusion estimates position and orientation using odometry and IMU measurements from the [RadonUlzer](https://github.com/BlueAndi/RadonUlzer) `SensorFusion` application. It processes the measurements with an extended Kalman filter and publishes the resulting position via MQTT.

PlatformIO environments: `SensorFusionTarget` and `SensorFusionSim`.

## Context

![SensorFusion context](https://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/applications/SensorFusion/Context.puml)

[PlantUML source](./Context.puml) | [Filter class diagram](../../architecture/uml/LogicalView/SensorFusion/SensorFusionClassDiagram.puml) | [Kalman model](../../architecture/uml/LogicalView/SensorFusion/SensorFusionKalmanModel.md)
