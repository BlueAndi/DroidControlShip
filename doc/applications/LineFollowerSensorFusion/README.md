# LineFollowerSensorFusion

LineFollowerSensorFusion follows a black track while estimating the robot's pose and velocity with an extended Kalman filter. It combines vehicle odometry and gyro data with position measurements from [SpaceShipRadar](https://github.com/NewTec-GmbH/SpaceShipRadar). Fusion starts after an initial SpaceShipRadar pose has been received.

The application exchanges sensor data and commands with the [RadonUlzer](https://github.com/BlueAndi/RadonUlzer) `RemoteControl` application over SerialMuxProt. It receives SpaceShipRadar poses and publishes sensor snapshots and fused poses via MQTT.

PlatformIO environments: `LineFollowerSensorFusionTarget` and `LineFollowerSensorFusionSim`.

## Context

![LineFollowerSensorFusion context](https://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/applications/LineFollowerSensorFusion/Context.puml)

[PlantUML source](./Context.puml) | [Hardware deployment](../../SensorFusion/architecture/DeploymentDiagram_RealWorld_SensorFusion.puml) | [Simulation deployment](../../SensorFusion/architecture/DeploymentDiagram_Simulation_SensorFusion.puml) | [Requirements](../../SensorFusion/SRS/srs_eng.md)
