# Software Requirements Specification (SRS)

## Project Title: Sensor fusion of Localization data for Pololu Zumo32U4

Author: Tobias Haeckel  
Bachelor Thesis

## Table of Contents

- [Software Requirements Specification (SRS)](#software-requirements-specification-srs)
  - [Table of Contents](#table-of-contents)
  - [1. Introduction](#1-introduction)
    - [1.1 Purpose](#11-purpose)
    - [1.2 Product Scope](#12-product-scope)
    - [1.3 Definitions, Acronyms, and Abbreviations](#13-definitions-acronyms-and-abbreviations)
    - [1.4 References](#14-references)
  - [2. Overall Description](#2-overall-description)
    - [2.1 Product Perspective](#21-product-perspective)
    - [2.2 Product Features](#22-product-features)
    - [2.3 Assumptions and Dependencies](#23-assumptions-and-dependencies)
  - [3. Requirements](#3-requirements)
    - [3.1 External Interface Requirements](#31-external-interface-requirements)
    - [3.2 Functional Requirements](#32-functional-requirements)
    - [3.3 Quality Attributes](#33-quality-attributes)
    - [3.4 Design- and Implementation Constraints](#34-design--and-implementation-constraints)
  - [4. Verification](#4-verification)
    - [4.1 Verification Methods](#41-verification-methods)
    - [4.2 Traceability (Requirements → Primary Method)](#42-traceability-requirements--primary-method)
    - [4.3 Test Environment](#43-test-environment)
  - [5. Appendices](#5-appendices)
    - [Appendix A: Glossary](#appendix-a-glossary)
    - [Appendix B: Analysis Models](#appendix-b-analysis-models)
    - [Appendix C: Open Issues (TBD)](#appendix-c-open-issues-tbd)

## 1. Introduction

### 1.1 Purpose

The purpose of this document is to specify the software requirements for the project Sensor fusion of Localization data for Pololu Zumo32U4 with the ZumoComSystem (ESP32). It serves as a guide for design, implementation, verification and later maintenance of the software.
The intended audience includes:

- The supervisor(s) of the bachelor thesis
- The examiner of the bachelor thesis
- The developer of the software
- Future maintainers of the software

### 1.2 Product Scope

The developed software acquires sensor data from the Zumo, integrates external camera-based position data (SpaceShipRadar/OpenCV) via MQTT, and fuses them into a robust estimate of position, orientation, and velocity.  
The solution runs both in the Webots simulation environment and on real hardware.
The data will be provided via MQTT to other systems.

### 1.3 Definitions, Acronyms, and Abbreviations

- **Zumo32U4** – Small tracked robot from Pololu with ATmega32U4 and onboard sensors.
- **ZumoComSystem (ZCS)** – ESP32-based extension board providing communication and additional processing power.
- **SSR (SpaceShipRadar)** – External computer vision system (OpenCV-based) running on PC/Laptop, providing absolute position data via MQTT.
- **IMU (Inertial Measurement Unit)** – LSM6DS33 sensor (accelerometer and gyroscope) integrated on the Zumo32U4.
- **Odometry** – Estimation of traveled distance and rotation based on wheel encoders.
- **Sensor Fusion** – Combination of multiple data sources (e.g., odometry, IMU, vision) into a consistent state estimate.
- **Webots** – Open-source robot simulator used for proof-of-concept evaluation.
- **Pose** – Robot’s position and orientation in 2D space (x, y, orientation).

### 1.4 References

- **SpaceShipRadar (SSR)** – OpenCV-based external localization system:  
  <https://github.com/NewTec-GmbH/SpaceShipRadar>

- **Webots** – Open-source robot simulator:  
  <https://www.cyberbotics.com/>

- **RadonUlzer Applications** – Firmware for Zumo32U4:  
  <https://github.com/BlueAndi/RadonUlzer>

- **DroidControlShip** – Firmware for ESP32 (ZumoComSystem):  
  <https://github.com/BlueAndi/DroidControlShip>

- **Pololu Zumo32U4 User Guide** – Hardware documentation:  
  <https://www.pololu.com/docs/0J63/all>

- **PEP-8** – Style Guide for Python Code:  
  <https://peps.python.org/pep-0008/>

## 2. Overall Description

### 2.1 Product Perspective

The software is intended for use both in a **simulation environment** and on **real hardware**.  
Simulation will be carried out using **Webots**, while the real deployment will run on the **Pololu Zumo32U4 v1.1** in combination with the **ZumoComSystem (ESP32)** developed by NewTec.

The overall system is designed as a **distributed architecture** and comprises the following components:

- **Pololu Zumo32U4 v1.1**

  - Base module with motor control and onboard sensors
  - Firmware: [RadonUlzer](https://github.com/BlueAndi/RadonUlzer), application _RemoteControl_

- **ZumoComSystem (ESP32-based extension board)**

  - Provides additional processing and communication capabilities
  - Firmware: [DroidControlShip](https://github.com/BlueAndi/DroidControlShip)
  - Basis for further development (LineFollower extension and sensor fusion logic)

- **PC/Laptop**
  - Runs the camera-based position detection using [SpaceShipRadar](https://github.com/NewTec-GmbH/SpaceShipRadar) (OpenCV)
  - Provides absolute position, orientation, and velocity via MQTT (topic structure `ssr/#`)

### 2.2 Product Features

The main features of the software include:

- **Integration of Sensor Data**

  - The ZumoComSystem shall receive odometry from the Zumo32U4 via USB (using the RadonUlzer firmware).
  - The ZumoComSystem may receive IMU data from the Zumo32U4 via USB.
  - The ZumoComSystem shall receive absolute position, orientation, and velocity from the SpaceShipRadar via MQTT.

- **Sensor Fusion**

  - The ZumoComSystem shall fuse the received data using appropriate sensor fusion techniques. The data sources include:

    - Odometry data (from Zumo32U4)
    - External position data (from SpaceShipRadar via MQTT)
    - IMU data (from Zumo32U4) may be added to the sensorfusion

- **Data Publishing**

  - The ZumoComSystem shall publish the calculated pose and velocity via MQTT for use by other systems.

- **Simulation and Real-World Operatuion**

  - The ZumoComSystem shall receive absolute position, orientation, and velocity from the SpaceShipRadar via MQTT.
  - The software shall be capable of running in both the Webots simulation environment and on real hardware.

- **Extensibility**
  - The software architecture shall be modular to allow for future extensions, such as adding more sensor sources (e.g., magnetometer) or alternative sensor fusion methods.

> System Overview Diagram (to be added later)

### 2.3 Assumptions and Dependencies

#### 2.3.1 Assumptions

| ID   | Type       | Description                                                                                              | Status                  |
| ---- | ---------- | -------------------------------------------------------------------------------------------------------- | ----------------------- |
| A-01 | Assumption | The ZumoComSystem shall receive odometry at least every 100ms.                                           | open                    |
| A-02 | Assumption | The ZumoComSystem may receive IMU data minimally every 100ms                                             | needs to be implemented |
| A-03 | Assumption | An MQTT broker should be available and reachable by all components.                                      | open                    |
| A-04 | Assumption | The ZumoComSystem shall connect to an MQTT broker                                                        | open                    |
| A-05 | Assumption | The SpaceShipRadar shall provide absolute position data at least every 200ms.                            | open                    |
| A-06 | Assumption | The RadonUlzer firmware on the Zumo32U4 shall stop the robot if SpaceShipRadar sets motor speed to zero. | open                    |
| A-07 | Assumption | The Communication between Zumo32U4 and ZumoComSystem shall be safe and reliable.                         | open                    |
| A-08 | Assumption | The development environment shall be operational: VS Code, PlatformIO (ESP32/Zumo) and Python ≥ 3.10 (host). | open                    |
| A-09 | Assumption | All runtime dependencies shall be installable: MQTT client, OpenCV, Webots.                             | open                    |

#### 2.3.2 Dependencies

| ID   | Type       | Description                                                                       | Status |
| ---- | ---------- | --------------------------------------------------------------------------------- | ------ |
| D-01 | Dependency | Use of external software libraries: RadonUlzer, DroidControlShip, SpaceShipRadar. | active |
| D-02 | Dependency | Simulation environment Webots is provided and compatible with RadonUlzer/DCS.     | ok     |
| D-03 | Dependency | Communication between Zumo32U4 and ZumoComSystem is reliably available.           | ok     |

## 3. Requirements

### 3.1 External Interface Requirements

| ID     | Description                                                                                                   | Verified by | Implemented by |
| ------ | ------------------------------------------------------------------------------------------------------------- | ----------- | -------------- |
| UI-101 | The ZumoComSystem shall publish fused localization data (pose and velocity) via MQTT topic `zumo/<id>/fusion` |             |                |
| UI-102 | The ZumoComSystem shall publish status messages via MQTT topic `zumo/<id>/status`                             |             |                |
| UI-103 | The ZumoComSystem shall publish raw data used for sensor fusion via MQTT topic `zumo/<id>/sensors`            |             |                |
| UI-104 | The ZumoComSystem shall subscribe to external position data via MQTT topic `ssr/<id>`                         |             |                |

### 3.2 Functional Requirements

#### 3.2.1 External Inputs & Communication

| ID       | Description                                                                                                            | Verified by | Implemented by |
| -------- | ---------------------------------------------------------------------------------------------------------------------- | ----------- | -------------- |
| FREQ-101 | The RadonUlzer firmware on the Zumo32U4 may provide IMU data to the ZumoComSystem                                      |             |                |
| FREQ-102 | The ZumoComSystem shall subscribe to the MQTT topic `ssr/<id>` to receive absolute position data                       |             |                |
| FREQ-103 | The ZumoComSystem shall check the integrity of received data (e.g. timestamp, format)                                  |             |                |
| FREQ-104 | Each instance of the ZumoComSystem shall use a unique Zumo ID to differentiate multiple robots in the same simulation. |             |                |

#### 3.2.2 Sensor Fusion

| ID       | Description                                                                                                                                 | Verified by | Implemented by |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------- | ----------- | -------------- |
| FREQ-204 | The ZumoComSystem shall time-align odometry, SSR, (and if enabled IMU measurements) within a configurable latency budget.                   |             |                |
| FREQ-205 | The ZumoComSystem shall fuse odometry with external absolute pose to estimate x, y, orientation, and linear velocity in SI units.           |             |                |
| FREQ-203 | The ZumoComSystem may integrate IMU measurements into the sensor fusion when enabled                                                        |             |                |
| FREQ-204 | The ZumoComSystem shall reject outlier and late measurements based on configurable gating thresholds before applying sensor fusion updates. |             |                |
| FREQ-206 | The ZumoComSystem shall align the track coordinates with the Webots world coordinate frame                                                  |             |                |

#### 3.2.3 Modes of Operation

##### 3.2.3.1 Simulation Mode

| ID       | Description                                                                                                                          | Verified by | Implemented by |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------ | ----------- | -------------- |
| FREQ-301 | In simulation mode, the system shall consume odometry (and IMU) data from the Webots simulation environment via RadonUlzer firmware. |             |                |
| FREQ-302 | In simulation mode the system shall apply the same sensor fusion logic as in real-world mode.                                        |             |                |
| FREQ-303 | In simulation mode, the system may use different parameters for the sensor fusion logic                                              |             |                |

##### 3.2.3.2 Real-World Mode

| ID       | Description                                                                                                     | Verified by | Implemented by |
| -------- | --------------------------------------------------------------------------------------------------------------- | ----------- | -------------- |
| FREQ-311 | In real-world mode, the system shall consume odometry (and IMU) data from the Zumo32U4 via RadonUlzer firmware. |             |                |
| FREQ-312 | In real-world mode, the system shall apply the same sensor fusion logic as in simulation mode.                  |             |                |
| FREQ-313 | In real-world mode, the system may use different parameters for the sensor fusion logic                         |             |                |

#### 3.2.4 Logging and Diagnostics

| ID       | Description                                                                                         | Verified by | Implemented by |
| -------- | --------------------------------------------------------------------------------------------------- | ----------- | -------------- |
| FREQ-401 | The Host System (PC/Laptop) shall be able to log all relevant MQTT topics for offline analysis.     |             |                |
| FREQ-402 | Each recorded log entry shall contain a timestamp and source (Topic).                               |             |                |
| FREQ-403 | The Host system shall record the Ground Truth pose from Webots for testing and validation purposes. |             |                |

#### 3.2.5 Error Handling

##### 3.2.5.1 Functional Requirements

| ID       | Description                                                                                                                                                                                               | Verified by | Implemented by |
| -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------- | -------------- |
| FREQ-501 | The ZumoComSystem shall detect fault and exception conditions.                                                                                                                                            |             |                |
| FREQ-502 | The ZumoComSystem shall publish status messages [per 3.2.5.3](#3253-status--event-codes) via MQTT topic `zumo/<id>/status`.                                                                                      |             |                |
| FREQ-503 | The system shall continue operation for non-critical faults (INFO/WARN) and discard faulty inputs.                                                                                                        |             |                |
| FREQ-504 | The ZumoComSystem shall enter a safe state (ERROR) and stop the sensor fusion when critical faults occur.                                                                                                 |             |                |
| FREQ-505 | The ZumoComSystem shall apply a fallback/recovery strategy (e.g., MQTT reconnect with bounded backoff; resubscribe; resume processing) when a fault is detected and publish corresponding status updates. |             |                |
| FREQ-506 | The ZumoComSystem shall provide the status levels OK, INFO, WARN, and ERROR. Further defined in [3.2.5.2](#3252-status-levels)                                                                                                                |             |                |

**Safe State**:

| ID       | Description                                                           | Verified by | Implemented by |
| -------- | --------------------------------------------------------------------- | ----------- | -------------- |
| FREQ-510 | In safe state (ERROR), the Robot shall stop moving.                   |             |                |
| FREQ-511 | In safe state (ERROR), the ZumoComSystem shall stop sensor fusion.    |             |                |

##### 3.2.5.2 Status Levels

The ZumoComSystem shall provide the following status levels:

| Level     | Meaning / Emission rule                                                                                           |
| --------- | ----------------------------------------------------------------------------------------------------------------- |
| **OK**    | Nominal operation. Emit once after startup self-check, and again after any WARN condition has been cleared. |
| **INFO**  | Informational/diagnostic event with no functional impact (e.g., start/stop, parameter load).                      |
| **WARN**  | Non-critical fault; the system continues in a degraded mode (e.g., external pose temporarily unavailable).        |
| **ERROR** | Critical fault; sensor fusion is stopped and publication of fused data ceases.                                    |

##### 3.2.5.3 Status / Event Codes

| Code        | Description                                                                                      |
| ----------- | ------------------------------------------------------------------------------------------------ |
| **OK-01**   | **System stable** — All components operate within defined limits.                                |
| **OK-02**   | **Sensor fusion active** — Sensor fusion runs without errors and publishes continuously.         |
| **INFO-01** | **System boot initiated** — ZumoComSystem (ESP32) startup has begun.                             |
| **INFO-02** | **System ready** — Boot complete; firmware running.                                              |
| **INFO-03** | **Sensors initialized** — Encoders and IMU detected and configured.                              |
| **INFO-04** | **Filter initialized** — Sensor fusion algorithm started and ready for inputs.                   |
| **INFO-05** | **MQTT connected** — Broker connection established.                                              |
| **INFO-06** | **Configuration loaded** — Mode, filter parameters, limits applied from config.                  |
| **INFO-07** | **Mode switched** — System switched to simulation or real mode.                                  |
| **INFO-08** | **External source recovered** — SSR available again after timeout.                               |
| **INFO-09** | **Calibration complete** — Sensor calibration (e.g., IMU/encoders) finished successfully.        |
| **INFO-10** | **Diagnostic message** — General note without functional impact (e.g., logging started/stopped). |
| **WARN-01** | **Invalid message received** — Schema error or values out of allowed range.                      |
| **WARN-02** | **External pose timeout (SSR)** — No SSR data for the configured interval.                       |
| **WARN-03** | **Short sensor timeout** — Encoders or IMU temporarily not delivering data.                      |
| **WARN-04** | **Latency budget exceeded** — Incoming packets arrive later than allowed.                        |
| **ERR-01**  | **MQTT connection lost** — No connection to broker.                                              |
| **ERR-02**  | **Persistent source timeout** — SSR or sensors beyond the allowed outage duration.               |
| **ERR-03**  | **NaN/Inf in sensor fusion** — Numerical instability detected; sensor fusion stopped.            |
| **ERR-04**  | **Repeated invalid messages** — Threshold of invalid messages exceeded.                          |
| **ERR-05**  | **Reconnect failure** — Multiple attempts to reconnect to broker or source failed.               |

May be extended later.

### 3.3 Quality Attributes

#### 3.3.1 Performance

| ID        | Description                                                                                                 | Verified by | Implemented by |
| --------- | ----------------------------------------------------------------------------------------------------------- | ----------- | -------------- |
| NFR-P-101 | The ZumoComSystem shall make fused results available within < 100 ms end-to-end latency under nominal load. |             |                |
| NFR-P-102 | The ZumoComSystem shall publish fused state at a configurable rate with a minimum of ≥ 10 Hz.               |             |                |
| NFR-P-103 | The ZumoComSystem (ESP32) firmware shall use ≤ 80% of available flash/ROM                                   |             |                |
| NFR-P-104 | The ZumoComSystem (ESP32) shall keep peak RAM usage ≤ 80% of available memory.                              |             |                |

#### 3.3.2 Availability

| ID        | Description                                                                                                                                                                               | Verified by | Implemented by |
| --------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------- | -------------- |
| NFR-A-201 | In real-world mode, the system shall complete ≥ 3 continuous laps on the LineFollowerTrack without any ERROR status events (per 3.2.6.2) and without stopping the …/sensor fusion stream. |             |                |
| NFR-A-202 | In simulation mode (Webots), the system shall run continuously for ≥ 10 minutes without any ERROR status events.|             |                |

#### 3.3.3 Standards & Compliance

| ID       | Description                                                                                                                                                                         | Verified by | Implemented by |
| -------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------- | -------------- |
| COMP-301 | The RadonUlzer and DroidControlShip firmwares shall be implemented in C++11.                                                                                                        |             |                |
| COMP-302 | All C++ code shall conform to the project’s Path Finder Rule coding guideline.                                                                                                      |             |                |
| COMP-303 | All PC/host programs shall be programmed in Python ≥ 3.10.                                                                                                                          |             |                |
| COMP-304 | All Python code shall conform to PEP-8 style guide.                                                                                                                                 |             |                |
| COMP-305 | SI units shall be used for all published and stored numerical data                                                                                                                  |             |                |
| COMP-306 | The [LineFollowerTrack](https://github.com/BlueAndi/RadonUlzer/blob/main/webots/worlds/zumo_with_com_system/LineFollowerTrack.wbt) shall be used as the most basic track for tests. |             |                |

### 3.4 Design- and Implementation Constraints

#### 3.4.1 Distribution

| ID       | Description                                                                                                                                   | Verified by | Implemented by |
| -------- | --------------------------------------------------------------------------------------------------------------------------------------------- | ----------- | -------------- |
| DIST-401 | In **real-world mode**, the system shall support ≤ 2 robots concurrently with unique `<id>` namespaces and no topic cross-talk.               |             |                |
| DIST-402 | In **simulation mode**, the system shall support ≤ 5 robots concurrently with unique `<id>` namespaces and sustained host-logging throughput. |             |                |

## 4. Verification

Verification ensures that all functional and non-functional requirements in Section 3 are met.

### 4.1 Verification Methods

TBD

- **Inspection/Review**:
- **Tests**:
- **Measurements**:
- **Analysis**:

### 4.2 Traceability (Requirements → Primary Method)

TBD

| Category | Requirement IDs | Primary Verification Method |
| -------- | --------------- | --------------------------- |

### 4.3 Test Environment

- **Simulation**: Webots with LineFollowerTrack and multiple Robots (≤ 5)
- **Real-World**: Zumo32U4 + ZumoComSystem, SSR-Camera (≤ 2)

## 5. Appendices

### Appendix A: Glossary

### Appendix B: Analysis Models

### Appendix C: Open Issues (TBD)
