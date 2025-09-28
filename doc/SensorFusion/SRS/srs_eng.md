# Software Requirements Specification (SRS)
## Project Title: Sensorfusion of Localization data for Pololu Zumo32U4

Version 0.2
Author: Tobias Haeckel
Bachelor Thesis

## Version History
| Name | Date  | Reason for Change  | Version |
|------|--------|---------------------|---------|
|Tobias Haeckel | 2025-10-01 |Init|0.1|

## Table of Contents
- [Software Requirements Specification (SRS)](#software-requirements-specification-srs)
  - [Project Title: Sensorfusion of Localization data for Pololu Zumo32U4](#project-title-sensorfusion-of-localization-data-for-pololu-zumo32u4)
  - [Version History](#version-history)
  - [Table of Contents](#table-of-contents)
- [1. Introduction](#1-introduction)
  - [1.1 Purpose](#11-purpose)
  - [1.2 Product Scope](#12-product-scope)
  - [1.3 Definitions, Acronyms, and Abbreviations](#13-definitions-acronyms-and-abbreviations)
  - [1.4 References](#14-references)
  - [1.5 Overview](#15-overview)
- [2. Overall Description](#2-overall-description)
  - [2.1 Product Perspective](#21-product-perspective)
  - [2.2 Product Features](#22-product-features)
  - [2.3 Assumptions and Dependencies](#23-assumptions-and-dependencies)
    - [2.3.1 Assumptions](#231-assumptions)
    - [2.3.2 Dependencies](#232-dependencies)
- [3. Requirements](#3-requirements)
  - [3.1 External Interface Requirements](#31-external-interface-requirements)
    - [3.1.1 User Interfaces](#311-user-interfaces)
    - [3.1.2 Hardware Interfaces](#312-hardware-interfaces)
    - [3.1.3 Software Interfaces](#313-software-interfaces)
  - [3.2 Functional Requirements](#32-functional-requirements)
    - [3.2.1 External Inputs \& Communication](#321-external-inputs--communication)
    - [3.2.2 Sensor Fusion/Data Fusion](#322-sensor-fusiondata-fusion)
    - [3.2.3 Modes of Operation](#323-modes-of-operation)
      - [Simulation Mode](#simulation-mode)
      - [Real-World Mode](#real-world-mode)
      - [3.2.4 Logging and Diagnostics](#324-logging-and-diagnostics)
    - [3.2.5 Error Handling](#325-error-handling)
      - [3.2.5.1 Functional Requirements](#3251-functional-requirements)
      - [3.2.5.2 Status Levels](#3252-status-levels)
      - [3.2.5.3 Status / Event Codes](#3253-status--event-codes)
  - [3.3 Quality Attributes](#33-quality-attributes)
    - [3.3.1 Performance](#331-performance)
    - [3.3.2 Availability](#332-availability)
    - [3.3.3 Standards \& Compliance](#333-standards--compliance)
  - [3.4 Design- and Implementation Constraints](#34-design--and-implementation-constraints)
    - [3.4.1 Installation](#341-installation)
    - [3.4.2 Distribution](#342-distribution)
- [4. Verification](#4-verification)
  - [4.1 Verification Methods](#41-verification-methods)
  - [4.2 Traceability (Requirements → Primary Method)](#42-traceability-requirements--primary-method)
  - [4.3 Test Environment](#43-test-environment)
- [5. Appendices](#5-appendices)
  - [Appendix A: Glossary](#appendix-a-glossary)
  - [Appendix B: Analysis Models](#appendix-b-analysis-models)
  - [Appendix C: Open Issues (TBD)](#appendix-c-open-issues-tbd)

# 1. Introduction
## 1.1 Purpose 
The purpose of this document is to specify the software requirements for the project *Sensorfusion of Localization data for Pololu Zumo32U4* with the ZumoComSystem (ESP32). It serves as a guide for design, implementation, verification and later maintenance of the software. 
The intended audience includes:
- The supervisor(s) of the Bachelor thesis
- The examiner of the Bachelor thesis
- The developer of the software
- Future maintainers of the software

## 1.2 Product Scope
The developed software acquires sensor data from the Zumo, integrates external camera-based position data (SpaceShipRadar/OpenCV) via MQTT, and fuses them into a robust estimate of position, orientation, and velocity.  
The solution runs both in the Webots simulation environment and on real hardware.
The data will be provided via MQTT to other systems.

## 1.3 Definitions, Acronyms, and Abbreviations
- **Zumo32U4** – Small tracked robot from Pololu with ATmega32U4 and onboard sensors.  
- **ZumoComSystem (ZCS)** – ESP32-based extension board providing communication and additional processing power.  
- **SSR (SpaceShipRadar)** – External computer vision system (OpenCV-based) running on PC/Laptop, providing absolute position data via MQTT.  
- **IMU (Inertial Measurement Unit)** – LSM6DS33 sensor (accelerometer and gyroscope) integrated on the Zumo32U4.  
- **Odometry** – Estimation of traveled distance and rotation based on wheel encoders.  
- **MQTT** – Message Queuing Telemetry Transport, a lightweight publish/subscribe communication protocol.  
- **Sensor Fusion** – Combination of multiple data sources (e.g., odometry, IMU, vision) into a consistent state estimate (e.g., via EKF).  
- **Webots** – Open-source robot simulator used for proof-of-concept evaluation.  
- **Pose** – Robot’s position and orientation in 2D space (x, y, yaw).  

## 1.4 References

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

## 1.5 Overview
This document is structured into the following main sections:

- **Section 1 – Introduction**  
  Defines the purpose, scope, terminology, and references relevant to this project.  

- **Section 2 – Overall Description**  
  Provides the system context, the product perspective, major functions, and assumptions or dependencies.  

- **Section 3 – Requirements**  
  Specifies the functional and non-functional requirements.  
  Subdivided into interface definitions, system features, quality attributes, standards compliance, and implementation constraints.  

- **Section 4 – Verification**  
  Describes the methods and environments used to verify and validate the requirements.  

- **Section 5 – Appendices**  
  Contains supporting information such as glossary, analysis models, and a list of open issues (TBD).  


# 2. Overall Description
## 2.1 Product Perspective
The software is intended for use both in a **simulation environment** and on **real hardware**.  
Simulation will be carried out using **Webots**, while the real deployment will run on the **Pololu Zumo32U4 v1.1** in combination with the **ZumoComSystem (ESP32)** developed by NewTec.

The overall system is designed as a **distributed architecture** and comprises the following components:

- **Pololu Zumo32U4 v1.1**  
  - Base module with motor control and onboard sensors  
  - Firmware: [RadonUlzer](https://github.com/BlueAndi/RadonUlzer), application *RemoteControl*

- **ZumoComSystem (ESP32-based extension board)**  
  - Provides additional processing and communication capabilities  
  - Firmware: [DroidControlShip](https://github.com/BlueAndi/DroidControlShip)  
  - Basis for further development (LineFollower extension and fusion logic)

- **PC/Laptop**  
  - Runs the camera-based position detection using [SpaceShipRadar](https://github.com/NewTec-GmbH/SpaceShipRadar) (OpenCV)  
  - Provides absolute position, orientation, and velocity via MQTT (topic structure `ssr/#`)

## 2.2 Product Features

The main features of the software include:
- **Integration of Sensor Data**  
  - The ZumoComSystem shall receive odometry from the Zumo32U4 via USB (using the RadonUlzer firmware). 
  - (Optional) The ZumoComSystem shall receive IMU data from the Zumo32U4 via USB.
  - The ZumoComSystem shall receive absolute position, orientationm, and velocity from the SpaceShipRadar via MQTT.

- **Sensor Fusion**
  - The ZumoComSystem shall fuse the received data using appropriate sensor fusion techniques. The data sources include:
    - Odometry (from Zumo32U4)
    - (IMU data (from Zumo32U4))
    - External position data (from SpaceShipRadar via MQTT)

- **Data Publishing**
  - The ZumoComSystem shall publish the calculated pose (position and orientation) and velocity via MQTT for use by other systems.

- **Simulation and Real-World Operatuion**
  - The software shall be capable of running in both the Webots simulation environment and on real hardware (Zumo32U4 + ZumoComSystem).

- **Extensibility**
  - The software architecture shall be modular to allow for future extensions, such as adding more sensor sources (e.g., magnetometer) or alternative fusion methods.

>System Overview Diagram (to be added later)

## 2.3 Assumptions and Dependencies
### 2.3.1 Assumptions

| ID | Type | Description | Status |
|----|------|-------------|--------|
| A-01 | Assumption | The ZumoComSystem receives Odometry minimally every 100ms. |  open |
| A-02 | Assumption | The ZumoComSystem receives IMU data minimally every 100ms | needs to be implemented |
| A-03 | Assumption | A MQTT broker is available and reachable by all components. |  open |

### 2.3.2 Dependencies
| ID | Type | Description | Status |
|----|------|-------------|--------|
| D-01 | Dependency | Use of external software libraries: RadonUlzer, DroidControlShip, SpaceShipRadar. | active |
| D-02 | Dependency | Simulation environment *Webots* is provided and compatible with RadonUlzer/DCS. | ok |
| D-03 | Dependency | Communication between Zumo32U4 and ESP32 via MAX3421E is reliably available. | ok |


# 3. Requirements
## 3.1 External Interface Requirements
### 3.1.1 User Interfaces
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| UI-101 | The ZumoComSystem shall publish fused localization data (pose and velocity) via MQTT topic `zumo/<id>/fusion` | | 
| UI-102 | The ZumoComSystem shall publish status messages via MQTT topic `zumo/<id>/status` | |
| UI-103 | The ZumoComSystem shall publish raw data used for fusion via MQTT topic `zumo/<id>/sensors` | |
| UI-104 | The ZumoComSystem shall subscribe to external position data via MQTT topic `ssr/<id>` | |

### 3.1.2 Hardware Interfaces
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| HI-201 | The ZumoComSystem shall receive odometry data from the Zumo32U4 | | |
| HI-202 | (Optional) The ZumoComSystem shall receive IMU data from the Zumo32U4 | | |
| HI-203 | The ZumoComSystem shall connect to a MQTT Broker | | |


### 3.1.3 Software Interfaces
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| SW-301 | [RadonUlzer](https://github.com/BlueAndi/RadonUlzer) (app RemoteControl) on Zumo32U4 provides sensor data to ZumoComSystem| | |
| SW-302 | [DroidControlShip](https://github.com/BlueAndi/DroidControlShip) (app LineFollower) on ZumoComSystem: Data acquisition, fusion engine, MQTT Communication, diagnostics | | |
| SW-303 | [SpaceShipRadar](https://github.com/NewTec-GmbH/SpaceShipRadar) on PC/Laptop provides absolute position data via MQTT | | |
| SW-304 | MQTT Broker (e.g., Mosquitto) for communication between components | | |

## 3.2 Functional Requirements

### 3.2.1 External Inputs & Communication
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| FREQ-101 | The RadonUlmzer firmware on the Zumo32U4 shall provide IMU data to the ZumoComSystem (Optional)| | | 
| FREQ-102 | The ZumoComSystem shall subscribe to the MQTT topic `ssr/<id>` to receive absolute position data | | |
| FREQ-103 | The ZumoComSystem shall check the integrity of received data (e.g. timestamp, format, correct Zumo ID) | | |
| FREQ-104 | Each instance of the ZumoComSystem shall use a unique Zumo ID to differentiate multiple robots in the same simulation. | | |

### 3.2.2 Sensor Fusion/Data Fusion
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| FREQ-204 | The ZumoComSystem shall time-align odometry, SSR, (and if enabled IMU measurements) within a configurable latency budget. | | |
| FREQ-205 | The ZumoComSystem shall fuse odometry with external absolute pose to estimate x, y, yaw, and linear velocity in SI units. | | |
| FREQ-203 (Optional) | The ZumoComSystem shall integrate IMU measurements into the fusion when enabled | | |
| FREQ-204 | The ZumoComSystem shall reject outlier and late measurements based on configurable gating thresholds before applying fusion updates. | | |
| FREQ-206 | The ZumoComSystem shall align the track coordinates with the Webots world coordinate frame | | |

### 3.2.3 Modes of Operation

#### Simulation Mode
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| FREQ-301 | In simulation mode, the system shall consume odometry (and IMU) data from the Webots simulation environment via RadonUlzer firmware. | | |
| FREQ-302 | In simulation mode the system shall apply the same fusion logic as in real-world mode. | | |
| FREQ-303 | In simulation mode, the system may use different parameters for the fusion logic | | |


#### Real-World Mode
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| FREQ-311 | In real-world mode, the system shall consume odometry (and IMU) data from the Zumo32U4 via RadonUlzer firmware. | | |
| FREQ-312 | In real-world mode, the system shall apply the same fusion logic as in simulation mode. | | |
| FREQ-313 | In real-world mode, the system may use different parameters for the fusion logic | | |

#### 3.2.4 Logging and Diagnostics
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| FREQ-401 | The Host System (PC/Laptop) shall be able to log all relevant MQTT topics for offline analysis. | | |
| FREQ-402 | Each recorded log entry shall contain a timestamp and source (Topic). | | |
| FREQ-403 | The Host system shall record the Ground Truth pose from Webots for testing and validation purposes. | | |

### 3.2.5 Error Handling

#### 3.2.5.1 Functional Requirements
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| FREQ-501 | The ZumoComSystem shall detect fault and exception conditions automatically. | | |
| FREQ-502 | The ZumoComSystem shall publish error and status messages(per [3.2.5.3](3.2.5.3)) via MQTT topic `zumo/<id>/status`. | | |
| FREQ-503 | The ZumoComSystem shall trigger a status transition to OK/INFO/WARN/ERROR [per 3.2.5.4](3.2.5.4) | | |
| FREQ-504 | The system shall continue operation for non-critical faults (INFO/WARN) and discard faulty inputs. | | |
| FREQ-505 | The ZumoComSystem shall enter a safe state (ERROR) and stop the fusion when critical faults occur. | | |
| FREQ-506 | The ZumoComSystem shall apply a fallback/recovery strategy (e.g., MQTT reconnect with bounded backoff; resubscribe; resume processing) when a fault is detected and publish corresponding status updates. | | |


#### 3.2.5.2 Status Levels
| Level     | Meaning / Emission rule|
| --------- | ------------------------------------------------------------------------------------------------------------------------- |
| **OK**    | Nominal operation. Emit once after startup self-check, and again after any WARN/ERROR condition has been cleared. |
| **INFO**  | Informational/diagnostic event with no functional impact (e.g., start/stop, parameter load).                          |
| **WARN**  | Non-critical fault; the system continues in a degraded mode (e.g., external pose temporarily unavailable).            |
| **ERROR** | Critical fault; fusion is stopped and publication of fused data ceases (e.g., persistent sensor timeout, NaN/Inf).    |

#### 3.2.5.3 Status / Event Codes

| Code      | Description|
| --------- | --------------------------------------------------------------------------------------------- |
| **OK-01** | **System stable** — All components operate within defined limits.                             |
| **OK-02** | **Fusion active** — Fusion runs without errors and publishes continuously.                    |
| **INFO-01** | **System boot initiated** — ZumoComSystem (ESP32) startup has begun.                             |
| **INFO-02** | **System ready** — Boot complete; firmware running.                                              |
| **INFO-03** | **Sensors initialized** — Encoders and IMU detected and configured.                              |
| **INFO-04** | **Filter initialized** — Fusion algorithm started and ready for inputs.                          |
| **INFO-05** | **MQTT connected** — Broker connection established.                                              |
| **INFO-06** | **Configuration loaded** — Mode, filter parameters, limits applied from config.                  |
| **INFO-07** | **Mode switched** — System switched to simulation or real mode.                                  |
| **INFO-08** | **External source recovered** — SSR available again after timeout.                               |
| **INFO-09** | **Calibration complete** — Sensor calibration (e.g., IMU/encoders) finished successfully.        |
| **INFO-10** | **Diagnostic message** — General note without functional impact (e.g., logging started/stopped). |
| **WARN-01** | **Invalid message received** — Schema error or values out of allowed range. |
| **WARN-02** | **External pose timeout (SSR)** — No SSR data for the configured interval.  |
| **WARN-03** | **Short sensor timeout** — Encoders or IMU temporarily not delivering data. |
| **WARN-04** | **Latency budget exceeded** — Incoming packets arrive later than allowed.   |
| **ERR-01** | **MQTT connection lost** — No connection to broker.                                |
| **ERR-02** | **Persistent source timeout** — SSR or sensors beyond the allowed outage duration. |
| **ERR-03** | **NaN/Inf in fusion** — Numerical instability detected; fusion stopped.            |
| **ERR-04** | **Repeated invalid messages** — Threshold of invalid messages exceeded.            |
| **ERR-05** | **Reconnect failure** — Multiple attempts to reconnect to broker or source failed. |

May be extended later.

## 3.3 Quality Attributes

### 3.3.1 Performance
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| NFR-P-101 | The ZumoComSystem shall make fused results available within < 100 ms end-to-end latency under nominal load. | | |
| NFR-P-102 | The ZumoComSystem shall publish fused state at a configurable rate with a minimum of ≥ 10 Hz. | | |
| NFR-P-103 | The ZumoComSystem (ESP32) firmware shall use ≤ 80% of available flash/ROM | | |
| NFR-P-104 | The ZumoComSystem (ESP32) shall keep peak RAM usage ≤ 80% of available memory. | | |

### 3.3.2 Availability
| ID | Description | Verified by | Implemented by |
|----|-------------|-------------|------------------|
| NFR-A-201 | In real-world mode, the system shall complete ≥ 3 continuous laps on the LineFollowerTrack without any ERROR status events (per 3.2.6.2) and without stopping the …/fusion stream. | | |
| NFR-A-202 | In simulation mode (Webots), the system shall run continuously for ≥ 10 minutes without any ERROR status events and while publishing …/fusion continuously. | | |

### 3.3.3 Standards & Compliance
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| COMP-301 | The RadonUlzer and DroidControlShip firmwares shall be implemented in C++11. | | |
| COMP-302 | All C++ code shall conform to the project’s Path Finder Rule coding guideline. | | |
| COMP-303 | All PC/host programs shall be programmed in Python ≥ 3.10. | | |
| COMP-304 | All Python code shall conform to PEP-8 style guide. | | |
| COMP-305 | SI units shall be used for all published and stored numerical data | | |
| COMP-306 | The [LineFollowerTrack](https://github.com/BlueAndi/RadonUlzer/blob/main/webots/worlds/zumo_with_com_system/LineFollowerTrack.wbt) 
shall be used as the most basic track for tests.| | |


## 3.4 Design- and Implementation Constraints
### 3.4.1 Installation
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| INST-001 | The development environment shall be operational: VS Code, PlatformIO (ESP32/Zumo) and Python ≥ 3.10 (host).| | |
| INST-002 | All runtime dependencies shall be installable: MQTT client, OpenCV, Webots.| | |

### 3.4.2 Distribution
| ID     | Description | Verified by | Implemented by |
|--------|-------------|-------------|------------------|
| DIST-401 | In **real-world mode**, the system shall support ≤ 2 robots concurrently with unique `<id>` namespaces and no topic cross-talk.| | |
| DIST-402 | In **simulation mode**, the system shall support ≤ 5 robots concurrently with unique `<id>` namespaces and sustained host-logging throughput. | | |

# 4. Verification
Verification ensures that all functional and non-functional requirements in Section 3 are met.
## 4.1 Verification Methods
TBD
- **Inspection/Review**:
- **Tests**: 
- **Measurements**:
- **Analysis**:

## 4.2 Traceability (Requirements → Primary Method)
TBD
| Category | Requirement IDs | Primary Verification Method |
|----------|-----------------|-----------------------------|


## 4.3 Test Environment
- **Simulation**: Webots with LineFollowerTrack and multiple Robots (≤ 5)
- **Real-World**: Zumo32U4 + ZumoComSystem, SSR-Camera (≤ 2) 

# 5. Appendices
## Appendix A: Glossary

## Appendix B: Analysis Models

## Appendix C: Open Issues (TBD)