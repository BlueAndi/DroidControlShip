# Droid Control Ship - Firmware for ZumoComSystem <!-- omit in toc -->

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](http://choosealicense.com/licenses/mit/)
[![Repo Status](https://www.repostatus.org/badges/latest/wip.svg)](https://www.repostatus.org/#wip)
[![Release](https://img.shields.io/github/release/BlueAndi/DroidControlShip.svg)](https://github.com/BlueAndi/DroidControlShip/releases)
[![Build Status](https://github.com/BlueAndi/DroidControlShip/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/BlueAndi/DroidControlShip/actions/workflows/main.yml)

The Droid Control Ship is the firmware running on the [ZumoComSystem](https://github.com/NewTec-GmbH/ZumoComSystem) hardware, which is a shield for the [Pololu 32U4 Zumo](https://www.pololu.com/product/2510) robot.

It provides different kind of exclusive applications, used for educational purposes:

* Convoy Leader Robot - Based on odometry information, it leads the followers to follow like a lemming trail.
* Convoy Follower Robot - It follows the leader by mainly reacting on the received information.
* Remote Control - Receives commands and motor speeds from an external source through MQTT.

![RobotWithZumoComSystem](doc/images/RobotWithZumoComSystem.jpg)

It supports running in the [Webots simulation](https://www.cyberbotics.com/) too.

The Droid Control Ship communicates with the [Radon Ulzer](https://github.com/BlueAndi/RadonUlzer), which is the firmware running on the Pololu Zumo32U4 robot.

## Table of content

* [The target](#the-target)
* [The simulation](#the-simulation)
  * [Installation](#installation)
    * [Simulation](#simulation)
    * [Target](#target)
* [Documentation](#documentation)
* [Used Libraries](#used-libraries)
  * [Issues, Ideas And Bugs](#issues-ideas-and-bugs)
  * [License](#license)
  * [Contribution](#contribution)

# The target

The main target of the firmware is the [ZumoComSystem](https://github.com/NewTec-GmbH/ZumoComSystem) from NewTec GmbH, which is a shield for the [Pololu 32U4 Zumo](https://www.pololu.com/product/2510) robot.

Together with [Radon Ulzer](https://github.com/BlueAndi/RadonUlzer) it can be run in the [Webots simulation](https://www.cyberbotics.com/) too.

![target-deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/architecture/uml/PhysicalView/TargetDeployment.plantuml)

# The simulation

The simulation is based on the open source robot simulator *Webots*. The application and the services are equal to the target firmware. Only the HAL is different in the simulation.

* Website: <https://cyberbotics.com/#cyberbotics>
* Github: <https://github.com/cyberbotics/webots>

![simulation-deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/architecture/uml/PhysicalView/SimulationDeployment.plantuml)

## Installation

### Simulation

1. Install the native compiler toolchain:
    * Linux
        * Install the gcc toolchain, depended on your distribution.
    * Windows
        * Install the [MSYS2](https://www.msys2.org) toolchain.
        * Open MSYS2 shell.
            * Update package database: ```pacman -Sy pacman```
            * Install GCC: ```pacman -Sy mingw-w64-ucrt-x86_64-gcc```
            * Install Mosquitto: ```pacman -Sy mingw-w64-ucrt-x86_64-mosquitto```
            * Install OpenSSL: ```pacman -S mingw-w64-ucrt-x86_64-openssl```

### Target

1. Make sure that the hardware version of your [ZumoComSystem](https://github.com/NewTec-GmbH/ZumoComSystem) is supported. Currently, only v1.1 and v1.2 are supported.
2. Install the [drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads) for the CP2102 USB-UART converter if required.


# Documentation

* [SW Architecture](./doc/architecture/README.md)
* [SW Configuration](./doc/configuration/README.md)

# Used Libraries

| Library | Description | License |
| - | - | - |
| [Arduino](https://github.com/platformio/platform-espressif32) | ESP32 Arduino framework | Apache-2.0 |
| [Arduino client for MQTT](https://github.com/knolleary/pubsubclient) | This library provides a client for doing simple publish/subscribe messaging with a server that supports MQTT. | MIT |
| [ArduinoJson](https://arduinojson.org/) | JSON handling | MIT |
| [SerialMuxProt](https://github.com/gabryelreyes/SerialMuxProt) | Multiplexing Communication Protocol | MIT |
| [USB_Host_Shield_2.0](https://github.com/NewTec-GmbH/USB_Host_Shield_2.0/tree/3_Endpoints_ACM) | Maxim USB-Host IC driver. Using fork of the [original](https://github.com/felis/USB_Host_Shield_2.0) library that solves issue with endpoints. | GPLv2 |

## Issues, Ideas And Bugs

If you have further ideas or you found some bugs, great! Create a [issue](https://github.com/BlueAndi/DroidControlShip/issues) or if you are able and willing to fix it by yourself, clone the repository and create a pull request.

## License

The whole source code is published under the [MIT license](http://choosealicense.com/licenses/mit/).
Consider the different licenses of the used third party libraries too!

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, shall be licensed as above, without any
additional terms or conditions.
