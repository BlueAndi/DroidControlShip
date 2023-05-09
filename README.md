# Droid Control Ship - ZumoComSystem <!-- omit in toc -->

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](http://choosealicense.com/licenses/mit/)
[![Repo Status](https://www.repostatus.org/badges/latest/wip.svg)](https://www.repostatus.org/#wip)
[![Release](https://img.shields.io/github/release/BlueAndi/DroidControlShip.svg)](https://github.com/BlueAndi/DroidControlShip/releases)
[![Build Status](https://github.com/BlueAndi/DroidControlShip/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/BlueAndi/DroidControlShip/actions/workflows/main.yml)

The Droid Control Ship contains different kind of applications, used for educational purposes:
* Convoy Leader Robot - Based on odometry information, it leads the followers to follow like a lemming trail.
* Convoy Follower Robot - It follows the leader by mainly reacting on the received information.
* etc.

It is designed to run on the [ZumoComSystem](https://github.com/NewTec-GmbH/ZumoComSystem) hardware, which is a shield for the [Pololu 32U4 Zumo Robot](https://www.pololu.com/product/2510). It supports running in the [Webots simulation](https://www.cyberbotics.com/) too.

The Droid Control Ship communicates with the [Radon Ulzer](https://github.com/BlueAndi/RadonUlzer).

- [The target](#the-target)
- [The simulation](#the-simulation)
  - [Installation](#installation)
- [Used Libraries](#used-libraries)
- [Issues, Ideas And Bugs](#issues-ideas-and-bugs)
- [License](#license)
- [Contribution](#contribution)

# The target
The main target of the firmware is the [ZumoComSystem](https://github.com/NewTec-GmbH/ZumoComSystem) from NewTec GmbH, which is a shield for the [Pololu 32U4 Zumo Robot](https://www.pololu.com/product/2510).

Together with [Radon Ulzer](https://github.com/BlueAndi/RadonUlzer) it can be run in the [Webots simulation](https://www.cyberbotics.com/) too.

# The simulation
The simulation is based on the open source robot simulator *Webots*. The application and the services are equal to the target firmware. Only the HAL is different in the simulation.

* Website: https://cyberbotics.com/#cyberbotics
* Github: https://github.com/cyberbotics/webots

## Installation

Follow the [installation description in the Radon Ulzer repository](https://github.com/BlueAndi/RadonUlzer).

# Used Libraries

| Library | Description | License |
| - | - | - |
| ... | ... | ... |

# Issues, Ideas And Bugs
If you have further ideas or you found some bugs, great! Create a [issue](https://github.com/BlueAndi/RadonUlzer/issues) or if you are able and willing to fix it by yourself, clone the repository and create a pull request.

# License
The whole source code is published under the [MIT license](http://choosealicense.com/licenses/mit/).
Consider the different licenses of the used third party libraries too!

# Contribution
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, shall be licensed as above, without any
additional terms or conditions.
