# ROS 2 and micro-ROS
ROS 2 is a powerful framework for developing robot applications, while micro-ROS extends ROS 2 capabilities to microcontrollers, enabling robotics applications on a wider range of hardware.

## Table of Contents
- [What is ROS 2?](#what-is-ros-2)
- [What is micro-ROS?](#what-is-micro-ros)
- [What has improved since ROS 1](#what-has-improved-since-ros-1)
- [What is micro-ROS?](#what-is-micro-ros)
- [micro-ROS Project Components](#what-is-micro-ros)

## What is ROS 2?
ROS 2 is the second major version of the Robot Operating System (ROS), an open-source framework to build complex robot applications.

Key features of ROS 2 include:
- Distributed Computing: Nodes can be distributed across multiple machines.
- Real-Time Capabilities: Improved support for real-time systems.
- Cross-Platform: Runs on various operating systems, including Linux, Windows, and macOS.
- Modularity: Encourages modularity and reusability of code.

## What has improved since ROS 1
Major advancements from ROS (1) include improved real-time capability, more generic Data Distribution Service (DDS) than previously used TCPROS/UDPROS protocols.

List of key differences and new features:
- Middleware: ROS 2 uses DDS for better performance and QoS.
- Real-Time Support: ROS 2 offers enhanced real-time capabilities.
- Cross-Platform: ROS 2 supports multiple operating systems and RTOS.
- Distributed Computing: ROS 2 enables distributed node execution.
- QoS: ROS 2 provides extensive QoS settings.
- Lifecycle Management: ROS 2 introduces lifecycle management for nodes.
- API Stability: ROS 2 emphasizes stable and versioned APIs.
- Security: ROS 2 includes initial security features.
- Development Tools: ROS 2 builds on the ROS 1 ecosystem with new tools.

## What is micro-ROS?
micro-ROS brings ROS 2 to microcontrollers, allowing developers to run ROS 2 applications on resource-constrained devices. This extension enables the integration of microcontrollers into ROS 2 networks, making it possible to create more complex and distributed robotic systems.

Key features of micro-ROS include:
- [Small Footprint](https://micro.ros.org/docs/concepts/benchmarking/memo_prof/): Optimized for microcontrollers with limited resources.
- [Real-Time Capabilities](https://micro.ros.org/docs/concepts/rtos/): Supports real-time operation on microcontrollers.
- [ROS 2 Compatibility](https://micro.ros.org/docs/overview/ROS_2_feature_comparison/): Fully compatible with ROS 2, allowing seamless integration with larger ROS 2 systems. See 
- [Cross-Platform](https://micro.ros.org/docs/overview/hardware/): Supports a variety of microcontroller platforms and RTOS (Real-Time Operating Systems).

## External micro-ROS build system
micro-ROS supports a variety of external build system beyond the micro_ros_setup tool such as PlatformIO [micro_ros_platformio](https://github.com/micro-ROS/micro_ros_platformio/). The following component diagram depi

![uros_ws](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/feature/ROS2/doc/ROS2/uml/micro-ros_build_system.puml)

# ROS 2 in DCS
This section describes the integration of micro-ROS in the existing workspace, extending it by implementing a ROS 2 node and the micro-ROS stack. The goal is to connect the ZumoComSystem with the DDS network using DDS-XRCE, making the robot controllable via ROS messages to mimic turtle graphics drawing using the robot.

## Component Deployment 
The following diagram illustrates the component deployment of micro-ROS in the DroidControlSystem.

![turtle_sim](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/feature/ROS2/doc/ROS2/uml/turtle_sim.plantuml)


## Installation Steps

* [WSL with Ubuntu](./setup/wsl.md)
* [ROS2 Jazzy](./setup/ROS2_Jazzy.md)