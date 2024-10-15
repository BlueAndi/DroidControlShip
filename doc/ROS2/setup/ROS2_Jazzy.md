# Installation of ROS2 Jazzy

## Official Jazzy Setup Procedure
Follow the official installation guide For Ubuntu (Debian packages) on the 
[ROS2 Jazzy Webpage](https://docs.ros.org/en/jazzy/Installation.html).

It boils down to copy+paste the commands from
[Ubuntu-Install-Debs](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
into a ubuntu terminal window.

> **_NOTE:_**
Some of the steps result in larger package downloads.\
Optional parts can be omitted.

## Setup ROS2 environment for every Shell

Add the following line to the end of your ~/.bashrc:

        source /opt/ros/jazzy/setup.bash

```bash
    echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
    source $HOME/.bashrc
```
