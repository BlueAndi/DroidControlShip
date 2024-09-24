# Static micro-ROS library

## Create a workspace for the library

See https://micro.ros.org/docs/tutorials/core/first_application_linux/

Create a workspace and get the micro-ROS tools:


```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```


## Create static micro-ROS library

See https://micro.ros.org/docs/tutorials/advanced/create_custom_static_library/

Prepare the micro-ROS environment:
```bash
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
```

Copy the files [my_custom_toolchain.cmake](./native_build/my_custom_toolchain.cmake) and [my_custom_colcon.meta](./native_build/my_custom_colcon.meta) in the microros_ws


Now the library can be build by executing
```bash
ros2 run micro_ros_setup build_firmware.sh $(pwd)/my_custom_toolchain.cmake $(pwd)/my_custom_colcon.meta
```

## Link the micro-ROS Library into DroidControlShip

Create a symbolic link from the micro-ROS library to the ```lib``` folder of your DroidControlShip workspace.

Example:
```bash
cd DroidControlShip/lib
ln -s ~/microros_ws/firmware/build libmicroros
```

## Verify binary type of static library
To check the binary type of the static library use this command and see expected output:
```bash
ar p lib/libmicroros/libmicroros.a | file -
/dev/stdin: ELF 64-bit LSB relocatable, x86-64, version 1 (SYSV), with debug_info, not stripped
```
