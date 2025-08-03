# Run micro-ROS on ESP32 Target

How setup and use micro-ROS DCS PIO target `TurtleTarget`.

## Build TurtleTarget

Perform standard pio build of the TurtleTarget application. It will pull the [micro_ros_platformio](https://github.com/gabryelreyes/micro_ros_platformio) as library dependency and build scripts.
During the build process the micro_ros_platoformio will build uROS as static library which will be placed in the libdeps directory `.pio/libdeps/TurtleTarget/micro_ros_platformio/libmicroros`.

Create symbolic link to static library. Previous links will be overwritten using the `-f` option (ensure that the relative path is correct from the perspective of the link's location):

```bash
ln -sf ../.pio/libdeps/TurtleTarget/micro_ros_platformio/libmicroros ./lib/libmicroros
```

Verify the target architecture of libmicroros.a is for the ESP32. Command and expected output:

```bash
ar p lib/libmicroros/libmicroros.a | file -
/dev/stdin: ELF 32-bit LSB relocatable, Tensilica Xtensa, version 1 (SYSV), with debug_info, not stripped
```

## Bind USB port with WSL

Make the usb serial port visible to wsl in order to flash the binary using [usbipd](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).

Run the following in an administrator shell:

```bat
usbipd bind --busid <busid>
```

Check available usb devices and sharing status.

```bat
usbipd list
```

![usbipd list](img/wsl_bind_usb.png)

Attach USB bus to wsl shell (no admin rights needed for attaching a shared port)

```bat
usbipd attach --wsl --busid <busid>
```
