"""Webots launcher"""

# MIT License
#
# Copyright (c) 2022 - 2026 Andreas Merkle (web@blue-andi.de)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

################################################################################
# Imports
################################################################################
import os
import sys
import platform

# pyright: reportUndefinedVariable=false
Import("env")  # pylint: disable=undefined-variable

################################################################################
# Variables
################################################################################
OS_PLATFORM_TYPE_WIN = "Windows"
OS_PLATFORM_TYPE_LINUX = "Linux"
OS_PLATFORM_TYPE_MACOS = "Darwin"
OS_PLATFORM_TYPE = platform.system()

WEBOTS_IP_ADDRESS = env.GetProjectOption("custom_webots_ip_address")  # pylint: disable=undefined-variable
WEBOTS_PROTOCOL = env.GetProjectOption("custom_webots_protocol")  # pylint: disable=undefined-variable

ROBOT_NAME = env.GetProjectOption("custom_webots_robot_name")  # pylint: disable=undefined-variable

ROBOT_SERIAL_RX_CHANNEL = env.GetProjectOption("custom_webots_robot_serial_rx_channel")  # pylint: disable=undefined-variable
ROBOT_SERIAL_TX_CHANNEL = env.GetProjectOption("custom_webots_robot_serial_tx_channel")  # pylint: disable=undefined-variable

PROGRAM_PATH = "$BUILD_DIR/"
PROGRAM_OPTIONS = '--cfgFilePath "../../../data/config/config.json" ' \
    + '--serialRxCh ' + ROBOT_SERIAL_RX_CHANNEL + ' ' \
    + '--serialTxCh ' + ROBOT_SERIAL_TX_CHANNEL + ' ' \
    + '-v'
WEBOTS_CONTROLLER_OPTIONS = '--robot-name=' + ROBOT_NAME + ' ' \
                            '--stdout-redirect' + ' ' \
                            '--ip-address=' + WEBOTS_IP_ADDRESS + ' --protocol=' + WEBOTS_PROTOCOL
WEBOTS_HOME = os.getenv('WEBOTS_HOME')

if WEBOTS_HOME is None:
    print("WEBOTS_HOME environment variable is not set. "
          "Please set it to the Webots installation directory.")
    sys.exit(1)

if OS_PLATFORM_TYPE == OS_PLATFORM_TYPE_WIN:

    WEBOTS_HOME = WEBOTS_HOME.replace('\\', '/')
    WEBOTS_CONTROLLER = "\""
    WEBOTS_CONTROLLER += f"{WEBOTS_HOME}/msys64/mingw64/bin/webots-controller.exe"
    WEBOTS_CONTROLLER += "\""
    PROGRAM_NAME = "${PROGNAME}.exe"

elif OS_PLATFORM_TYPE == OS_PLATFORM_TYPE_LINUX:

    WEBOTS_CONTROLLER = f"{WEBOTS_HOME}/webots-controller"
    PROGRAM_NAME = "${PROGNAME}"

elif OS_PLATFORM_TYPE == OS_PLATFORM_TYPE_MACOS:

    WEBOTS_CONTROLLER = f"{WEBOTS_HOME}/Contents/MacOS/webots-controller"
    PROGRAM_NAME = "${PROGNAME}"

else:
    print(f"OS type {OS_PLATFORM_TYPE} not supported.")
    sys.exit(1)

WEBOTS_LAUNCHER_CMD = WEBOTS_CONTROLLER + ' '\
    + WEBOTS_CONTROLLER_OPTIONS + ' ' \
    + PROGRAM_PATH + PROGRAM_NAME + ' ' \
    + PROGRAM_OPTIONS

################################################################################
# Classes
################################################################################

################################################################################
# Functions
################################################################################

def _abort_missing_slot(protocol, ip, robot_name):
    """Check the robot has an extern slot in Webots, print a clear error and exit if not."""
    import socket

    if protocol == "tcp":
        try:
            s = socket.create_connection((ip, 1234), timeout=2)
            s.close()
        except OSError:
            print(f"Error: Cannot reach Webots at {ip}:1234 via TCP. Is Webots running?")
            raise SystemExit(1)
        print(f"Warning: Cannot verify extern slot for '{robot_name}' over TCP — "
              "make sure the correct world is loaded.")
        return

    import getpass
    ipc_dir = f"/tmp/webots/{getpass.getuser()}/1234/ipc"
    if not os.path.isdir(ipc_dir):
        print("Error: Webots is not running or no world is loaded.")
        raise SystemExit(1)

    ipc_socket = f"{ipc_dir}/{robot_name}/extern"
    if not os.path.exists(ipc_socket):
        available = [n for n in os.listdir(ipc_dir)
                     if os.path.exists(os.path.join(ipc_dir, n, "extern"))]
        slots = ", ".join(available) if available else "(none)"
        print(f"Error: Robot '{robot_name}' has no extern controller slot in the loaded world.")
        print(f"       Available extern slots: {slots}")
        print("       Load a world where the robot has controller \"<extern>\".")
        raise SystemExit(1)

    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(1)
        s.connect(ipc_socket)
        s.close()
    except OSError:
        print(f"Error: Extern slot for '{robot_name}' exists but is already claimed by another controller.")
        raise SystemExit(1)


def check_webots_ready(source, target, env):  # pylint: disable=unused-argument
    """Check that Webots is running and the robot has a free extern controller slot."""
    protocol = env.GetProjectOption("custom_webots_protocol")
    ip = env.GetProjectOption("custom_webots_ip_address")
    robot_name = env.GetProjectOption("custom_webots_robot_name")
    _abort_missing_slot(protocol, ip, robot_name)


def launch_webots_controller(source, target, env):  # pylint: disable=unused-argument
    """Run webots-controller, tolerating SIGSEGV (exit 139) on TCP disconnect.

    webots-controller segfaults in libController.so when the remote Webots host
    closes the TCP connection. The DCS process itself exits cleanly; the crash
    is inside the launcher binary and is harmless.
    """
    import subprocess
    cmd = env.subst(WEBOTS_LAUNCHER_CMD)
    print(cmd)
    result = subprocess.run(cmd, shell=True)
    if result.returncode not in (0, -11, 139):  # 139 = 128 + SIGSEGV(11)
        env.Exit(result.returncode)


################################################################################
# Main
################################################################################

# pylint: disable=undefined-variable
env.AddCustomTarget(
    name="webots_launcher",
    dependencies=PROGRAM_PATH + PROGRAM_NAME,
    actions=[
        check_webots_ready,
        launch_webots_controller,
    ],
    title="Launch",
    description="Launch application with Webots launcher."
)
