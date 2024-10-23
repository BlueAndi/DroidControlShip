""" Check that the OS is supported for specific applications. """

# MIT License
#
# Copyright (c) 2022 - 2024 Andreas Merkle (web@blue-andi.de)
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
import sys
import platform

Import("env")  # pylint: disable=undefined-variable

################################################################################
# Variables
################################################################################

APP_NAME = env["PIOENV"]

WIN_ONLY_APPLICATIONS = []
MACOS_ONLY_APPLICATIONS = []
LINUX_ONLY_APPLICATIONS = ["TurtleSim"]

OS_PLATFORM_TYPE_WIN = "Windows"
OS_PLATFORM_TYPE_LINUX = "Linux"
OS_PLATFORM_TYPE_MACOS = "Darwin"
OS_PLATFORM_TYPE = platform.system()


if (APP_NAME in WIN_ONLY_APPLICATIONS) and (OS_PLATFORM_TYPE != OS_PLATFORM_TYPE_WIN):
    print("This application is only supported on Windows.")
    sys.exit(1)

elif (APP_NAME in MACOS_ONLY_APPLICATIONS) and (OS_PLATFORM_TYPE != OS_PLATFORM_TYPE_MACOS):
    print("This application is only supported on macOS.")
    sys.exit(1)

elif (APP_NAME in LINUX_ONLY_APPLICATIONS) and (OS_PLATFORM_TYPE != OS_PLATFORM_TYPE_LINUX):
    print("This application is only supported on Linux.")
    sys.exit(1)
else:
    print("OS is supported for this application.")

################################################################################
# Classes
################################################################################

################################################################################
# Functions
################################################################################

################################################################################
# Main
################################################################################
