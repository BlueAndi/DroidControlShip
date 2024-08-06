# Installation of WSL for ROS2 Jazzy

Each ROS2 distribution targets a specific Linux environment. 
For Jazzy it is *Ubuntu 24.04*. To run it on Windows 10/11, 
the WSL2 feature will be used. 

## Install WSL 
Installing WSL (Linux on Windows) is officially supported by Microsoft.
For details refer to the following
[Microsoft documentation](https://learn.microsoft.com/en-us/windows/wsl/install).

But all you need is this from a windows power shell:

```bat
    wsl --install -d "Ubuntu-24.04"
```

> **_NOTE:_**
A Windows message dialog may popup in the background, to ask for elevated rights. 
Check for such a window if the installation does not progress.

## First time use
A console window should appear with installation messages. At the it will ask you
for a user name and password. Use values your remember as you need this password
later whenever you install software.

![Ubuntu First Time](./img/wsl_ubuntu_1st_start.png)

If no window opens, or you get an error message here, try rebooting. Windows may
require updates to install for WSL. Then start an Ubuntu console from start menu:

![Ubuntu Start Menu](./img/ubuntu_start_menu.png)

## Checking Installation

Check that you have the right distribution installed (Ubuntu 24.04 LTS):

```bash
    No LSB modules are available.
    Distributor ID: Ubuntu
    Description:    Ubuntu 24.04 LTS
    Release:        24.04
    Codename:       noble
```

## New Terminal Window

Is is higly recommended to install the new Microsoft Terminal Window 
if not done already. It is available from

[Github Microsoft Terminal](https://github.com/microsoft/terminal).

It offers tabs, history and embedds all kinds of shells (cmd, powershell, Ubuntu, git ...)

![New Shell](./img/new_shell.png)
