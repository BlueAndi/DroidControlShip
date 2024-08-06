# Installation of Webots

## Official Setup Procedure
Follow the official installation guide for Webots on Linux with APT on the 
[Webots installation Page](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt).

> **_NOTE:_**
Some of the steps result in larger package downloads.

## Launching Webots

Try 
```bash
    webots
```

If you get an error like "cannot open Display", try the following:
* close Ubuntu console
* In a Command window run

```bat
    wsl --update
```
* try running Webots again
