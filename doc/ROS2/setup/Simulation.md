# Run micro-ROS on Webots simulation

## RadonUlzer
1. Run the following command in the terminal:
```bash
pio run -e RemoteControlSim -t webots_launcher_zumo_com_system
```

## DroidControlShip
1. Set micro-ROS agent ip address and port in ```data/config.json``` in MQTT section.
2. Run the following command in the terminal:
```bash
pio run -e TurtleSim -t webots_launcher
```

## Notes
* If the display shows an MCAL error, the calibration for the RadonUlzer is missing. Perform it first or manipulate the settings:
    ```bash
    nano .pio/build/RemoteControlSim/settings.json
    ```
    Set the maxSpeed value to 4200.
