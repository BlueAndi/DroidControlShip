# SerialMuxProtocol Channels

The following table lists the current SerialMuxProtocol channels being used in the applications, along with their characteristics.
Specific payload implementations can be found in the respective `SerialMuxChannels.h` files for each application


|    Name   | Publisher |     Payload     | Send-type             |                      Comment                      |
|:---------:|:---------:|:---------------:|-----------------------|:-------------------------------------------------:|
| CMD       | DCS       | Command         | Triggered / On-Demand |                                                   |
| CMD_RSP   | RU        | CommandResponse | Triggered / On-Demand | Responds to commands from the CMD channel.        |
| SPEED_SET | DCS       | SpeedData       | Periodic              |                                                   |
| CURR_DATA | RU        | VehicleData     | Periodic              |                                                   |
| STATUS    | DCS / RU  | Status          | Periodic              | Used as an application heartbeat between systems. |