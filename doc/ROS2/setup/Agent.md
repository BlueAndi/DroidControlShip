# Micro XRCE-DDS Agent

Sources: [here](https://micro-xrce-dds.docs.eprosima.com/en/latest/index.html)

## Installation

Follow the instructions detailed [here](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html). It has only been tested as standalone executable (without Docker neither using Snap).
`cmake` and `make` are required to build the Agent.

## Using the agent with the serial interface

Run the agent with the command:

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0
```

## Using the agent with the wifi interface

The `wifi` transport layer has not been tested yet.
