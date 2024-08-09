# Micro XRCE-DDS Agent

Sources: [here](https://micro-xrce-dds.docs.eprosima.com/en/latest/index.html)

- [Installation](#installation)
- [Using the agent with the serial interface](#using-the-agent-with-the-serial-interface)
- [Using the agent with the wifi interface](#using-the-agent-with-the-wifi-interface)
- [Testing the node](#testing-the-node)

## Installation

Follow the instructions detailed [here](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html). It has only been tested as standalone executable (without Docker neither using Snap).
`cmake` and `make` are required to build the Agent.

## Using the agent with the serial interface

Run the agent with the command:

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0
```

Remember to change `/dev/ttyUSB0` to the correct port where the client is found.

Once the Agent and the Client are connected, the terminal should show something like this:

```bash
[1723186868.281353] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1723186868.282252] info     | Root.cpp              | set_verbose_level        | logger setup           | verbose_level: 4
[1723186868.511335] info     | Root.cpp              | create_client            | create                 | client_key: 0x64C59DFF, session_id: 0x81
[1723186868.511388] info     | SessionManager.hpp    | establish_session        | session established    | client_key: 0x64C59DFF, address: 0
[1723186868.534031] info     | ProxyClient.cpp       | create_participant       | participant created    | client_key: 0x64C59DFF, participant_id: 0x000(1)
[1723186868.551419] info     | ProxyClient.cpp       | create_topic             | topic created          | client_key: 0x64C59DFF, topic_id: 0x000(2), participant_id: 0x000(1)
[1723186868.561392] info     | ProxyClient.cpp       | create_publisher         | publisher created      | client_key: 0x64C59DFF, publisher_id: 0x000(3), participant_id: 0x000(1)
[1723186868.574254] info     | ProxyClient.cpp       | create_datawriter        | datawriter created     | client_key: 0x64C59DFF, datawriter_id: 0x000(5), publisher_id: 0x000(3)
```

## Using the agent with the wifi interface

The `wifi` transport layer has not been tested yet.

## Testing the node

In order to test your node, you can use `ros2 topic list` to list all topics used, or `ros2 topic echo <topic_name>` to listen to incoming data in a specific topic.
