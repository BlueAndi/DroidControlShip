# Micro XRCE-DDS Agent

Sources: [here](https://micro-xrce-dds.docs.eprosima.com/en/latest/index.html)

* [Micro XRCE-DDS Agent](#micro-xrce-dds-agent)
  * [Installation](#installation)
  * [Using the agent with the serial interface](#using-the-agent-with-the-serial-interface)
  * [Using the agent with the UDP interface](#using-the-agent-with-the-udp-interface)
  * [Using the agent with the TCP interface](#using-the-agent-with-the-tcp-interface)
  * [Troubleshooting on WSL environment](#troubleshooting-on-wsl-environment)
  * [Testing the node](#testing-the-node)

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

## Using the agent with the TCP interface

Start the MicroXRCEAgent binary to listen to **TCP** connections:

```bash
./MicroXRCEAgent tcp4 -p 8888 -v 6
[1730295182.773705] info     | TCPv4AgentLinux.cpp | init                     | running...             | port: 8888
[1730295182.774347] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 6
[1730295187.775068] debug    | TCPv4AgentLinux.cpp | recv_message             | [==>> TCP <<==]        | client_key: 0x00000000, len: 16, data:
0000: 80 00 00 00 02 01 08 00 00 0A FF FD 02 00 00 00
[1730295187.775861] debug    | TCPv4AgentLinux.cpp | send_message             | [** <<TCP>> **]        | client_key: 0x00000000, len: 36, data:
0000: 80 00 00 00 06 01 1C 00 00 0A FF FD 00 00 01 0D 58 52 43 45 01 00 01 0F 00 01 0D 00 01 00 00 00
0020: 00 00 00 00
[1730295188.777062] debug    | TCPv4AgentLinux.cpp | recv_message             | [==>> TCP <<==]        | client_key: 0x00000000, len: 24, data:
...
```

## Using the agent with the UDP interface

> [!WARNING]  
> Note: UDP ports on WSL are not working properly if you need to access them outside of the WSL VM. Use TCP with WSL instead.

Start the MicroXRCEAgent binary to listen to UDP connections:

```bash
./MicroXRCEAgent udp4 -p 8888 -v 6
[1724834512.980210] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
[1724834512.981013] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 6
[1724834515.742593] debug    | UDPv4AgentLinux.cpp | recv_message             | [==>> UDP <<==]        | client_key: 0x00000000, len: 24, data:
0000: 80 00 00 00 00 01 10 00 58 52 43 45 01 00 01 0F 61 53 75 C7 81 00 FC 01
[1724834515.742856] info     | Root.cpp           | create_client            | create                 | client_key: 0x615375C7, session_id: 0x81
[1724834515.744803] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x615375C7, address: 127.0.0.1:9903
[1724834515.744916] debug    | UDPv4AgentLinux.cpp | send_message             | [** <<UDP>> **]        | client_key: 0x615375C7, len: 19, data:
0000: 81 00 00 00 04 01 0B 00 00 00 58 52 43 45 01 00 01 0F 00
```

## Troubleshooting on WSL environment

UDP connections [setup wsl](./wsl.md#exposing-wsl-udp-ports-to-the-network) (Requires Win 11).

Update: 2024-10-30: TCP transport is available in DroidControlShip, making UDP usage optional. Use TCP on WSL to avoid the trouble.

## Testing the node

In order to test your node, you can use `ros2 topic list` to list all topics used, or `ros2 topic echo <topic_name>` to listen to incoming data in a specific topic.

Publish some test messages (1Hz twist message):
```bash
ros2 topic pub -r 1.0 cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
