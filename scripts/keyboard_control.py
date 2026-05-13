#!/usr/bin/env python3
"""
Keyboard remote control for the Zumo robot via MQTT.

Controls:
    W / Up    - Forward
    S / Down  - Backward
    A / Left  - Turn left
    D / Right - Turn right
    Space     - Stop
    Q / Esc   - Quit

Usage:
    python3 keyboard_control.py [--broker <host>] [--port <port>] [--robot <name>] [--speed <mm/s>]

The robot name must match the DCS config (data/config/config.json "robotName").
If robotName is empty in the config, DCS derives it from its process ID — set a
fixed name in the config and pass it here.
"""

import argparse
import json
import sys
import termios
import tty

import paho.mqtt.client as mqtt

DEFAULT_BROKER = "localhost"
DEFAULT_PORT = 1883
DEFAULT_ROBOT = "Zumo"
DEFAULT_SPEED = 100  # mm/s

TOPIC_MOTOR_SPEEDS = "{robot}/motorSpeeds"
TOPIC_CMD = "{robot}/cmd"

CMD_ID_START_DRIVING = 5


def parse_args():
    """Parse command-line arguments and return the populated namespace."""
    p = argparse.ArgumentParser(
        description="Keyboard MQTT remote control for Zumo robot")
    p.add_argument("--broker", default=DEFAULT_BROKER, help="MQTT broker host")
    p.add_argument("--port",   type=int, default=DEFAULT_PORT,
                   help="MQTT broker port")
    p.add_argument("--robot",  default=DEFAULT_ROBOT,
                   help="Robot name (MQTT client ID prefix)")
    p.add_argument("--speed",  type=int, default=DEFAULT_SPEED,
                   help="Drive speed in mm/s")
    return p.parse_args()


def publish_motor_speeds(client, topic, left, right):
    """Publish left/right motor speeds (mm/s) as JSON to the given MQTT topic."""
    payload = json.dumps({"LEFT": left, "RIGHT": right})
    client.publish(topic, payload)


def publish_start_driving(client, cmd_topic):
    """Send CMD_ID_START_DRIVING to the robot's command topic, enabling motor control."""
    payload = json.dumps({"CMD_ID": CMD_ID_START_DRIVING})
    client.publish(cmd_topic, payload)


def on_connect(client, userdata, flags, reason_code, properties):
    """MQTT on-connect callback — sends START_DRIVING on success, exits on failure."""
    if reason_code == 0:
        print(
            f"Connected to MQTT broker at {userdata['broker']}:{userdata['port']}")
        # Tell DCS to enter driving mode
        publish_start_driving(client, userdata["cmd_topic"])
    else:
        print(f"Connection failed with code {reason_code}", file=sys.stderr)
        sys.exit(1)


def get_char():
    """Read a single character from stdin without requiring Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Handle escape sequences (arrow keys)
        if ch == "\x1b":
            ch2 = sys.stdin.read(1)
            if ch2 == "[":
                ch3 = sys.stdin.read(1)
                return "\x1b[" + ch3
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    """Connect to the MQTT broker and run the keyboard control loop until quit."""
    args = parse_args()

    motor_topic = TOPIC_MOTOR_SPEEDS.format(robot=args.robot)
    cmd_topic = TOPIC_CMD.format(robot=args.robot)

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id=f"keyboard_control_{args.robot}",
    )
    client.user_data_set({
        "broker":      args.broker,
        "port":        args.port,
        "cmd_topic":   cmd_topic,
    })
    client.on_connect = on_connect

    try:
        client.connect(args.broker, args.port, keepalive=60)
    except Exception as e:
        print(f"Cannot connect to broker: {e}", file=sys.stderr)
        sys.exit(1)

    client.loop_start()

    speed = args.speed

    print()
    print("=== Zumo Keyboard Control ===")
    print(f"  Robot : {args.robot}")
    print(f"  Broker: {args.broker}:{args.port}")
    print(f"  Speed : {speed} mm/s")
    print()
    print("  W/Up    – Forward")
    print("  S/Down  – Backward")
    print("  A/Left  – Turn left (in place)")
    print("  D/Right – Turn right (in place)")
    print("  Space   – Stop")
    print("  Q/Esc   – Quit")
    print()

    try:
        current = (0, 0)

        while True:
            ch = get_char().lower()

            if ch in ("q", "\x1b"):
                break

            if ch in ("w", "\x1b[a"):      # W or Up arrow
                left, right = speed, speed
                label = "FORWARD"
            elif ch in ("s", "\x1b[b"):    # S or Down arrow
                left, right = -speed, -speed
                label = "BACKWARD"
            elif ch in ("a", "\x1b[d"):    # A or Left arrow
                left, right = -speed, speed
                label = "LEFT"
            elif ch in ("d", "\x1b[c"):    # D or Right arrow
                left, right = speed, -speed
                label = "RIGHT"
            elif ch == " ":
                left, right = 0, 0
                label = "STOP"
            else:
                continue

            if (left, right) != current:
                current = (left, right)
                publish_motor_speeds(client, motor_topic, left, right)
                print(
                    f"\r  [{label}]  L={left:+5d} mm/s  R={right:+5d} mm/s    ", end="", flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        publish_motor_speeds(client, motor_topic, 0, 0)
        client.loop_stop()
        client.disconnect()
        print("\nStopped.")


if __name__ == "__main__":
    main()
