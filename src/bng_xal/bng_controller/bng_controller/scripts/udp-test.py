#!/usr/bin/env python3

import socket
import json
import time
import argparse


def send_test_message(ip, port):
    """Send a test message to the specified IP and port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Create a simple test message matching the expected format
    message = {
        "engine_torque": 100.0,
        "road_wheel_angle": 0.2,
        "brake_torque": 0.0,
        "timestamp": int(time.time()),
    }

    # Convert to JSON and send
    msg_bytes = json.dumps(message).encode("utf-8")
    sock.sendto(msg_bytes, (ip, port))
    print(f"Sent test message to {ip}:{port}: {message}")

    # Close the socket
    sock.close()


def listen_for_messages(ip, port, timeout=10):
    """Listen for messages on the specified IP and port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    sock.settimeout(1.0)  # 1 second timeout for each attempt

    print(f"Listening for messages on {ip}:{port}...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        try:
            data, addr = sock.recvfrom(8192)
            message = json.loads(data.decode("utf-8"))
            print(f"Received message from {addr}: {message}")
        except socket.timeout:
            print(".", end="", flush=True)
            continue
        except json.JSONDecodeError:
            print(f"Received non-JSON data from {addr}: {data}")
        except Exception as e:
            print(f"Error receiving data: {e}")

    print("\nTimeout reached. Closing listener.")
    sock.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test UDP communication.")
    parser.add_argument(
        "--mode",
        choices=["send", "listen"],
        required=True,
        help="Operating mode: 'send' to send a test message, 'listen' to listen for messages",
    )
    parser.add_argument("--ip", default="0.0.0.0", help="IP address to use")
    parser.add_argument("--port", type=int, default=64257, help="Port to use")
    parser.add_argument(
        "--timeout", type=int, default=10, help="Listening timeout in seconds"
    )

    args = parser.parse_args()

    if args.mode == "send":
        send_test_message(args.ip, args.port)
    else:  # listen
        listen_for_messages(args.ip, args.port, args.timeout)
