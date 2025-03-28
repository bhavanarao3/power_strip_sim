#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt

class FakeArduino(Node):
    def __init__(self):
        super().__init__('fake_arduino')

        self.relay_states = [False] * 4  # Now supports 4 outlets

        # Initialize MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Connect to the Mosquitto broker (your local machine)
        self.mqtt_client.connect("127.0.0.1", 1883, 60)  # Replace with your ROS2 IP
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT")
        client.subscribe("power_strip/command")

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode().strip().upper()
        self.get_logger().info(f"Received command: {command}")

        if command.startswith("ON") or command.startswith("OFF") or command.startswith("TOGGLE"):
            try:
                outlet = int(command[-1]) - 1  # Convert to zero-based index
                if 0 <= outlet < len(self.relay_states):
                    if command.startswith("ON"):
                        self.relay_states[outlet] = True
                    elif command.startswith("OFF"):
                        self.relay_states[outlet] = False
                    elif command.startswith("TOGGLE"):
                        self.relay_states[outlet] = not self.relay_states[outlet]

                    self.get_logger().info(f"Outlet {outlet+1} is now {'ON' if self.relay_states[outlet] else 'OFF'}")
                else:
                    self.get_logger().error("Invalid outlet number")
            except ValueError:
                self.get_logger().error("Invalid command format")

        elif command == "STATUS":
            status_msg = " | ".join([f"Outlet {i+1}: {'ON' if state else 'OFF'}" for i, state in enumerate(self.relay_states)])
            self.get_logger().info(f"Current Status: {status_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeArduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
