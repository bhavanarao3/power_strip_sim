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

        # Connect to MQTT broker (localhost)
        self.mqtt_client.connect("127.0.0.1", 1883, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT")
        client.subscribe("power_strip/command")

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode().strip().upper()
        self.get_logger().info(f"Received command: {command}")

        if command == "ON ALL":
            for i in range(len(self.relay_states)):
                if not self.relay_states[i]:  # Ignore if already ON
                    self.relay_states[i] = True
                    self.get_logger().info(f"Outlet {i+1} turned ON")

        elif command == "OFF ALL":
            for i in range(len(self.relay_states)):
                if self.relay_states[i]:  # Ignore if already OFF
                    self.relay_states[i] = False
                    self.get_logger().info(f"Outlet {i+1} turned OFF")

        elif command == "TOGGLE ALL":
            for i in range(len(self.relay_states)):
                self.relay_states[i] = not self.relay_states[i]
                self.get_logger().info(f"Outlet {i+1} is now {'ON' if self.relay_states[i] else 'OFF'}")

        elif command.startswith("ON") or command.startswith("OFF") or command.startswith("TOGGLE"):
            try:
                outlet = int(command[-1]) - 1  # Convert to zero-based index
                if 0 <= outlet < len(self.relay_states):
                    if command.startswith("ON"):
                        if self.relay_states[outlet]:  # Ignore if already ON
                            self.get_logger().info(f"Outlet {outlet+1} is already ON, ignoring")
                        else:
                            self.relay_states[outlet] = True
                            self.get_logger().info(f"Outlet {outlet+1} turned ON")

                    elif command.startswith("OFF"):
                        if not self.relay_states[outlet]:  # Ignore if already OFF
                            self.get_logger().info(f"Outlet {outlet+1} is already OFF, ignoring")
                        else:
                            self.relay_states[outlet] = False
                            self.get_logger().info(f"Outlet {outlet+1} turned OFF")

                    elif command.startswith("TOGGLE"):
                        self.relay_states[outlet] = not self.relay_states[outlet]
                        self.get_logger().info(f"Outlet {outlet+1} is now {'ON' if self.relay_states[outlet] else 'OFF'}")
                else:
                    self.get_logger().error("Invalid outlet number")
            except ValueError:
                self.get_logger().error("Invalid command format")

        elif command.startswith("STATUS"):
            if command == "STATUS":
                status_msg = " | ".join([f"Outlet {i+1}: {'ON' if state else 'OFF'}" for i, state in enumerate(self.relay_states)])
                self.get_logger().info(f"Current Status: {status_msg}")
            else:
                try:
                    outlet = int(command[6]) - 1  # Extract individual outlet number
                    if 0 <= outlet < len(self.relay_states):
                        self.get_logger().info(f"Outlet {outlet+1} is {'ON' if self.relay_states[outlet] else 'OFF'}")
                    else:
                        self.get_logger().error("Invalid outlet number")
                except ValueError:
                    self.get_logger().error("Invalid STATUS command format")

def main(args=None):
    rclpy.init(args=args)
    node = FakeArduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
