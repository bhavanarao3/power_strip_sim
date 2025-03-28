#!/usr/bin/env python3

import pygame
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt

# Initialize pygame
pygame.init()

# Window settings
WIDTH, HEIGHT = 600, 300
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Power Strip Simulator")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Outlet positions for 4 outlets
outlet_positions = [(100, 150), (250, 150), (400, 150), (550, 150)]
outlet_states = [False] * 4  # Supports 4 outlets

class PowerStripVisualizer(Node):
    def __init__(self):
        super().__init__('power_strip_visualizer')

        # Initialize MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Connect to MQTT Broker (localhost)
        self.mqtt_client.connect("127.0.0.1", 1883, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT")
        client.subscribe("power_strip/command")

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode().strip().upper()
        self.get_logger().info(f"Received command: {command}")

        if command == "ON ALL":
            for i in range(len(outlet_states)):
                outlet_states[i] = True

        elif command == "OFF ALL":
            for i in range(len(outlet_states)):
                outlet_states[i] = False

        elif command == "TOGGLE ALL":
            for i in range(len(outlet_states)):
                outlet_states[i] = not outlet_states[i]

        elif command.startswith("ON") or command.startswith("OFF") or command.startswith("TOGGLE"):
            try:
                outlet = int(command[-1]) - 1
                if 0 <= outlet < len(outlet_states):
                    if command.startswith("ON"):
                        outlet_states[outlet] = True
                    elif command.startswith("OFF"):
                        outlet_states[outlet] = False
                    elif command.startswith("TOGGLE"):
                        outlet_states[outlet] = not outlet_states[outlet]
                else:
                    self.get_logger().error("Invalid outlet number")
            except ValueError:
                self.get_logger().error("Invalid command format")

def draw_scene():
    screen.fill(WHITE)

    for i, (x, y) in enumerate(outlet_positions):
        pygame.draw.rect(screen, BLACK, (x - 20, y, 40, 20))  # Outlet box
        pygame.draw.circle(screen, GREEN if outlet_states[i] else RED, (x, y - 30), 10)  # Indicator

    pygame.display.flip()

def main():
    rclpy.init()
    node = PowerStripVisualizer()

    running = True
    while running:
        draw_scene()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()
