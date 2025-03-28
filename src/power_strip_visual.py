#!/usr/bin/env python3

import pygame
import rclpy
from rclpy.node import Node
from power_strip_sim.srv import SetOutlet, GetOutlet

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
outlet_positions = [(100, 150), (250, 150), (400, 150), (550, 150)]  # (x, y)
outlet_states = [False] * 4  # Outlet ON/OFF states

class PowerStripVisualizer(Node):
    def __init__(self):
        super().__init__('power_strip_visual')
        self.set_client = self.create_client(SetOutlet, '/set_outlet_state')
        self.get_client = self.create_client(GetOutlet, '/get_outlet_status')

    def update_outlet_state(self, outlet):
        """Request outlet status from ROS2 service"""
        req = GetOutlet.Request()
        req.outlet = outlet
        self.get_client.wait_for_service()
        future = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            outlet_states[outlet] = future.result().state

def draw_scene():
    screen.fill(WHITE)

    for i, (x, y) in enumerate(outlet_positions):
        # Draw the outlet box
        pygame.draw.rect(screen, BLACK, (x - 20, y, 40, 20))

        # Draw the light bulb above the outlet
        color = GREEN if outlet_states[i] else RED
        pygame.draw.circle(screen, color, (x, y - 30), 10)

    pygame.display.flip()

def main():
    rclpy.init()
    node = PowerStripVisualizer()

    running = True
    while running:
        draw_scene()
        for i in range(4):  # Update all 4 outlets
            node.update_outlet_state(i)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()
