#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from power_strip_sim.srv import SetOutlet, GetOutlet

class FakeArduino(Node):
    def __init__(self):
        super().__init__('fake_arduino')

        self.relay_states = [False] * 4  # 4 simulated relays

        # Status Publisher
        self.status_pub = self.create_publisher(String, '/power_strip/status', 10)
        self.create_timer(3.0, self.publish_status)

        # Services
        self.create_service(SetOutlet, '/set_outlet_state', self.handle_set_state)
        self.create_service(GetOutlet, '/get_outlet_status', self.handle_get_status)

    def handle_set_state(self, request, response):
        if 0 <= request.outlet < len(self.relay_states):
            self.relay_states[request.outlet] = request.state
            response.success = True
            self.get_logger().info(f"Outlet {request.outlet+1} set to {'ON' if request.state else 'OFF'}")
        else:
            response.success = False
        return response

    def handle_get_status(self, request, response):
        if 0 <= request.outlet < len(self.relay_states):
            response.state = self.relay_states[request.outlet]
        return response

    def publish_status(self):
        status_msg = String()
        status_msg.data = " | ".join([f"Outlet {i+1} is {'ON' if state else 'OFF'}" for i, state in enumerate(self.relay_states)])
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeArduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
