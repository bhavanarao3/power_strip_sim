import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimulatedArduino(Node):
    def __init__(self):
        super().__init__('simulated_arduino')
        self.outlet_states = {'outlet1': False, 'outlet2': False, 'outlet3': False}  # Simulate 3 outlets
        self.command_subscriber = self.create_subscription(
            String,
            'arduino_commands',
            self.command_callback,
            10
        )
        self.status_publisher = self.create_publisher(String, 'arduino_status', 10)
        self.get_logger().info("Simulated Arduino Node Initialized.")

    def command_callback(self, msg):
        command = msg.data.strip().split()
        if len(command) < 2:
            self.get_logger().info("Invalid command format. Use: <COMMAND> <outlet_name>")
            return

        action, outlet = command[0], command[1]
        if outlet not in self.outlet_states:
            self.get_logger().info(f"Invalid outlet: {outlet}. Available outlets: {list(self.outlet_states.keys())}")
            return

        if action == "ON":
            if not self.outlet_states[outlet]:
                self.outlet_states[outlet] = True
                self.get_logger().info(f"{outlet} turned ON.")
            else:
                self.get_logger().info(f"{outlet} is already ON.")
        elif action == "OFF":
            if self.outlet_states[outlet]:
                self.outlet_states[outlet] = False
                self.get_logger().info(f"{outlet} turned OFF.")
            else:
                self.get_logger().info(f"{outlet} is already OFF.")
        elif action == "TOGGLE":
            self.outlet_states[outlet] = not self.outlet_states[outlet]
            state = "ON" if self.outlet_states[outlet] else "OFF"
            self.get_logger().info(f"{outlet} toggled to {state}.")
        elif action == "STATUS":
            state = "ON" if self.outlet_states[outlet] else "OFF"
            self.get_logger().info(f"{outlet} is {state}.")
        else:
            self.get_logger().info(f"Unknown command: {action}")

        # Publish the status of all outlets
        status_msg = String()
        status_msg.data = str(self.outlet_states)
        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    simulated_arduino = SimulatedArduino()
    rclpy.spin(simulated_arduino)
    simulated_arduino.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
