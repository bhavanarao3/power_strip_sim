import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PowerstripController(Node):
    def __init__(self):
        super().__init__('powerstrip_controller')
        self.command_publisher = self.create_publisher(String, 'arduino_commands', 10)
        self.status_subscriber = self.create_subscription(
            String,
            'arduino_status',
            self.status_callback,
            10
        )
        self.get_logger().info("Powerstrip Controller Node Initialized.")

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Sent command: {command}")

    def status_callback(self, msg):
        self.get_logger().info(f"Arduino Status: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    controller = PowerstripController()

    def user_interface():
        while rclpy.ok():
            print("\nCommands:")
            print("1. Turn ON outlet (Format: ON outlet1)")
            print("2. Turn OFF outlet (Format: OFF outlet1)")
            print("3. Toggle outlet (Format: TOGGLE outlet1)")
            print("4. Query outlet status (Format: STATUS outlet1)")
            print("5. Exit")

            command = input("Enter your command: ").strip()
            if command.lower() == "exit":
                break

            controller.send_command(command)
            rclpy.spin_once(controller, timeout_sec=1.0)

    try:
        user_interface()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
