#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from power_strip_sim.srv import SetOutlet, GetOutlet

class PowerStripController(Node):
    def __init__(self):
        super().__init__('power_strip_controller')

        # Clients for service calls
        self.set_client = self.create_client(SetOutlet, '/set_outlet_state')
        self.get_client = self.create_client(GetOutlet, '/get_outlet_status')

    def send_command(self, outlet, state):
        """ Send ON/OFF command to fake Arduino """
        if outlet not in range(4):  # Support for 4 outlets
            print("Invalid outlet number. Use 1, 2, 3, or 4.")
            return
        
        req = SetOutlet.Request()
        req.outlet = outlet
        req.state = state

        self.set_client.wait_for_service()
        future = self.set_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            print(f"Outlet {outlet + 1} set to {'ON' if state else 'OFF'}")
        else:
            print("Failed to set outlet state.")

    def toggle_command(self, outlet):
        """ Toggle command (queries current state and switches it) """
        if outlet not in range(4):  
            print("Invalid outlet number. Use 1, 2, 3, or 4.")
            return
        
        req = GetOutlet.Request()
        req.outlet = outlet

        self.get_client.wait_for_service()
        future = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            new_state = not future.result().state
            self.send_command(outlet, new_state)
        else:
            print("Failed to get outlet status.")

    def query_status(self):
        """ Query and display the status of all outlets """
        print("Checking status of all outlets...")
        for i in range(4):  # 4 outlets
            req = GetOutlet.Request()
            req.outlet = i

            self.get_client.wait_for_service()
            future = self.get_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result():
                state = "ON" if future.result().state else "OFF"
                print(f"Outlet {i + 1} is {state}")
            else:
                print(f"Failed to get status for outlet {i + 1}")

def parse_command(command):
    """ Parse user input and return (action, outlet) """
    command = command.strip().upper()

    if command.startswith("ON") and len(command) == 3:
        outlet = command[2]
        if outlet.isdigit() and int(outlet) in [1, 2, 3, 4]:
            return "ON", int(outlet) - 1

    if command.startswith("OFF") and len(command) == 4:
        outlet = command[3]
        if outlet.isdigit() and int(outlet) in [1, 2, 3, 4]:
            return "OFF", int(outlet) - 1

    if command.startswith("TOGGLE") and len(command) == 7:
        outlet = command[6]
        if outlet.isdigit() and int(outlet) in [1, 2, 3, 4]:
            return "TOGGLE", int(outlet) - 1

    if command == "STATUS":
        return "STATUS", None

    if command == "EXIT":
        return "EXIT", None

    return None, None

def main(args=None):
    rclpy.init(args=args)
    node = PowerStripController()

    while rclpy.ok():
        try:
            command = input("\nEnter command (ON, OFF, TOGGLE, STATUS, EXIT): ").strip()
            action, outlet = parse_command(command)

            if action == "ON":
                node.send_command(outlet, True)
            elif action == "OFF":
                node.send_command(outlet, False)
            elif action == "TOGGLE":
                node.toggle_command(outlet)
            elif action == "STATUS":
                node.query_status()
            elif action == "EXIT":
                break
            else:
                print("Invalid command. Use ON1, OFF2, TOGGLE1, STATUS, or EXIT.")

        except Exception as e:
            print(f"Error: {e}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
