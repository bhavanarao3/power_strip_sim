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

        # Store the last known state to avoid redundant commands
        self.outlet_states = [None] * 4  # Unknown state initially

    def get_outlet_status(self, outlet):
        """ Query the current status of a specific outlet """
        req = GetOutlet.Request()
        req.outlet = outlet

        self.get_client.wait_for_service()
        future = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.outlet_states[outlet] = future.result().state  # Update local state
            return future.result().state
        else:
            print(f"Failed to get status for outlet {outlet + 1}")
            return None

    def send_command(self, outlet, state):
        """ Send ON/OFF command only if it changes the current state """
        current_state = self.get_outlet_status(outlet)

        if current_state is None:
            return  # Ignore if status retrieval failed

        if current_state == state:
            print(f"Outlet {outlet + 1} is already {'ON' if state else 'OFF'}, ignoring redundant command.")
            return  # Ignore redundant command

        req = SetOutlet.Request()
        req.outlet = outlet
        req.state = state

        self.set_client.wait_for_service()
        future = self.set_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.outlet_states[outlet] = state  # Update local state
            print(f"Outlet {outlet + 1} set to {'ON' if state else 'OFF'}")
        else:
            print("Failed to set outlet state.")

    def toggle_command(self, outlet):
        """ Toggle command (queries current state and switches it) """
        current_state = self.get_outlet_status(outlet)

        if current_state is None:
            return  # Ignore if status retrieval failed

        new_state = not current_state
        self.send_command(outlet, new_state)

    def query_status(self):
        """ Query and display the status of all outlets """
        print("Checking status of all outlets...")
        for i in range(4):  # 4 outlets
            state = self.get_outlet_status(i)
            if state is not None:
                print(f"Outlet {i + 1} is {'ON' if state else 'OFF'}")

def parse_command(command):
    """ Parse user input and return (action, outlet) """
    command = command.strip().upper()

    if command == "ON ALL":
        return "ON_ALL", None
    if command == "OFF ALL":
        return "OFF_ALL", None
    if command == "TOGGLE ALL":
        return "TOGGLE_ALL", None

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

    if command.startswith("STATUS") and len(command) == 7:
        outlet = command[6]
        if outlet.isdigit() and int(outlet) in [1, 2, 3, 4]:
            return "STATUS_SINGLE", int(outlet) - 1

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
            elif action == "STATUS_SINGLE":
                state = node.get_outlet_status(outlet)
                if state is not None:
                    print(f"Outlet {outlet + 1} is {'ON' if state else 'OFF'}")
            elif action == "ON_ALL":
                for i in range(4):  
                    node.send_command(i, True)
            elif action == "OFF_ALL":
                for i in range(4):  
                    node.send_command(i, False)
            elif action == "TOGGLE_ALL":
                for i in range(4):  
                    node.toggle_command(i)
            elif action == "EXIT":
                break
            else:
                print("Invalid command. Use ON1, OFF2, TOGGLE1, STATUS, ON ALL, OFF ALL, TOGGLE ALL, or EXIT.")

        except Exception as e:
            print(f"Error: {e}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
