# **Power Strip Simulator (ROS2)**  

This package simulates a **power strip** controlled via **ROS2** and **MQTT**. It provides:  
- A **simulated Arduino node** handling power outlet states  
- A **ROS2 service interface** for controlling outlets  
- A **visual representation** using **pygame**  
- **MQTT integration** for remote control via mobile apps  

---

## **1. Installation & Setup**  

### **Clone & Build the Package**  
Place the package inside the `src/` folder of your ROS2 workspace:  
```bash
cd ~/ros2_ws/src
git clone <repository-link> power_strip_sim
```

Navigate to your workspace root, build, and source:  
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## **2. Running the Simulation**  

### **A. Simulating the Arduino Node & ROS2 Commands**  
1️⃣ **Start the Fake Arduino Node:**  
```bash
ros2 run power_strip_sim fake_arduino.py
```
2️⃣ **Start the Power Strip Controller:**  
```bash
ros2 run power_strip_sim power_strip_controller.py
```
💡 **Send Commands (In Terminal 2):**  
Type commands in the format:  
```
ON1   → Turns Outlet 1 ON  
OFF2  → Turns Outlet 2 OFF  
TOGGLE3 → Toggles Outlet 3  
STATUS  → Displays outlet statuses  
```

3️⃣ **Start the Visualization:**  
```bash
ros2 run power_strip_sim power_strip_visual.py
```

---

### **B. Remote Control via MQTT & Mobile App**  

1️⃣ **Start the MQTT Broker (if not running):**  
```bash
sudo systemctl start mosquitto
```

2️⃣ **Run the MQTT Fake Arduino Node:**  
```bash
ros2 run power_strip_sim mqtt_fake_arduino.py
```

3️⃣ **Run the MQTT Power Strip Visualizer:**  
```bash
ros2 run power_strip_sim mqtt_power_strip_visual.py
```

📱 **Mobile Control:**  
Ensure your **system and mobile device are on the same network**.  
Use **IoT MQTT Panel App** (or any MQTT app).  
- Add buttons to send commands to **`power_strip/command`** topic.  
- Example messages: `ON1`, `OFF2`, `TOGGLE3`, `STATUS`.  

---

## **3. Directory Structure**  
```
power_strip_sim/
│── include/                  # Header files (if needed)
│── src/                      # Source files
│   ├── fake_arduino.py       # ROS2 simulated Arduino node
│   ├── power_strip_controller.py  # ROS2 CLI-based controller
│   ├── power_strip_visual.py  # Pygame-based visualizer
│   ├── mqtt_fake_arduino.py   # MQTT-integrated Fake Arduino
│   ├── mqtt_power_strip_visual.py  # MQTT-integrated visualizer
│── srv/                      # Custom service definitions
│   ├── GetOutlet.srv         # Get outlet status
│   ├── SetOutlet.srv         # Set outlet state
│── CMakeLists.txt            # CMake build instructions
│── README.md                 # Documentation
```

---

## **4. Dependencies**  
Ensure you have:  
✅ **ROS2 Humble** installed  
✅ **pygame** for visualization  
```bash
pip install pygame
```  
✅ **Mosquitto MQTT** installed  
```bash
sudo apt install mosquitto mosquitto-clients
```
✅ **paho-mqtt** (for Python MQTT integration)  
```bash
pip install paho-mqtt
```

---

## **5. Future Improvements**  
- Support for **more outlets**  
- **Enhanced UI** with clickable controls  
- Integration with **voice assistants**  

---


