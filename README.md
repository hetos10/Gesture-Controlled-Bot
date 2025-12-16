# 🤖 Gesture-Controlled Robot – Hand Gesture Recognition with ROS 2
This repository contains a ROS 2-based mobile robot controlled entirely through real-time hand gesture recognition. The robot uses OpenCV for gesture detection and responds to hand gestures to move forward, turn left, turn right, or stop. Works seamlessly in both **Gazebo simulation** and on **real hardware (4WD chassis with ESP32)**.

---

## 📦 Prerequisites

### Software Requirements
* ✅ OS: Ubuntu 22.04 LTS
* ✅ ROS 2: Humble Hawksbill
* ✅ Gazebo Fortress (included in ROS 2 Desktop Full)
* ✅ Python 3.10+
* ✅ USB Webcam / Laptop Camera
* ✅ OpenCV & NumPy

### Hardware Requirements (for Physical Robot)
* ✅ ESP32 Microcontroller
* ✅ 4WD Chassis with Motors
* ✅ L298N Motor Driver
* ✅ 4× BO DC Motors
* ✅ Power Bank / Battery (5V for ESP32, 12V for Motors)
* ✅ USB Cable for ESP32 Programming

---

## 🧰 Step-by-Step Installation

### 1️⃣ Install ROS 2 Humble
Follow the official ROS 2 installation instructions:
```bash
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```

### 2️⃣ Source ROS 2 and Add to `.bashrc`
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3️⃣ Install Gazebo Fortress
```bash
sudo apt install gazebo-fortress
```

### 4️⃣ Set RMW Implementation (CycloneDDS - Optional but recommended)
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

### 5️⃣ Source `.bashrc`
```bash
source ~/.bashrc
```

---

## 🧠 Workspace Setup

### 6️⃣ Install Git (if not already)
```bash
sudo apt install git
```

### 7️⃣ Clone This Repository
```bash
cd ~
mkdir -p bot_ws/src
cd bot_ws/src
git clone https://github.com/hetos_10/bot_ws.git
cd ~/bot_ws
```

---

## ⚙️ Setup Python Dependencies

### 8️⃣ Install OpenCV and Required Libraries
```bash
pip install opencv-python numpy opencv-contrib-python
```

Or install from requirements file:
```bash
pip install -r requirements.txt
```

### 9️⃣ Install ROS Dependencies
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src --skip-keys="ament_python"
```

---

## 🧱 Build the Workspace

```bash
cd ~/bot_ws
colcon build
```

### 🔟 Source the Workspace
```bash
source install/setup.bash
echo "source ~/bot_ws/install/setup.bash" >> ~/.bashrc
```

---

## 🧩 (Optional) Shell Autocompletion
```bash
sudo apt install python3-colcon-argcomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

---

## 🎮 Hand Gesture Controls

| Gesture | Hand Configuration | Robot Action |
|---------|-------------------|--------------|
| ✋ Open Palm | All 5 fingers extended | Move **Forward** |
| ✊ Closed Fist | All fingers closed | **Stop** (Emergency) |
| 👈 Left Hand | Pointer finger raised | Turn **Left** |
| 👉 Right Hand | Pointer + Middle fingers raised | Turn **Right** |

---

## 🚀 Run the Simulation (Gazebo)

### Terminal 1: Launch Gazebo with the Robot
```bash
ros2 launch bot_bringup simulated_robot.launch.py
```

You should now see:
* **Gazebo** window with a simulated robot
* **Webcam feed** in a new OpenCV window showing hand detection
* Robot responding to your **hand gestures** in real-time
* RViz displaying sensor data (optional)

### 🎯 Test the Gestures
1. Open your palm in front of the webcam → Robot moves forward
2. Close your fist → Robot stops
3. Raise One fingers → Robot turns left
4. Raise Two fingers → Robot turns right

---

## 🔧 Run on Hardware (4WD Robot with ESP32)

### Step 1️⃣: Setup Micro-ROS in Arduino IDE

#### Install Arduino IDE (if not already installed)
```bash
# Download and install Arduino IDE
sudo apt install arduino

# Alternatively, download from: https://www.arduino.cc/en/software
```

#### Add ESP32 Board Support to Arduino IDE
```bash
# Open Arduino IDE
arduino

# Go to: File → Preferences
# In "Additional Boards Manager URLs" add this URL:
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# Click OK

# Then go to: Tools → Board → Board Manager
# Search for "esp32" and install "ESP32 by Espressif Systems"
```

#### Install Micro-ROS Library in Arduino IDE
```bash
# In Arduino IDE, go to: Sketch → Include Library → Manage Libraries
# Search for "Micro-ROS"
# Install: "micro_ros_arduino" by Micro-ROS

# Alternative: Download from GitHub
# https://github.com/micro-ROS/micro_ros_arduino
```

#### Arduino IDE Board Configuration for ESP32
```
Tools Menu Settings:
├── Board: "ESP32 Dev Module"
├── Upload Speed: 115200
├── Flash Frequency: 80 MHz
├── Flash Mode: QIO
├── Flash Size: 4 MB
├── Partition Scheme: Default 4MB with spiffs
├── Core Debug Level: None
├── PSRAM: Disabled
└── Port: /dev/ttyUSB0 (or your ESP32 port)
```

**Step-by-step in Arduino IDE:**
1. Click on `Tools` menu
2. Select `Board` → `ESP32 Arduino` → `ESP32 Dev Module`
3. Select `Port` → Choose your USB port (usually `/dev/ttyUSB0`)
4. Select `Upload Speed` → `115200`
5. Ready to upload!

---

### Step 2️⃣: Setup Micro-ROS Agent on Host Machine

### Step 5️⃣: Install Micro-ROS Build Dependencies
```bash
# Update package list
sudo apt update

# Install build tools
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep
sudo apt install -y python3-vcstool
sudo apt install -y build-essential cmake git

# Initialize rosdep
sudo rosdep init
rosdep update
```

#### Step 6️⃣: Create Micro-ROS Workspace
```bash
# Create workspace directory
mkdir -p ~/microros_ws/src
cd ~/microros_ws

# Clone micro-ROS setup repository
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Build the micro-ROS setup
colcon build
source install/local_setup.bash
```

#### Step 7️⃣: Create Micro-ROS Agent
```bash
# Navigate to workspace
cd ~/microros_ws

# Create micro-ROS agent workspace
ros2 run micro_ros_setup create_agent_ws.sh

# Build the agent
colcon build --packages-up-to micro_ros_agent

# Source the agent setup
source install/local_setup.bash
```

#### Step 8️⃣: Add Micro-ROS Agent to `.bashrc` (Optional but Recommended)
```bash
# Add to bashrc for automatic sourcing
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc

# Reload bashrc
source ~/.bashrc
```

#### Step 9️⃣: Verify Micro-ROS Installation
```bash
# Check if agent was installed
which micro_ros_agent

# Expected output: /home/your_username/microros_ws/install/bin/micro_ros_agent
```

### Step 3️⃣: Choose Your Connection Type & Upload

You already have the Arduino sketches in your repository! Choose one based on your connection preference:

#### 🔌 Option A: Serial Connection (USB) - Recommended for Beginners
This is the simplest option. ESP32 connects to your laptop via USB cable.

**Steps:**
1. Open Arduino IDE
2. Go to `File → Open`
3. Navigate to: `~/bot_ws/src/bot_microros/bot_microros_serial.ino`
4. Click Open
5. Verify board settings (Tools menu - see below)
6. Click **Upload (→)** button

**Expected Output in Serial Monitor:**
```
Serial port opened at 115200 baud
Waiting for Micro-ROS Agent...
Connected to Micro-ROS Agent!
Listening for cmd_vel commands...
```

---

#### 📡 Option B: WiFi Connection (Optional) - Wireless
Use this if you want wireless control without USB cable. Requires WiFi network.

**Before Uploading - Edit WiFi Settings:**
1. Open Arduino IDE
2. Go to `File → Open`
3. Navigate to: `~/bot_ws/src/bot_microros/bot_microros_wifi.ino`
4. Click Open
5. Find and edit these lines (around line 15-18):
   ```cpp
   const char * ssid = "YOUR_WIFI_SSID";           // ← Change to your WiFi name
   const char * password = "YOUR_WIFI_PASSWORD";   // ← Change to your WiFi password
   const char * agent_ip = "192.168.1.100";        // ← Change to your ROS2 machine IP
   const int agent_port = 8888;
   ```

**How to find your ROS2 machine IP:**
```bash
# On your laptop (running ROS2)
hostname -I
# Example output: 192.168.1.50
# Use this as agent_ip in the sketch
```

6. Click **Upload (→)** button

**Expected Output in Serial Monitor:**
```
Connecting to WiFi: YOUR_WIFI_SSID
...
WiFi Connected!
IP: 192.168.1.65
Waiting for Micro-ROS Agent...
Connected to Micro-ROS Agent!
```

### Step 4️⃣: Verify Board Settings Before Upload

Make sure these settings are correct in Arduino IDE `Tools` menu:

```
✓ Board: ESP32 Dev Module
✓ Upload Speed: 115200
✓ Flash Frequency: 80 MHz
✓ Flash Mode: QIO
✓ Flash Size: 4 MB
✓ Partition Scheme: Default 4MB with spiffs
✓ Port: /dev/ttyUSB0 (or your USB port)
```

Then click the **Upload (→)** button and wait for "Upload complete"

**If upload fails:**
- Try a different USB cable
- Try different USB port on your computer
- Hold the BOOT button on ESP32 while uploading
- Check if port is correct: `ls -l /dev/ttyUSB*`

---

### Step 5️⃣: Connect ESP32 Hardware
```bash
# Connect ESP32 to laptop via USB cable
# Verify connection
ls -l /dev/ttyUSB*

# Expected output: /dev/ttyUSB0 (or ttyUSB1, etc.)

# Give permissions (if needed)
sudo chmod 666 /dev/ttyUSB0
```

---

### Step 6️⃣: Launch Micro-ROS Agent

#### For Serial Connection (USB)
```bash
# Source micro-ROS setup (if not already in bashrc)
source ~/microros_ws/install/local_setup.bash

# Launch the agent on Serial port (Terminal 1)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Expected output:**
```
[INFO] Micro-ROS Agent started
[INFO] Serial port opened: /dev/ttyUSB0 @ 115200 baud
[INFO] Waiting for client...
[INFO] Client ID: 0x01 connected!
[INFO] Agent created successfully
```

#### For WiFi Connection (Optional)
```bash
# Source micro-ROS setup (if not already in bashrc)
source ~/microros_ws/install/local_setup.bash

# Launch the agent on UDP port for WiFi (Terminal 1)
ros2 run micro_ros_agent micro_ros_agent udp4 --ip 192.168.1.100 -p 8888
```

**Expected output:**
```
[INFO] Micro-ROS Agent started
[INFO] Listening on UDP: 192.168.1.100:8888
[INFO] Waiting for client...
[INFO] Client ID: 0x01 connected!
```

**Verify ESP32 Connection in New Terminal:**
```bash
# Check if the robot node is visible
ros2 node list

# Expected output:
# /esp32_robot_microcontroller       
```

---

### Step 8️⃣: Terminal 2 - Launch Bot Controller
```bash
ros2 launch bot_bringup simulated_robot.launch.py
```

**This will:**
* Initialize motor driver communication with ESP32
* Setup ROS2 publishers/subscribers
* Connect to `/wheel_controller/cmd_vel_unstamped` topic from gesture node


**Expected output:**
```
[INFO] Hand Control Node Started
[INFO] Hand Detection Initialized
[INFO] Publishing to /cmd_vel
[INFO] Waiting for hand gestures...
```

### Step 🔟: Control Your Physical Robot
Make hand gestures in front of the webcam to move your actual 4WD robot!

**Monitoring Command (Optional - Terminal 4):**
```bash

# View all active topics
ros2 topic list

# Check micro-ROS agent connection
ros2 node list
```

---

## 🔌 Hardware Connection Diagram

```
Laptop (ROS2)
    ↓
Micro-ROS Agent (Terminal 1)
    ↓ (Serial /dev/ttyUSB0 @ 115200 baud)
ESP32 Microcontroller
    ↓
L298N Motor Driver
    ↓
4× BO Motors → 4WD Robot Movement
```

---

## 🧪 Hardware Troubleshooting Checklist

```bash
# 1. Check if ESP32 is detected
ls -l /dev/ttyUSB*

# 2. Test serial connection
screen /dev/ttyUSB0 115200
# (Press Ctrl+A then Ctrl+D to exit)

# 3. Verify micro-ROS agent is running
ps aux | grep micro_ros_agent

# 4. Check if /cmd_vel topic exists
ros2 topic list | grep cmd_vel

# 5. Monitor motor commands in real-time
ros2 topic echo /cmd_vel

# 6. Test motors directly with manual command
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# 7. Check ROS2 node connections
ros2 node list
ros2 node info /gesture_control_node
```

---

## 📁 Project Structure Overview
```
bot_ws/
├── src/
│   ├── bot_description/
│   │   ├── launch/
│   │   │   ├── gazebo.launch.py       # Gazebo simulation launcher
│   │   │   └── display.launch.py      # RViz display launcher
│   │   ├── urdf/                      # Robot URDF model files
│   │   └── worlds/                    # Gazebo world files
│   │
│   ├── bot_controller/
│   │   ├── launch/
│   │   │   └── controller.launch.py   # Motor controller launcher
│   │   └── src/                       # Controller implementation
│   │
│   ├── bot_bringup/
│   │   ├── launch/
│   │   │   └── simulated_robot.launch.py  # Main simulation launcher
│   │   └── config/                   # Configuration files
│   │
│   ├── bot_script/
│   │   ├── bot_script/
│   │   │   ├── hand_control.py       # Hand gesture recognition & control
│   │   │   ├── edge_detection.py     # LIDAR edge detection (bonus feature)
│   │   │   └── __init__.py
│   │   ├── setup.py                  # Package setup
│   │   └── test/                     # Unit tests
│   │
│   └── bot_microros/
│       ├── bot_microros_serial.ino   # Micro-ROS firmware (Serial/USB)
│       └── bot_microros_wifi.ino     # Micro-ROS firmware (WiFi)
│
└── README.md
```

---




## 📚 How It Works

### Gesture Recognition Pipeline:
1. **Hand Detection** → OpenCV detects hand landmarks
2. **Gesture Classification** → Compares hand pose against trained patterns
3. **Temporal Smoothing** → Reduces noise and jitter in detection
4. **Command Generation** → Converts gestures to ROS `Twist` messages
5. **Robot Control** → Publishes to `/wheel_controller/cmd_vel_unstamped` for robot movement

---


## 🙌 Credits

This project is maintained by **[Het Chauhan]** - (https://github.com/hetos_10)

**Special thanks to:**
* OpenCV community for computer vision tools
* ROS2 and Gazebo teams for excellent robotics frameworks
* Micro-ROS project for embedded robotics support
* MediaPipe for hand detection models

---

## 📖 References

* [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
* [Gazebo Fortress Docs](https://gazebosim.org/docs/fortress/)
* [OpenCV Documentation](https://docs.opencv.org/)
* [Micro-ROS Guide](https://micro.ros.org/)
* [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)

---

🌟 If this project helped you, please give it a star on GitHub!