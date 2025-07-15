Underwater ROV ROS 2 Control System

This project implements a full control pipeline for an underwater ROV using ROS 2 and LoRa communication. It supports **6-DOF thruster control**, **ballast tank operation**, and **serial communication with an Arduino** that drives the motors and ESCs.

---
##  System Overview

### Ground Side:
- Joystick data is transmitted wirelessly using **LoRa SX1278** to the buoy.

### Buoy + ROV Side:
- **LoRa module (via SPI)** receives data directly on a **Raspberry Pi 5**.
- RPi5 runs ROS 2 nodes to interpret and process data for thruster and ballast control.
- Final PWM values are sent to an Arduino, which controls:
  - 6 bidirectional BLDC motors via ESCs (thrusters)
  - 2 ballast tank motors (for pitch/depth adjustment)

---

## Node Architecture
### 1. `lora_coms`
>  LoRa Communication Node

- Reads data from the **SX1278 LoRa module** via SPI.
- Expects 8 comma-separated float values per packet:

- Parses and publishes to:
- `/rov/thruster_cmd_raw` (`geometry_msgs/Twist`)
- `/rov/ballast_cmd` (`geometry_msgs/Vector3`)

---

### 2. `control_pwm`
>  Control & PWM Mapping Node

- Subscribes to:
- `/rov/thruster_cmd_raw` (Twist)
- `/rov/ballast_cmd` (Vector3)

- Performs:
- **Thruster mixing** via a 6x6 matrix
- **Clipping** the output to `[-1, 1]`
- **Converting to PWM** in the safe range `1100–1900 µs`

- Combines all 8 PWM values:
- 6 for thrusters
- 2 for ballast tanks

- Sends final serial packet to Arduino:

---

### 3. Arduino (ROS-less)
> ⚡ Serial Receiver

- Listens to USB serial (115200 baud) from RPi5.
- Parses the received line and sends proper `writeMicroseconds()` signals to 8 output pins:
- 6 ESC-controlled thruster outputs
- 2 ballast motor controls
- Holds last signal if no data is received (optional failsafe logic can be added).

---





## 🔧 Build & Run

```bash
# Install dependencies
pip install pyserial numpy sx127x

# Build packages
cd ~/ros2_ws
colcon build
source install/setup.bash

# Run nodes
ros2 run lora_coms lora_node
ros2 run control_pwm control_node
