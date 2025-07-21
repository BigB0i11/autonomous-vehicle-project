# 🤖 Autonomous Vehicle Project  
*Applied Electrical & Electronic Engineering Year 1 | University of Nottingham*

![EEEbot](https://github.com/BigB0i11/autonomouse-vehicle-project/blob/main/images/EEEbot.png)

## 🧭 Introduction

This project demonstrates my design and implementation of a functional **autonomous vehicle system**, developed entirely from first principles. It showcases my skills across:

- 📐 Circuit design (H-Bridge, logic gates, op-amps)
- 🧠 Microcontroller programming (ESP32 with Arduino IDE)
- 🔧 Sensor integration (Ultrasonic, IMU, IR sensors)
- 🤝 Master-slave I²C communication
- 🛠 Prototyping, testing, and debugging hardware-software systems

The project was completed over two major phases: **Autonomous driving (10m challenge)** and **Environmental interaction (maze navigation + line following)**.

---

## 🚀 Key Outcomes

| Challenge            | Goal                                       | Outcome                                     |
|----------------------|--------------------------------------------|---------------------------------------------|
| 10m Driving          | Reach 10m autonomously in straight line    | Reached ~5m with speed adjustments          |
| Maze Navigation      | Navigate a maze using sensors              | Partial success (sensor logic worked)       |
| Line Following       | Follow a black line on white surface       | Circuit & software functional, not deployed |

---

## 🧠 System Overview

### Phase 1: Motion & Control

![Motor Setup](https://github.com/BigB0i11/autonomouse-vehicle-project/blob/main/images/motor_circuit.png)

- **ESP32-based control**
- Custom **H-Bridge circuit** for bidirectional motor drive  
- Directional logic built from **NAND gates**
- Dual motor driver **L298N** integration
- PWM speed tuning to correct motion drift  
- Servo for front steering using software-controlled angle logic

---

### Phase 2: Maze Navigation

![Maze Schematic](https://github.com/BigB0i11/autonomouse-vehicle-project/blob/main/images/maze_schematic.png)

- **HC-SR04 Ultrasonic Sensor** for obstacle detection  
- **MPU-6050 IMU** to measure 90° rotation  
- Master-slave communication using **I²C protocol**
- Adaptive software logic: stop, turn, move based on distance & angle

---

### Phase 2: Line Following

![IR Sensor Stripboard](https://github.com/BigB0i11/autonomouse-vehicle-project/blob/main/images/stripboard.png)

- 4x **IR Photodiode + IR LED pairs**
- Signal amplification via **transimpedance op-amp circuits**
- Analog sensor readout for precise threshold control
- Final stripboard optimised for signal strength & alignment

---

## 💻 Programming Highlights

- Modular C++ code using **Arduino IDE** and libraries:  
  - `ESP32Servo`, `ESP32PWM`, `Wire`, `Adafruit_MPU6050`, `HCSR04`
- Implemented:
  - Real-time **servo steering**
  - **PWM speed correction** based on trial + feedback
  - **Distance-triggered state changes** via ultrasonic feedback
  - **I²C communication protocol** between two ESP32 boards
  - **Sensor-threshold-based navigation states** for line following

---

## 🛠 Skills Demonstrated

| Area                     | Examples                                                                 |
|--------------------------|--------------------------------------------------------------------------|
| **Embedded Programming** | PWM, I²C, GPIO control, sensor integration using ESP32 + Arduino IDE     |
| **Circuit Design**       | Custom op-amp amplifier, logic gate circuits, H-Bridge motor controller  |
| **Signal Processing**    | Analog sensor calibration, voltage division, real-time response control  |
| **Debugging**            | Oscilloscope testing, motor tuning, voltage range troubleshooting        |
| **Documentation**        | Two full technical reports with simulations, schematics & test data      |

---

## 📁 Repository Structure

autonomouse-vehicle-project/
├── Code/ # Arduino sketches for all modules
├── images/ # Schematic + hardware photos
├── Reports/ # Full PDF reports for both phases
│ ├── Technical Report 1.pdf
│ └── Technical Report 2.pdf
├── README.md


---

## 📄 Reports

- 📘 [Technical Report 1](./Reports/Technical%20Report%201.pdf) – Core motor control, H-Bridge design, servo integration  
- 📗 [Technical Report 2](./Reports/Technical%20Report%202.pdf) – Maze navigation, line following, sensor systems

---

## 🧪 Testing & Iteration

- ⚠️ Early motor drift corrected using PWM tuning
- 🧠 Logic circuit redesigned to avoid transistor overlap errors
- 🔁 Sensor inputs tested in isolation before full integration
- ❗ Encountered I²C timing issues under load — flagged for future fix

---

## 👨‍💻 Author

**Mohamed Unuse Jalloh**  
First-Year EEE Student, University of Nottingham  
[GitHub: BigB0i11](https://github.com/BigB0i11)

---

## 📜 License

This project is licensed under the MIT License.  
© 2025 Mohamed Unuse Jalloh

---

## 📬 Contact

Interested in collaborating or have feedback?  
Feel free to reach out via GitHub or LinkedIn!


