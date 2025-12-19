# VoxRovers – Voice Controlled Smart Car

VoxRovers is a compact, low-cost **voice-controlled robotic car** designed to respond to spoken commands while ensuring safe navigation through real-time obstacle detection. The project combines embedded systems, sensor integration, and motor control to demonstrate an intuitive human–machine interface using voice input.

---

## Project Overview

The goal of VoxRovers is to create a robotic vehicle that can be operated entirely using **simple voice commands** such as:

- **Forward**
- **Backward**
- **Left**
- **Right**
- **Stop**

Unlike conventional remote-controlled robots, VoxRovers prioritizes **safety** by continuously monitoring obstacles and overriding motion commands whenever a collision risk is detected. The system is built around an **Arduino Nano**, ensuring simplicity, modularity, and ease of future upgrades.

---

## Key Features

- **Voice-Controlled Operation** using Voice Recognition Module V3  
- **Automatic Obstacle Avoidance** using ultrasonic sensing  
- **Servo-Based Scanning** for dynamic distance measurement  
- **Motor Control** via L293D motor driver  
- **Bluetooth Support (HC-05)** for testing and backup control  
- **Portable Power System** using Li-ion batteries  
- **Modular PCB Design** for stability and scalability  

---

## System Architecture

The system is composed of three tightly integrated subsystems:

1. **Voice Recognition Subsystem**  
   - Interprets trained voice commands  
   - Sends corresponding control signals to the microcontroller  

2. **Motion Control Subsystem**  
   - Drives DC motors through an H-bridge motor driver  
   - Executes directional movement commands  

3. **Safety & Obstacle Detection Subsystem**  
   - Uses an ultrasonic sensor mounted on a servo motor  
   - Continuously checks for obstacles and halts movement if required  

The Arduino Nano acts as the central controller, coordinating all subsystems and ensuring that **safety always takes priority over motion commands**.

---

## Hardware Components

- Arduino Nano  
- Voice Recognition Module V3  
- Ultrasonic Sensor (HC-SR04)  
- Servo Motor  
- L293D Motor Driver IC  
- DC Gear Motors  
- HC-05 Bluetooth Module  
- Li-ion Battery Pack & Power Switch  
- Custom PCB  

---

## Software & Tools

- Arduino IDE  
- Embedded C / Arduino Programming  
- Serial Communication  
- PWM Motor Control  
- PCB Design Tools  

---

## System Implementation

- **Circuit Design**: Logical separation of motor power and sensor logic to reduce noise  
- **PCB Layout**: Custom-designed PCB to improve reliability and reduce wiring issues  
- **Final Assembly**: Compact and portable chassis with securely mounted components  

---

## Applications

- Assistive robotics (e.g., voice-controlled mobility aids)  
- Educational robotics kits  
- Smart service robots  
- Interactive learning platforms  
- Voice-based human–robot interaction research  

---

## Future Work

The current implementation provides a solid foundation that can be extended in several impactful ways:

- **Upgrade to ESP32 or Similar MCU**  
  Enables multitasking, Wi-Fi, and Bluetooth Low Energy (BLE)

- **AI-Based Speech Recognition**  
  Replace offline voice modules with ML-based speech models for higher accuracy

- **Multi-User & Multilingual Support**  
  Allow different users and languages without retraining limitations

- **Camera-Based Navigation**  
  Integrate computer vision for line following, object detection, or SLAM

- **Autonomous Navigation Mode**  
  Combine voice control with autonomous path planning

- **Improved Power Management**  
  Optimize battery efficiency and add power monitoring

- **Enhanced PCB Design**  
  Better power routing, EMI reduction, and modular expansion headers

---

## Team Members

- **Aditya Abhilash** (22EC01059)  
- **N. Jay Roshan Devankar** (22EC01005)  
- **Atharva Tol** (22EC01003)  
- **Nihar Ranjan Jena** (22EC01001)  

---

## License

This project is developed for academic and educational purposes.  
Feel free to modify and extend it with proper attribution.

---

⭐ *VoxRovers demonstrates how simple electronic components, when thoughtfully integrated, can create intuitive and safety-aware robotic systems.*
