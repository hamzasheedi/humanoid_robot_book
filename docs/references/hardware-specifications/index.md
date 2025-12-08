---
title: Hardware Specifications
---

# Hardware Specifications

This document outlines the hardware specifications for various components and platforms referenced in the AI Robotics Textbook.

## Recommended System Requirements

### Digital Twin Workstation (Required)
- **CPU**: Intel i7-12700K or AMD Ryzen 7 5800X (8+ cores)
- **GPU**: NVIDIA RTX 4070 Ti or higher (12GB+ VRAM recommended)
- **RAM**: 64GB DDR4/DDR5 (16GB minimum)
- **Storage**: 2TB NVMe SSD (fast read/write for simulation)
- **OS**: Ubuntu 22.04 LTS or Windows 10/11 with WSL2
- **Network**: Gigabit Ethernet (for robot communication)

### Physical AI Edge Kit
- **Platform**: NVIDIA Jetson Orin AGX (32GB) or Jetson Orin NX
- **OS**: JetPack SDK
- **Sensors**: 
  - Depth camera (e.g., Intel RealSense D435i or similar)
  - IMU for orientation sensing
  - Microphone array for voice commands
- **Connectivity**: Wi-Fi 6, Ethernet, Bluetooth
- **Power**: 19V DC adapter (provided with Jetson kit)

## Humanoid Robot Platforms

### Unitree Go2 Specifications
- **Degrees of Freedom**: 12 (3 per leg)
- **Weight**: 10 kg
- **Height**: 52 cm
- **Battery**: 25.6V/8.8Ah lithium battery (2+ hours operation)
- **Actuators**: 12 Unitree servo actuators
- **Sensors**: IMU, 3D LiDAR, RGB camera
- **Control**: Real-time control at 500Hz

### Unitree G1 Specifications
- **Degrees of Freedom**: 23
- **Weight**: 35 kg
- **Height**: 103 cm
- **Battery**: 25.6V/10Ah lithium battery
- **Actuators**: 23 custom servo actuators
- **Sensors**: IMU, LiDAR, 3D camera
- **Payload**: Up to 5 kg

### Alternative Platforms
Other humanoid platforms may be used with appropriate configuration adjustments.

## Simulation Hardware Requirements

### Minimum Specifications for Gazebo Simulations
- **GPU**: NVIDIA GTX 1660 SUPER or equivalent
- **VRAM**: 6GB+ for basic simulations
- **RAM**: 16GB+
- **CPU**: 6+ cores

### Recommended Specifications for High-Fidelity Simulations
- **GPU**: NVIDIA RTX 4080 or higher
- **VRAM**: 16GB+ for complex scene rendering
- **RAM**: 32GB+
- **CPU**: 8+ cores, high clock speed

## Peripherals and Accessories

### For Digital Twin Workstations
- Large monitor (24"+ recommended for development)
- VR headset for immersive simulation (optional)
- Gamepad for robot teleoperation (optional)
- USB hub for connecting various sensors/devices

### For Physical Robots
- Charging stations for batteries
- Protective equipment for safe operation
- Mobile workstations for field deployment
- Calibration tools for sensors

## Network Specifications

### Robot Communication
- **Protocol**: ROS 2 over DDS
- **Bandwidth**: 100 Mbps minimum, 1 Gbps recommended
- **Latency**: Less than 10ms for real-time control
- **Range**: As required by deployment environment

### Cloud Integration
When using cloud-based processing or simulation:
- **Minimum Upload Speed**: 10 Mbps
- **Recommended Upload Speed**: 50+ Mbps
- **Latency**: Less than 50ms for interactive operations

## Compatibility Notes

All specifications are tested with:
- ROS 2 Humble Hawksbill
- Gazebo Garden or Fortress
- NVIDIA Isaac Sim 2023.1+
- Unity 2022.3 LTS

Performance may vary based on specific hardware configurations and environmental factors.