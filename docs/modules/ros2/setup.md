---
title: ROS 2 Setup Guide
---

# ROS 2 Setup Guide

This guide will help you set up ROS 2 (Robot Operating System 2) on your development environment. We'll use ROS 2 Humble Hawksbill, which is an LTS (Long Term Support) version with 5-year support.

## Supported Hardware Configurations

This textbook supports multiple hardware configurations to ensure accessibility:

### On-Premise Lab Configuration
- **OS**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **CPU**: 4+ cores recommended
- **RAM**: 8GB+ (16GB recommended)
- **GPU**: Dedicated GPU recommended for simulations
- **Storage**: 20GB+ free space

### Ether Lab (Cloud) Configuration
- Access to cloud-based ROS 2 development environment
- Web-based IDE with ROS 2 pre-installed
- Simulation environments pre-configured
- Shared computing resources

### Economy Jetson Student Kit Configuration
- **Hardware**: NVIDIA Jetson Nano or equivalent
- **OS**: JetPack SDK (based on Ubuntu)
- **RAM**: 4GB+ (Jetson Nano) or 8GB+ (Jetson Xavier)
- **Storage**: 16GB+ microSD card
- **Connectivity**: Ethernet or WiFi

## Installation Steps

### Ubuntu 22.04 LTS (Recommended)

1. Update your Ubuntu packages:
   ```bash
   sudo apt update && sudo apt upgrade
   ```

2. Set locale to support UTF-8:
   ```bash
   sudo locale-gen en_US.UTF-8
   ```

3. Add the ROS 2 GPG key and repository:
   ```bash
   sudo apt update && sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. Install ROS 2 Humble Hawksbill:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

5. Install colcon build system:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

6. Source the ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

7. To automatically source ROS 2 in new terminals, add to your `.bashrc`:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

### Windows with WSL2

1. Install WSL2 with Ubuntu 22.04
2. Follow the Ubuntu instructions above within your WSL2 environment

### Jetson Setup

1. Flash your Jetson with appropriate JetPack SDK
2. Install ROS 2 via provided scripts or package manager

## Verification

To verify your installation, open a new terminal and run:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

If you see a list (likely empty if no nodes are running), your installation is successful.

## Troubleshooting

If you encounter issues during installation:

1. Check system requirements are met
2. Ensure your OS is fully updated
3. Verify Internet connectivity for package downloads
4. Check that no conflicting ROS versions are installed

If problems persist, consult the official ROS 2 documentation or reach out through the textbook's support resources.