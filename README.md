# Advanced Automated Seed Planting Robot
This repository hosts the design and code for an advanced automated seed planting robot. Developed as a part of the Computer Engineering Design Project for ECE 492, this project integrates cutting-edge LiDAR technology with Raspberry Pi's GPIO capabilities to revolutionize agricultural practices through automation and precision.

## Overview

The project aims to automate the seed planting process, utilizing the Tau LiDAR Camera for object detection and navigation, thereby enhancing efficiency and precision in agricultural operations. It's designed with scalability and adaptability in mind, allowing for future enhancements and integrations.

## Core Components

- **Object Detection:** Employs Tau LiDAR Camera for real-time object detection, enabling the robot to navigate through varying terrains and avoid obstacles.
- **Drive Mechanism:** Utilizes Raspberry Pi GPIO control for maneuvering, equipped with a sophisticated drive control system for precise movements.
- **Seed Dispensing:** Automated seed dispensing mechanism, controlled through servo motors, ensuring optimal seed placement and depth.

## Technical Details

This project is a showcase of integrating hardware and software to solve real-world problems. While not intended for direct user interaction through a graphical interface, it provides a comprehensive backend system that controls and monitors the seed planting process.

### Technologies Used

- **Raspberry Pi:** Serves as the brain of the robot, handling computations, control signals, and peripheral communications.
- **Tau LiDAR Camera:** Provides spatial data and object detection capabilities essential for autonomous navigation.
- **Python:** The primary programming language used for developing the software control system.

## Project Structure

- `distance.py`: Script for initiating and handling LiDAR-based object detection.
- `drive_controls.py`: Contains the logic for the robot's movement controls.
- `main.py`: The main script that orchestrates the seed planting operations.
