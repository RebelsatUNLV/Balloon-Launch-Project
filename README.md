# Balloon Launch Project

## Overview
This repository contains the code and documentation for the Balloon Launch Project by RebelsatUNLV. The project aims to design, build, and launch a high-altitude balloon equipped with sensors to collect atmospheric data and capture imagery from near space.

## Repository Structure
The repository is organized as follows:
```
├── CubeSat_code/      # Code for each of the circuits in cubesat
├── hardware/           # Hardware specifications and configurations
├── README.md           # This file
└── Testing/            # Code for testing the .ino code
```

## Getting Started

### Prerequisites
- Edgeflyte cubesat

-or-

- Raspberry Pi hardware and accessories
- Arduino IDE (for some sensor modules)
- Python 3.x
- Environmental sensors (temperature, pressure, humidity)
- Required packages (see `MCB.ino, ASB.ino, BMS.ino`)

### Installation
1. Clone this repository:
   ```
   git clone https://github.com/RebelsatUNLV/Balloon-Launch-Project.git
   cd Balloon-Launch-Project
   ```
2. Install dependencies:
    Use Arduino IDE libraries to install libraries
3. Configure your hardware according to the specifications in the hardware directory
4. Test individual components using the code in the Testing directory

## Hardware
The balloon payload consists of:
- Raspberry Pi as the main flight computer
- GPS module for location tracking
- Environmental sensors for atmospheric data collection
- Camera system for aerial photography (HIATUS)
- Radio telemetry for real-time data transmission
- Power management system


## Software Components
- ASB.ino: Software for the Atmospheric board
- MCB.ino: Main flight software for the Raspberry Pi Pico 2
- BMS.ino: Software for the power management
- Testing: Validation scripts for pre-flight testing


## Contributors
- UNLV Rebelsat Team

## Acknowledgments
- UNLV Engineering Department
- [Other sponsors and supporters]