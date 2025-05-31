# leader-teleop

## Overview

**leader-teleop** is a Python package for teleoperation of robotic joints using Dynamixel motors. It provides scripts for reading joint states, transferring joint angles, and saving joint data.

## Features

- Read and save joint angles from Dynamixel motors
- Transfer joint angles between robots
- Configurable via YAML and CSV files
- Modular code structure for easy extension

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/leader-teleop.git
   cd leader-teleop
   ```

2. Install dependencies:
# Robotics SDK and Driver Installation Guide

This guide covers the installation of several popular robotics SDKs and drivers:

- [Stereolabs ZED Python API](#stereolabs-zed-python-api)
- [AgileX Robotics PiPER SDK](#agilex-robotics-piper-sdk)
- [CH341SER USB-to-Serial Driver for Linux](#ch341ser-usb-to-serial-driver-for-linux)
- [ROBOTIS Dynamixel SDK](#robotis-dynamixel-sdk)

---

## Stereolabs ZED Python API

The ZED Python API enables Python development with Stereolabs ZED stereo cameras.

### Prerequisites

- ZED SDK installed (`https://www.stereolabs.com/docs/`)
- Python 3.6+ (64-bit)
- Cython ≥ 0.26, NumPy ≥ 1.13

### Installation

```bash
python3 -m pip install cython numpy opencv-python pyopengl
cd /usr/local/zed
python3 get_python_api.py
```

For more information, see [ZED Python API Docs](https://www.stereolabs.com/docs/app-development/python/install/).

---

## AgileX Robotics PiPER SDK

The PiPER SDK is used for controlling the AgileX PiPER robotic arm via CAN.

### Prerequisites

- Python 3.x
- `python-can` (≥ 3.3.4)

### Installation

```bash
pip3 install python-can
pip3 install piper_sdk
```

> Ensure your CAN interface is configured and connected.

- [PiPER SDK GitHub](https://github.com/agilexrobotics/piper_sdk)

---

## CH341SER USB-to-Serial Driver for Linux

The CH341SER driver supports WCH CH340/CH341-based USB-to-Serial (RS232/RS485) adapters.

### Installation

1. **Download the driver:**

   [CH341SER_LINUX.ZIP (Official)](https://www.wch.cn/downloads/CH341SER_LINUX_ZIP.html)

2. **Extract and install:**

   ```bash
   unzip CH341SER_LINUX.ZIP
   cd CH341SER_LINUX
   make clean
   make
   sudo make load
   ```

3. **Plug in your CH340/341 device and verify:**

   ```bash
   dmesg | grep ch341
   ls /dev/ttyUSB*
   ```

> If you encounter issues on newer kernels, try a patched version such as [wch/ch341ser_linux](https://github.com/WCHSoftGroup/ch341ser_linux).

---

## ROBOTIS Dynamixel SDK

The Dynamixel SDK provides cross-platform support for controlling Dynamixel actuators.

### Installation

#### For C++

```bash
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/c++/build/linux64
make
```

#### For Python

```bash
cd DynamixelSDK/python
pip3 install .
```

For details and sample code, visit the [DynamixelSDK GitHub](https://github.com/ROBOTIS-GIT/DynamixelSDK) and the [official e-Manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/).

---

## Notes

- Make sure all hardware connections (USB, CAN, serial) are secure before using the SDKs.
- For serial/USB devices, you may need to add your user to the `dialout` group for permissions:

  ```bash
  sudo usermod -aG dialout $USER
  # Log out and back in after running the above command.
  ```

- Use `minicom`, `screen`, or `python-can` to test your CAN/serial interfaces.

---

*For troubleshooting or further assistance, consult the respective project documentation or ask in the project’s issues section.*


## Usage

### 1. Configure Your Robot

Edit `robot_config.yaml` to match your robot's hardware setup.

### 2. Read and Save Joint Angles

```bash
python group_read_save.py --config robot_config.yaml --output joint_angle_deltas.csv
```

### 3. Transfer Joint Angles

```bash
python joint_transfer.py --config robot_config.yaml --input joint_angle_deltas.csv
```

### 4. Command-Line Arguments

Most scripts support the following arguments:

- `--config`: Path to the YAML configuration file
- `--input` / `--output`: Path to CSV file for input/output
- `--port`: Serial port for Dynamixel communication (optional)

## File Structure

```
leader-teleop/
├── group_read_save.py      # Script to read and save joint angles
├── joint_transfer.py       # Script to transfer joint angles
├── robot_config.yaml       # Robot hardware configuration
├── joint_angle_deltas.csv  # Example CSV of joint angles
├── requirements.txt        # Python dependencies
├── README.md               # This file
└── src/                    # (Recommended) Core library code
```

## Development

- All code follows [PEP8](https://www.python.org/dev/peps/pep-0008/) style guidelines.
- Type hints are used throughout for clarity.
- Logging is used instead of print statements for diagnostics.

## Testing

Unit tests are located in the `tests/` directory. To run tests:

```bash
pytest
```

## Contributing

Pull requests are welcome! Please ensure your code is well-documented and tested.

## License

MIT License

---