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
   ```bash
   pip install -r requirements.txt
   ```

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