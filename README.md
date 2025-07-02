# led_pannel_gazebo
## Overview

This ROS 2 package, `led_panel_sim`, simulates a 16x16 LED matrix panel in Gazebo Harmonic. It provides two nodes for visualizing 100-character data on the panel using different encoding strategies.

## Nodes

### 1. `code_publisher.py`

- **Function:** Publishes frames representing characters on the LED panel.
- **Encoding:**  
    - Each character is converted to its binary ASCII code (8 bits).
    - The code is split into two halves (4 bits each).
    - Each 4-bit half maps to a 4x4 submatrix:
        - Bit value `1`: Corresponding 4x4 submatrix is turned ON.
        - Bit value `0`: Corresponding 4x4 submatrix is turned OFF.
    - Two frames (one for each half) are used to represent a single character.

### 2. `qr_publisher.py`

- **Function:** Embeds the 100 characters into a QR code and visualizes it on the LED panel.
- **Encoding:**  
    - Uses QR Code version 7 (45x45 modules).
    - The QR code is padded with zeros to fit a 48x48 matrix.
    - The padded QR code is divided into 9 chunks (each 16x16).
    - Each chunk is sequentially displayed on the LED panel.

## Installation

First, create a ROS 2 workspace if you don't have one:

```bash
mkdir ~/led_panel_ws
cd ~/led_panel_ws
```

Clone the repository into the `src` folder:

```bash
git clone https://github.com/jhon-zelada/led_pannel_gazebo.git
```

Build the workspace:

```bash
cd ~/led_panel_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

1. Launch Gazebo Harmonic with the LED panel simulation:
    ```bash
    ros2 launch led_panel_sim launch_panel.launch.py
    ```
2. Run either `code_publisher.py` or `qr_publisher.py` to visualize data:
    ```bash
    ros2 run led_panel_sim code_publisher.py
    # or
    ros2 run led_panel_sim qr_publisher.py
    ```

## Requirements

- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3
- `qrcode` Python library (`pip install qrcode`)

## License

This project is licensed under the MIT License.