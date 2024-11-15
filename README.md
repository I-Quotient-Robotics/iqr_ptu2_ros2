# iqr_ptu2

ROS2 driver for IQR PTU2

# Installation

```shell
cd <your_ros2_ws>/src
git https://github.com/I-Quotient-Robotics/iqr_ptu2_ros2
cd iqr_ptu2_ros2
sudo cp ./56-pan-tilt.rules /etc/udev/rules.d/
cd ../..
colcon build
```

## Usage

```shell
ros2 launch iqr_ptu2 bringup.launch.py
```

keyboard teleoperation:

```shell
ros2 run iqr_ptu2 teleop.py
```