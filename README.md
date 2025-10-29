# face_detector

This is a simple face recognition method based on Opnecv DNN

## Hardware

RealsenseD435

## Installation

```shell
cd ~/YOUR_WORKSPACE/src
git clone
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Usage

```shell
source install/setup.bash
ros2 launch realsense_camera rs_launch.py
```

```shell
source install/setup.bash
ros2 launch face_detector_cpp face_detector.py
```
