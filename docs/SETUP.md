# Build and operation notes

## Environment

- Ubuntu environment with a compatible **ROS 1** distribution (the course documentation uses ROS Noetic / Ubuntu 20.04).
- `catkin`, `roscpp`, standard ROS message packages and `tf`.
- Navigation/localization packages: `gmapping`, `map_server`, `amcl`, TurtleBot3 bringup.
- OpenCV and `cv_bridge` for vision.
- TurtleBot3 **Waffle** with LiDAR and a camera, or a simulation publishing equivalent ROS topics.

The full hardware setup and original workspace are not provided in this repository.

## Building

```bash
mkdir -p ~/catkin_ws/src
cp -r botticelli_pkg ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Robot networking

The robot and workstation must communicate with the same ROS master; configure `ROS_MASTER_URI` and local `ROS_IP` appropriately on both computers. In the course setup, the robot drivers and camera were launched over SSH using TurtleBot3 bringup.

Set the model for your actual hardware, for example:

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

The launch files also expose a model argument and may default to `waffle`; adjust it to the robot you are using.

## Modes

With the robot's drivers and ROS master running, choose **one** mode:

```bash
# Explore / construct a map
roslaunch botticelli_pkg boticelli_slam.launch
```

or

```bash
# Load a saved map and start the drawing workflow
roslaunch botticelli_pkg boticelli_draw.launch
```

Configuration files are in `botticelli_pkg/config/`. Occupancy maps consist of paired `.yaml` and `.pgm` files under `botticelli_pkg/maps/`. Their YAML `image` references should be relative to the corresponding PGM.

## Verification limits

The launch XML and package structure can be inspected without hardware, but actual `catkin_make`, mapping, localization, camera capture and motor control need a complete compatible ROS environment. These integration tests are not claimed as having been run during portfolio preparation.
