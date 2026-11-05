# Bot-ticelli — Mobile Robot Drawing with ROS

**Advanced Robotics · BSc in Artificial Intelligence · Universitat Politècnica de Catalunya (UPC) · 2026**

Bot-ticelli is a university group project developed with a **TurtleBot3 Waffle** mobile robot. It combines mapping, computer vision and robot navigation to reproduce the outline of a drawing on the floor. The robot can explore an unknown environment or localize itself in an existing map, identify a suitable drawing area and follow a planned path.

The project is implemented in **C++ with ROS 1**, using laser scans, odometry, camera images and RViz.

## How it works

1. **Map or localize:** GMapping constructs a 2D occupancy map; alternatively, `map_server` and AMCL use a saved map.
2. **Find space:** the exploration node searches for a free region large enough for the drawing while avoiding obstacles.
3. **Capture the drawing:** OpenCV processes the camera image and extracts a simplified contour.
4. **Plan a path:** the contour is scaled and transformed into a `nav_msgs/Path` in the map frame.
5. **Navigate to the start:** the robot approaches and aligns with the first path point.
6. **Trace the shape:** the trajectory follower publishes movement commands and monitors obstacle proximity.

A coordinating node manages transitions between phases. The architecture separates perception, planning, navigation and control into **six C++ ROS nodes**.

## Demonstrations

Demonstration videos were recorded for the university presentation. **YouTube links will be added after verifying the original presentation URLs.** Large videos are intentionally not committed to the repository.

## Project structure

```text
botticelli_pkg/
├── src/             # Six ROS C++ nodes
├── launch/          # Mapping, localization and drawing workflows
├── config/          # ROS parameters and RViz configuration
├── maps/            # Example occupancy maps
├── package.xml
└── CMakeLists.txt
docs/
├── ARCHITECTURE.md  # Node and topic-level design
├── SETUP.md         # Build and execution notes
└── PUBLICATION.md   # Scope and publication checks
```

## ROS nodes

| Node | Responsibility |
| --- | --- |
| `free_space_explorer_node` | Exploration, mapping support and free-space selection |
| `vision_node` | Camera processing and contour extraction |
| `drawing_planner_node` | Convert the detected outline into a robot path |
| `drawing_navigator_node` | Navigate to the drawing start |
| `drawing_follower_node` | Execute the trajectory and monitor obstacles |
| `boticelli_interface_node` | Coordinate stages, user input and commands |

See [architecture and ROS topics](docs/ARCHITECTURE.md).

## Requirements and running the project

The code targets **ROS 1 / catkin**, with C++, OpenCV, ROS navigation/localization packages and a TurtleBot3 platform. It is **not ROS 2**. See [setup and run instructions](docs/SETUP.md).

To build in a configured ROS 1 catkin workspace, place `botticelli_pkg/` in `catkin_ws/src/` and run:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Example launch entry points are `boticelli_slam.launch` and `boticelli_draw.launch`. Running the complete workflow requires a configured robot/simulation, ROS networking and compatible sensors.

## Scope

This repository contains the source code and example maps developed for the university course. It is **not** a production robotics package, and the full integrated workflow has not been re-tested on hardware as part of this GitHub curation.

## Credits

Developed as a group project for the BSc in Artificial Intelligence at UPC. The original team repository also records the project's contributors.
