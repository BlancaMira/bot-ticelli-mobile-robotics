# Architecture and ROS communication

The design follows the team's architecture diagram for the advanced robotics project. The system uses **ROS 1** and separates localization, visual perception, trajectory planning and execution.

## 1. Mapping and localization

Two operating modes are supported:

- **Unknown environment:** `slam_gmapping` combines laser scans and odometry to publish the occupancy grid (`/map`) and the `map → odom` transform. `free_space_explorer_node` uses `/scan`, `/map` and TF to explore and select free drawing space.
- **Known environment:** `map_server` loads a saved `.yaml`/`.pgm` occupancy map; `amcl` estimates the robot position using `/scan`, `/odom`, `/tf` and an initial pose.

## 2. Image to trajectory

```text
/camera/image/compressed
          |
      vision_node
          | /contour_points, /contour_info
          v
   drawing_planner_node <--- dimensions and robot/map transform
          | /drawing_path
          v
  drawing_navigator_node --- /cmd_vel ---> TurtleBot3
          | /drawing_ready
          v
   drawing_follower_node --- /cmd_vel ---> TurtleBot3
          | /drawing_progress, /drawing_finished
          v
   boticelli_interface_node
```

The diagram is conceptual; the interface also coordinates capture, navigation and start/finish topics between the individual nodes.

## 3. Node responsibilities

- **`vision_node`:** processes compressed camera images with OpenCV, segments a contour and publishes the normalized contour and its metadata.
- **`drawing_planner_node`:** scales contour coordinates to physical drawing dimensions and generates a `nav_msgs/Path`.
- **`drawing_navigator_node`:** approaches the trajectory start, then reports readiness.
- **`drawing_follower_node`:** performs lookahead-based path following and uses laser scans for obstacle-related stops.
- **`free_space_explorer_node`:** explores and identifies a large free region suitable for drawing.
- **`boticelli_interface_node`:** handles operating phases and the handoff of velocity commands.

RViz visualizes the occupancy grid, planned path, executed path and navigation markers.

## 4. Important topics

| Topic | Purpose |
| --- | --- |
| `/scan`, `/odom`, `/tf` | Laser range, odometry and robot transforms |
| `/map` | Occupancy grid |
| `/camera/image/compressed` | Camera input |
| `/contour_points`, `/contour_info` | Detected drawing contour |
| `/drawing_path` | Planned drawing trajectory |
| `/cmd_vel` | Base velocity commands |
| `/drawing_ready` | Starting position reached |
| `/drawing_progress`, `/drawing_finished` | Execution status |

The original diagram distinguishes the exploration/SLAM mode from the saved-map/AMCL mode; they should not both control localization simultaneously.

## Source

Based on the team's supplied *Arquitectura bot-ticelli* PDF. A compact written architecture is included here so that the repository remains understandable without requiring external document viewers.
