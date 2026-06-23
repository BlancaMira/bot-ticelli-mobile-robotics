cmake_minimum_required(VERSION 3.0.2)
project(boticelli_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# =========================
# Visió
# =========================

add_executable(boticelli_vision_node src/vision_node.cpp)
set_target_properties(boticelli_vision_node PROPERTIES OUTPUT_NAME vision_node PREFIX "")
add_dependencies(boticelli_vision_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(boticelli_vision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# =========================
# Interfície principal
# =========================

add_executable(boticelli_interface_node src/boticelli_interface_node.cpp)
set_target_properties(boticelli_interface_node PROPERTIES OUTPUT_NAME boticelli_interface_node PREFIX "")
add_dependencies(boticelli_interface_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(boticelli_interface_node ${catkin_LIBRARIES})

# =========================
# Dibuix
# =========================

add_executable(boticelli_drawing_planner_node src/drawing_planner_node.cpp)
set_target_properties(boticelli_drawing_planner_node PROPERTIES OUTPUT_NAME drawing_planner_node PREFIX "")
add_dependencies(boticelli_drawing_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(boticelli_drawing_planner_node ${catkin_LIBRARIES})

add_executable(boticelli_drawing_navigator_node src/drawing_navigator_node.cpp)
set_target_properties(boticelli_drawing_navigator_node PROPERTIES OUTPUT_NAME drawing_navigator_node PREFIX "")
add_dependencies(boticelli_drawing_navigator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(boticelli_drawing_navigator_node ${catkin_LIBRARIES})

add_executable(boticelli_drawing_follower_node src/drawing_follower_node.cpp)
set_target_properties(boticelli_drawing_follower_node PROPERTIES OUTPUT_NAME drawing_follower_node PREFIX "")
add_dependencies(boticelli_drawing_follower_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(boticelli_drawing_follower_node ${catkin_LIBRARIES})

# =========================
# SLAM / exploració
# =========================

add_executable(boticelli_free_space_explorer_node src/free_space_explorer_node.cpp)
set_target_properties(boticelli_free_space_explorer_node PROPERTIES OUTPUT_NAME free_space_explorer_node PREFIX "")
add_dependencies(boticelli_free_space_explorer_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(boticelli_free_space_explorer_node ${catkin_LIBRARIES})
