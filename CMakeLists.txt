cmake_minimum_required(VERSION 3.8)
project(vacuum_gripper_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sdformat14 REQUIRED)
find_package(std_msgs REQUIRED)

# ── Librería C++ del plugin Gazebo ────────────────────────────────────────────
add_library(VacuumGripper SHARED
  src/VacuumGripper.cc
)

target_include_directories(VacuumGripper PUBLIC
  include
)

target_compile_features(VacuumGripper PUBLIC cxx_std_17)

target_link_libraries(VacuumGripper
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  sdformat14::sdformat14
)

ament_target_dependencies(VacuumGripper
  rclcpp
  std_msgs
)

install(
  TARGETS VacuumGripper
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ── Nodo bridge ROS 2 → Gazebo Transport ─────────────────────────────────────
install(
  PROGRAMS scripts/vacuum_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vacuum_bridge_node
)

# ── Hooks de entorno ──────────────────────────────────────────────────────────
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/vacuum_gripper_plugin.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/vacuum_gripper_plugin.sh.in"
)

ament_package()
