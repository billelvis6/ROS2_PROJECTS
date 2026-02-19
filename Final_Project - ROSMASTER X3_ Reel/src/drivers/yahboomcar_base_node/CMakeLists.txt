
cmake_minimum_required(VERSION 3.5)
project(yahboomcar_base_node)

# Standard C++ 14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dépendances (turtlesim SUPPRIMÉ ici)
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Déclaration des exécutables
# Base Node X3 (Ton robot)
add_executable(base_node_X3 src/base_node_X3.cpp)
ament_target_dependencies(base_node_X3 rclcpp tf2 tf2_ros nav_msgs geometry_msgs)

# Ton Talker (Pour les tests de connexion)
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# 3. Installation des programmes dans le dossier install
install(TARGETS
  base_node_X3
  talker
 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

