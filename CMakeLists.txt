cmake_minimum_required(VERSION 3.8)
project(oxebots_observers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(oxebots_interfaces REQUIRED)
find_package(ssl_league_msgs REQUIRED) # Necessário para ler do bridge
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Executável do Tradutor Central
add_executable(game_observer_node src/game_observer_node.cpp)
ament_target_dependencies(game_observer_node 
  rclcpp 
  oxebots_interfaces 
  ssl_league_msgs 
  nav_msgs 
  tf2 
  tf2_geometry_msgs
)

# Executável do Visualizador do Campo (MANTIDO)
add_executable(field_visualizer_node src/field_visualizer_node.cpp)
ament_target_dependencies(field_visualizer_node 
  rclcpp 
  visualization_msgs 
  geometry_msgs
)

ament_target_dependencies(game_observer_node 
  rclcpp oxebots_interfaces ssl_league_msgs nav_msgs tf2 tf2_geometry_msgs)

ament_target_dependencies(field_visualizer_node 
  rclcpp visualization_msgs geometry_msgs)

install(TARGETS 
    game_observer_node 
    field_visualizer_node 
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()