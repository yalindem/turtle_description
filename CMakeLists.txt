cmake_minimum_required(VERSION 3.8)
project(turtle_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Bağımlılıkları bul
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# -------------------------------------------------------------------
# KURULUM (INSTALLATION) BÖLÜMÜ
# Bu kısım, URDF ve Launch dosyalarının ROS 2 kurulum dizinine kopyalanmasını sağlar.
# -------------------------------------------------------------------

add_executable(diff_drive_node
  src/diff_drive_kinematic.cpp
)

ament_target_dependencies(diff_drive_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)


install(TARGETS
  diff_drive_node
  DESTINATION lib/${PROJECT_NAME}
)

# 1. URDF/Xacro dosyalarını kopyala
# Dosyalar: install/share/turtle_description/urdf/ dizinine kopyalanacak
install(
  DIRECTORY urdf launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
