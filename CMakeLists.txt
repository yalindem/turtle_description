cmake_minimum_required(VERSION 3.8)
project(turtle_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Bağımlılıkları bul
find_package(ament_cmake REQUIRED)

# -------------------------------------------------------------------
# KURULUM (INSTALLATION) BÖLÜMÜ
# Bu kısım, URDF ve Launch dosyalarının ROS 2 kurulum dizinine kopyalanmasını sağlar.
# -------------------------------------------------------------------

# 1. URDF/Xacro dosyalarını kopyala
# Dosyalar: install/share/turtle_description/urdf/ dizinine kopyalanacak
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()