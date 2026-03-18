cmake_minimum_required(VERSION 3.8)
project(avatar_challenge)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/shape_drawer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
