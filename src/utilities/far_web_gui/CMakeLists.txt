cmake_minimum_required(VERSION 3.5)
project(far_web_gui)

find_package(ament_cmake REQUIRED)

# Install the Python bridge node
install(PROGRAMS
  scripts/graph_file_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install web frontend
install(DIRECTORY www/
  DESTINATION share/${PROJECT_NAME}/www
)

ament_package()
