cmake_minimum_required(VERSION 3.5)
project(copub)

# Find dependencies
find_package(ament_cmake REQUIRED)
# Add other dependencies here
# find_package(...)

# Install Python scripts
install(
  DIRECTORY copub/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install dnn directory
install(
  DIRECTORY copub/dnn/
  DESTINATION share/${PROJECT_NAME}/dnn
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


# ament package macro
ament_package()
