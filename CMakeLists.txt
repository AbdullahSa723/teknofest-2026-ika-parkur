cmake_minimum_required(VERSION 3.8)
project(ika_parkur_gazebo)

find_package(ament_cmake REQUIRED)

# Everything in this package is data (SDF worlds, textures, launch files) plus
# the Python generator that produces the world. Nothing is compiled.
# The EXCLUDEs keep the generator's bytecode cache out of the install tree.
# Without them a stale __pycache__ from the source drive is copied into share/,
# where it is dead weight at best and confusing at worst.
install(
  DIRECTORY launch worlds models generator
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# The kayar engel driver. PROGRAMS rather than FILES so it keeps its exec bit,
# which matters because the source lives on an NTFS drive under WSL.
install(
  PROGRAMS nodes/kayar_engel_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

# Make worlds/ and models/ discoverable by Gazebo without extra env fiddling.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/resource_paths.dsv.in")

ament_package()
