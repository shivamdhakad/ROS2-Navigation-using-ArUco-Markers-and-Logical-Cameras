cmake_minimum_required(VERSION 3.8)
project(group20_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(rosgraph_msgs REQUIRED)

# include folder whish has all the header files
include_directories(include/group20_final)

# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  rclcpp 
  tf2_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
  ros2_aruco_interfaces
  mage_msgs
  nav_msgs
  nav2_msgs
  rosgraph_msgs
  rclcpp_action
)

# Find all dependencies
foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()


#-----------------------------
# C++
#-----------------------------
add_executable(code src/code.cpp)
ament_target_dependencies(code ${FRAME_DEMO_INCLUDE_DEPENDS})
# add_executable(listener src/listener.cpp )
# ament_target_dependencies(listener ${FRAME_DEMO_INCLUDE_DEPENDS})

install(TARGETS code
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install (DIRECTORY include config launch DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
code
DESTINATION lib/${PROJECT_NAME}
)
# install(TARGETS
# listener
# DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY include
#   DESTINATION share/${PROJECT_NAME}/
# )
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )
ament_package()
