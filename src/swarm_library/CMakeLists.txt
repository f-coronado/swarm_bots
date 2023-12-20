cmake_minimum_required(VERSION 3.8)
project(swarm_library)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# Add your library source files
# Build libraries:
add_library(swarm_library
  src/multi_robot_publisher.cpp
)

target_include_directories(swarm_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PUBLIC 
    ${rclcpp_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
)
# Link against external libraries
target_link_libraries(swarm_library
  ${rclcpp_LIBRARIES}
  # geometry_msgs::geometry_msgs
)


# Install executables and libraries:
install(TARGETS
  swarm_library
  EXPORT swarm_library-export
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(FILES
  include/swarm_library/multi_robot_publisher.hpp
  DESTINATION include
)

# Export build tree to the package registry (~/.cmake/packages/<package>/)
set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)
export(PACKAGE swarm_library)



export(TARGETS
  swarm_library
  FILE swarm_library-export.cmake
)

set (CMAKE_EXPORT_PACKAGE_REGISTRY ON)
export(PACKAGE swarm_library)

# Install the export group
install(EXPORT swarm_library-export
  DESTINATION share/${PROJECT_NAME}/cmake/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
