find_package(geometry_msgs REQUIRED)
# Any C++ source files needed to build this target (swarm_library_tester).
add_executable(swarm_library_tester
  # list of source cpp files:
  main.cpp
  test.cpp
  )

# Any include directories needed to build this target.
# Note: we do not need to specify the include directories for the
# dependent libraries, they are automatically included.
target_include_directories(swarm_library_tester PUBLIC
  # list of include directories:
  ${CMAKE_SOURCE_DIR}/include
  ${geometry_msgs_INCLUDE_DIRS}
  )

# Any dependent libraires needed to build this target.
target_link_libraries(swarm_library_tester PUBLIC
  # list of libraries:
  gtest
  ${geometry_msgs_LIBRARIES}
  swarm_library
  )

# Enable CMakes test runner to discover the tests included in the
# binary, using the GoogleTest CMake module.
gtest_discover_tests(swarm_library_tester)


# # Build executable:
# add_executable (swarm_library_tester
#   test/main.cpp
#   )
# target_link_libraries(swarm_library_tester
#   # list of libraries
#   swarm_library
#   gtest
#   )