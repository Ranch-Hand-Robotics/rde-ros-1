cmake_minimum_required(VERSION 3.0.2)
project(rde_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Create the hello_cpp_publisher executable
add_executable(hello_cpp_publisher src/hello_cpp_publisher.cpp)

# Link libraries
target_link_libraries(hello_cpp_publisher
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Set C++ standard
set_target_properties(hello_cpp_publisher PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)
