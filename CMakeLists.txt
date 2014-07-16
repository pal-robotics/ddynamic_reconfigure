cmake_minimum_required(VERSION 2.8.3)
project(ddynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/Tutorials.cfg
)

add_executable(dynamic_reconfigure_node src/dynamic_reconfigure_node.cpp)
target_link_libraries(dynamic_reconfigure_node ${catkin_LIBRARIES})

add_executable(fake_dynamic_reconfigure_server src/fake_dynamic_reconfigure_server.cpp)
target_link_libraries(fake_dynamic_reconfigure_server ${catkin_LIBRARIES})

# make sure configure headers are built before any node using them
add_dependencies(dynamic_reconfigure_node ${PROJECT_NAME}_gencfg)

