cmake_minimum_required(VERSION 2.8.3)
project(hear_navio_mc_example)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hear_architecture
  hear_ros_bridge
)

catkin_package(
  CATKIN_DEPENDS
    hear_ros_bridge
    hear_architecture
    roscpp
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mc_example_node example_node/example_node.cpp)
add_executable(mc_slam_mrft_node example_node/SLAM_tuning_node.cpp)
add_executable(mc_translation_node example_node/traslation_node.cpp)
add_executable(mc_test_node example_node/test_node.cpp)
add_executable(mc_standard_node example_node/standard_MC_node.cpp)
add_executable(mrft_scenario example_node/mrft_scenario.cpp)

add_dependencies(mc_example_node ${catkin_EXPORTED_TARGETS})
add_dependencies(mc_slam_mrft_node ${catkin_EXPORTED_TARGETS})
add_dependencies(mc_translation_node ${catkin_EXPORTED_TARGETS})
add_dependencies(mc_test_node ${catkin_EXPORTED_TARGETS})
add_dependencies(mc_standard_node ${catkin_EXPORTED_TARGETS})
add_dependencies(mrft_scenario ${catkin_EXPORTED_TARGETS})

target_link_libraries(mc_example_node ${catkin_LIBRARIES})
target_link_libraries(mc_test_node ${catkin_LIBRARIES})
target_link_libraries(mc_standard_node ${catkin_LIBRARIES})
target_link_libraries(mc_slam_mrft_node ${catkin_LIBRARIES})
target_link_libraries(mc_translation_node ${catkin_LIBRARIES})
target_link_libraries(mrft_scenario ${catkin_LIBRARIES})
