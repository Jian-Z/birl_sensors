cmake_minimum_required(VERSION 2.8.3)
project(robotiq_s_model_articulated_gazebo)

find_package(catkin REQUIRED)

# Flag to avoid libboost errors
add_compile_options(-std=c++11)

catkin_package(
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
