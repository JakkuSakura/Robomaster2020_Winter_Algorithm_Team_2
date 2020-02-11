cmake_minimum_required(VERSION 3.1)
project(test_game)


set(CMAKE_CXX_STANDARD 11)

#############################
#Dependencies
#############################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    std_msgs
    visualization_msgs
)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_game game.cpp)
target_link_libraries(test_game ${catkin_LIBRARIES})
