cmake_minimum_required(VERSION 3.1)
project(simple_game)


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

add_executable(game game.cpp)
target_link_libraries(game ${catkin_LIBRARIES})
