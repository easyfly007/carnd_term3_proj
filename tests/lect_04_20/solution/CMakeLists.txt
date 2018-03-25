cmake_minimum_required (VERSION 3.5)

project (BehaviorPlanning)

add_definitions(-std=c++11)


add_executable(BehaviorPlanning main.cc cost.cc road.cc vehicle.cc)

#set (CMAKE_CXX_FLAGS "{CMAKE_CXX_FLAGS} -std=gnu++11")
