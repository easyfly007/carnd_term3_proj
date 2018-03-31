cmake_minimum_required (VERSION 3.5)

project (HBF)

add_definitions(-std=c++11)


add_executable(HBF main.cc hybrid_breadth_first.cc)

#set (CMAKE_CXX_FLAGS "{CMAKE_CXX_FLAGS} -std=gnu++11")
