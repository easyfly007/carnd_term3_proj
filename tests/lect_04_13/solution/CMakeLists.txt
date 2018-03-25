cmake_minimum_required (VERSION 3.5)

project (Cost)

add_definitions(-std=c++11)


add_executable(Cost main.cc cost.cc)

#set (CMAKE_CXX_FLAGS "{CMAKE_CXX_FLAGS} -std=gnu++11")
