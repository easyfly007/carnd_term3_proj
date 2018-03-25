cmake_minimum_required (VERSION 3.5)

project (NaiveBayes)

add_definitions(-std=c++11)


add_executable(NaiveBayes main.cc classifier.cc)

#set (CMAKE_CXX_FLAGS "{CMAKE_CXX_FLAGS} -std=gnu++11")
