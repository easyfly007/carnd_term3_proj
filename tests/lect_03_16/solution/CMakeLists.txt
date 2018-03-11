cmake_minimum_required (VERSION 2.8)

project (Bayes)

add_executable(out main.cc classifier.cc)

if (UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()