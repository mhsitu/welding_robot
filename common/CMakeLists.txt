cmake_minimum_required(VERSION 3.12)

# Source/Include files
file(GLOB COMMON_SOURCE "*.cpp")

#Static Library
add_library(common STATIC ${COMMON_SOURCE})
