cmake_minimum_required(VERSION 3.12)

# Source/Include files
file(GLOB COMMON_SOURCE "*.cpp")

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

#Static Library
add_library(common STATIC ${COMMON_SOURCE})

#For matplotlib-cpp
target_link_libraries(common 
  Python3::Python
  Python3::Module
  Python3::NumPy
)

