cmake_minimum_required(VERSION 3.12)

# Source/Include files
file(GLOB COMMON_SOURCE "*.cpp")
include_directories(${PYTHON3_INCLUDE_DIRS})
include_directories("C:/Users/bun-i/AppData/Local/Programs/Python/Python38/include/")
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)

#Static Library
add_library(common STATIC ${COMMON_SOURCE})

#For matplotlib-cpp
add_definitions(-DWITHOUT_NUMPY)

target_link_libraries(common 
  Python3::Python
  Python3::Module
)

