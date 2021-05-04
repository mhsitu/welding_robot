cmake_minimum_required(VERSION 3.12)

project(welding_robot)

if(UNIX)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}-m64")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-m64")
endif()

#Make sure no in-source build -----------------------------------------------------------------------------#
if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

#Configure CMake and Compiler -----------------------------------------------------------------------------#
set( CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set( CMAKE_CXX_STANDARD 14)
set( CMAKE_C_STANDARD 11)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin) 
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib) 

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message("Setting build type to 'Debug' as none was specified.")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for test_footStest_footPlacementtepPlan-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()


add_definitions(-DNON_MATLAB_PARSING
                -DMAX_EXT_API_CONNECTIONS=255
                -DDO_NOT_USE_SHARED_MEMORY)

#Add sub directories --------------------------------------------------------------------------------------#
add_subdirectory(core)
add_subdirectory(common)
add_subdirectory(coppeliaSim-client)

#Add libs and include_directories -------------------------------------------------------------------------#
include_directories(".")
include_directories("common")
include_directories("core")
include_directories("coppeliaSim-client")
include_directories("coppeliaSim-client/include")
include_directories("coppeliaSim-client/include/stack")
include_directories("coppeliaSim-client/remoteApi")

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
file(GLOB USER_SOURCE "*.cpp")
add_executable(${PROJECT_NAME} ${USER_SOURCE})

target_link_libraries(${PROJECT_NAME} core)
target_link_libraries(${PROJECT_NAME} common)
target_link_libraries(${PROJECT_NAME} coppeliaSim_client)
target_link_libraries(${PROJECT_NAME} 
  Python3::Python
  Python3::Module
  Python3::NumPy
)

