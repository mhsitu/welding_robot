cmake_minimum_required(VERSION 3.12)

#Preprocessor definitions
add_definitions(-DNON_MATLAB_PARSING
                -DMAX_EXT_API_CONNECTIONS=255
                -DDO_NOT_USE_SHARED_MEMORY)

#Sources/Incllude files
include_directories(".")
include_directories("include/")
include_directories("include/stack")
include_directories("remoteApi/")
file(GLOB SIM_CLIENT_SOURCES "common/*.c" "common/*.cpp" "remoteApi/*.c" "common/stack/*.cpp" "./*.cpp")

#Static Library
add_library(coppeliaSim_client STATIC ${SIM_CLIENT_SOURCES})

target_link_libraries(coppeliaSim_client "ws2_32")
target_link_libraries(coppeliaSim_client "winmm")

