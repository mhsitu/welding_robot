# Source/Include files
include_directories(".")
file(GLOB SOURCE "*.cpp")

add_library(core STATIC ${SOURCE})

#Add common
target_link_libraries(core common)
include_directories("../common")



