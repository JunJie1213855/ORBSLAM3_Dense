cmake_minimum_required(VERSION 3.10)
project(elas)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3")
set(CMAKE_BUILD_TYPE Release)


## package
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

## source code
aux_source_directory(./elas SOURCE)

# library
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_library(elas SHARED
    ${SOURCE}
)
target_link_libraries(elas
    ${OpenCV_LIBS}
)



