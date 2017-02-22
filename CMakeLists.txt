cmake_minimum_required(VERSION 3.2)
project(rosim)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
file(MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

add_subdirectory(src)