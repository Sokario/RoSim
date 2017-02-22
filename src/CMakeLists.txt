include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

set(SOURCE_FILES main.cpp)

add_executable(rosim ${SOURCE_FILES})

include_directories(${SDL2_INCLUDE_DIRS})
target_link_libraries(rosim ${SDL2_LIBRARIES})
include_directories(${OpenGL_INCLUDE_DIRS})
target_link_libraries(rosim ${OPENGL_LIBRARIES})
include_directories(${GLEW_INCLUDE_DIRS})
target_link_libraries(rosim ${GLEW_LIBRARIES})