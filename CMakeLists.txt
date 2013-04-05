cmake_minimum_required(VERSION 2.6)

project(Surfaces)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(  ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

add_subdirectory(lib)
add_subdirectory(src)
