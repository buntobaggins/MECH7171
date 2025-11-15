#------------------------------------------------------------------------------
# CMAKE build file to compile MECH7171 lab projects
#------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.20)

project(MECH7171 LANGUAGES C CXX)

# Set build directory
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/_build/task3)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/_build/task3)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/_build/task3)

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG    ${CMAKE_SOURCE_DIR}/_build/task3/_debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE  ${CMAKE_SOURCE_DIR}/_build/task3/_release)

# Source files
file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/task3/src/*.c")
set (     ROBOT_SRC "${CMAKE_SOURCE_DIR}/src/robot.cpp")

# Add executable
add_executable(lab5_task3 ${ROBOT_SRC} ${SRC_FILES})
target_include_directories(lab5_task3 PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include/)

# Link libraries
target_link_libraries(lab5_task3 PRIVATE ${CMAKE_SOURCE_DIR}/lib/helper_functions.lib)

# Compiler flags
if (MSVC)
    target_compile_options(lab5_task3 PRIVATE /std:c17)
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(lab5_task3 PRIVATE /wd5045 /wd5039 /wd4820 /wd4668 /Zi /Wall /EHsc)
    elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
        target_compile_options(lab5_task3 PRIVATE /wd5045 /wd5039 /wd4820 /wd4668 /O2 /EHsc)
    endif()
endif()
