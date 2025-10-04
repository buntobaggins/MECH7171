#------------------------------------------------------------------------------
# CMAKE build file to compile MECH7171 lab projects
#------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.20)

project(MECH7171 LANGUAGES C CXX)

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG    ${CMAKE_SOURCE_DIR}/_build/_debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE  ${CMAKE_SOURCE_DIR}/_build/_release)

# Source files
file(GLOB C_SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.c")
file(GLOB CPP_SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")

# Add executable
add_executable(lab3_task1 ${C_SRC_FILES} ${CPP_SRC_FILES})
target_include_directories(lab3_task1 PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Compiler flags
if (MSVC)
    target_compile_options(lab3_task1 PRIVATE /std:c17)
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(lab3_task1 PRIVATE /Zi /Wall /EHsc)
    elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
        target_compile_options(lab3_task1 PRIVATE /O2 /EHsc)
    endif()
endif()



