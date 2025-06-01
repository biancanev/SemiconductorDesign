cmake_minimum_required(VERSION 3.16)
project(CircuitSimulator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler-specific options
if(MSVC)
    add_compile_options(/W4)
    add_compile_definitions(_USE_MATH_DEFINES)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Eigen3 using vcpkg's CONFIG mode
find_package(Eigen3 CONFIG REQUIRED)

# Collect source files from subdirectories
file(GLOB SOURCES 
    "src/*.cpp" 
    "src/*.c"
    "src/parser/*.cpp"
    "src/parser/*.c"
    "src/simulation/*.cpp"
)

# Add include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/parser
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation
)

# Create the main executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link Eigen3
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

# Platform-specific linking
if(WIN32 OR MINGW)
    # Add Windows-specific libraries here when needed
endif()

# Print what we found (helpful for debugging)
message(STATUS "Found ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "Sources: ${SOURCES}")
message(STATUS "Eigen3 found: ${Eigen3_FOUND}")