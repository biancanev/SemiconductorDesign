cmake_minimum_required(VERSION 3.16)
project(CircuitSimulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages the Linux way
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)

# Use pkg-config to find GLFW3
pkg_check_modules(GLFW REQUIRED glfw3)

# Add ImGui manually
set(IMGUI_DIR external/imgui)
file(GLOB IMGUI_SOURCES 
    ${IMGUI_DIR}/*.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# Your sources
set(SOURCES 
    src/parser/spice_parser.cpp
    src/simulation/dc_analysis.cpp
    src/simulation/transient_analysis.cpp
)

# Create executable
add_executable(CircuitSimulator 
    ${SOURCES}
    ${IMGUI_SOURCES}
    src/main.cpp
)

# Include directories
target_include_directories(CircuitSimulator PRIVATE 
    src/
    src/parser/
    src/simulation/
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${GLFW_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(CircuitSimulator PRIVATE 
    Eigen3::Eigen
    ${GLFW_LIBRARIES}
    OpenGL::GL
    ${CMAKE_DL_LIBS}
)

# Compiler flags
target_compile_options(CircuitSimulator PRIVATE 
    -Wall -Wextra 
    ${GLFW_CFLAGS_OTHER}
)