cmake_minimum_required(VERSION 3.16)
project(CircuitSimulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform-specific package finding
if(WIN32)
    # Windows-specific configuration
    find_package(OpenGL REQUIRED)
    
    # Use vcpkg for Windows - note the different case
    find_package(glfw3 CONFIG REQUIRED)
    
else()
    # Linux-specific configuration
    find_package(PkgConfig REQUIRED)
    find_package(OpenGL REQUIRED)
    
    # Use pkg-config to find GLFW3 on Linux
    pkg_check_modules(GLFW REQUIRED glfw3)
endif()

# Add ImGui manually (cross-platform)
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
)

# Platform-specific linking
if(WIN32)
    # On Windows with vcpkg, use the target directly
    target_link_libraries(CircuitSimulator PRIVATE 
        glfw  # This is the vcpkg target name
        OpenGL::GL
    )
    
    # Windows-specific compile definitions
    target_compile_definitions(CircuitSimulator PRIVATE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )
    
else()
    # Linux linking
    target_include_directories(CircuitSimulator PRIVATE ${GLFW_INCLUDE_DIRS})
    target_link_libraries(CircuitSimulator PRIVATE 
        ${GLFW_LIBRARIES}
        OpenGL::GL
        ${CMAKE_DL_LIBS}
    )
endif()

# Compiler flags
if(WIN32 AND MSVC)
    target_compile_options(CircuitSimulator PRIVATE /W0)
else()
    target_compile_options(CircuitSimulator PRIVATE 
        -Wall -Wextra -w
        ${GLFW_CFLAGS_OTHER}
    )
endif()