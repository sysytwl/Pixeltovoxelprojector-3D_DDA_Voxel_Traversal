cmake_minimum_required(VERSION 3.10)

project(VulkanImageProcessing)

set(CMAKE_CXX_STANDARD 17)

# Find Vulkan
find_package(Vulkan REQUIRED)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/main.cpp
    src/vulkan_image_processor.cpp
    src/utils.cpp
)

# Add executable
add_executable(VulkanImageProcessing ${SOURCES})

# Link Vulkan
target_link_libraries(VulkanImageProcessing Vulkan::Vulkan)

# Add shader files as custom commands
file(GLOB SHADERS "shaders/*.comp" "shaders/*.glsl")
foreach(SHADER ${SHADERS})
    get_filename_component(SHADER_NAME ${SHADER} NAME)
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${SHADER_NAME}
        COMMAND ${CMAKE_COMMAND} -E copy ${SHADER} ${CMAKE_CURRENT_BINARY_DIR}/${SHADER_NAME}
        DEPENDS ${SHADER}
    )
endforeach()

# Ensure shaders are built before the executable
add_custom_target(Shaders ALL DEPENDS ${SHADERS})