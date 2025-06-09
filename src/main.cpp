#include <vulkan/vulkan.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include "vulkan_image_processor.h"

int main() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Create a windowed mode window and its OpenGL context
    GLFWwindow* window = glfwCreateWindow(800, 600, "Vulkan Image Processing", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    // Initialize Vulkan
    VulkanImageProcessor imageProcessor;
    if (!imageProcessor.initialize()) {
        std::cerr << "Failed to initialize Vulkan" << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return -1;
    }

    // Main application loop
    while (!glfwWindowShouldClose(window)) {
        // Process images using Vulkan
        imageProcessor.processImages();

        // Poll for and process events
        glfwPollEvents();
    }

    // Cleanup
    imageProcessor.cleanup();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}