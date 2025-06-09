// This header file declares utility functions that can be used across different modules in the project.

#ifndef VULKAN_IMAGE_PROCESSING_UTILS_H
#define VULKAN_IMAGE_PROCESSING_UTILS_H

#include <vulkan/vulkan.h>
#include <stdexcept>
#include <string>

// Function to check Vulkan result and throw an exception if there is an error
inline void checkVulkanResult(VkResult result, const std::string& message) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error(message + ": " + std::to_string(result));
    }
}

// Function to create a Vulkan buffer
VkBuffer createBuffer(VkDevice device, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkDeviceMemory& bufferMemory);

// Function to copy data to a Vulkan buffer
void copyBuffer(VkDevice device, VkCommandPool commandPool, VkQueue graphicsQueue, VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);

#endif // VULKAN_IMAGE_PROCESSING_UTILS_H