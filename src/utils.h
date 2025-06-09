// utils.h
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

// Function to allocate memory for Vulkan resources
VkDeviceMemory allocateMemory(VkDevice device, const VkMemoryAllocateInfo& allocateInfo);

// Function to free Vulkan memory
void freeMemory(VkDevice device, VkDeviceMemory memory);

#endif // VULKAN_IMAGE_PROCESSING_UTILS_H