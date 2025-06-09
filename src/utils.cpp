#include "utils.h"
#include <stdexcept>
#include <vulkan/vulkan.h>

namespace vulkan_image_processing {

void check_vk_result(VkResult result) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error("Vulkan error: " + std::to_string(result));
    }
}

// Additional utility functions can be added here

} // namespace vulkan_image_processing