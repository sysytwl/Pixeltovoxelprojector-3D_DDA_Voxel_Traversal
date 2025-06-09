#ifndef VULKAN_IMAGE_PROCESSOR_H
#define VULKAN_IMAGE_PROCESSOR_H

#include <vulkan/vulkan.h>
#include <vector>
#include <array>

class VulkanImageProcessor {
public:
    VulkanImageProcessor();
    ~VulkanImageProcessor();

    void initialize(VkInstance instance, VkDevice device, VkCommandPool commandPool);
    void processImage(VkImage inputImage, VkImage outputImage, uint32_t width, uint32_t height);
    void cleanup();

private:
    VkInstance instance;
    VkDevice device;
    VkCommandPool commandPool;
    VkPipeline computePipeline;
    VkPipelineLayout pipelineLayout;
    VkDescriptorSet descriptorSet;
    VkDescriptorPool descriptorPool;

    void createComputePipeline();
    void createDescriptorSet();
    void updateDescriptorSet(VkImage inputImage, VkImage outputImage);
};

#endif // VULKAN_IMAGE_PROCESSOR_H