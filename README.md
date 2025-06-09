# Vulkan Image Processing Project

This project implements an image processing application using the Vulkan API. It leverages the power of modern GPUs to accelerate image processing tasks through compute shaders.

## Project Structure

The project is organized as follows:

```
vulkan_image_processing
├── src
│   ├── main.cpp                # Entry point of the application
│   ├── vulkan_image_processor.cpp # Implementation of the VulkanImageProcessor class
│   ├── vulkan_image_processor.h   # Header for the VulkanImageProcessor class
│   ├── utils.cpp                # Utility functions for Vulkan setup
│   └── utils.h                  # Header for utility functions
├── include
│   └── vulkan_image_processing
│       ├── vulkan_image_processor.h # Public interface for VulkanImageProcessor
│       └── utils.h                  # Header for utility functions
├── shaders
│   ├── image_processing.comp    # GLSL compute shader for image processing
│   └── common.glsl             # Common GLSL code for shared utilities
├── CMakeLists.txt              # CMake configuration file
├── README.md                   # Project documentation
└── third_party                  # Optional Vulkan SDK or helper libraries
```

## Setup Instructions

1. **Install Vulkan SDK**: Download and install the Vulkan SDK from the official LunarG website. Follow the installation instructions for your operating system.

2. **Clone the Repository**: Clone this repository to your local machine.

3. **Build the Project**:
   - Navigate to the project directory.
   - Create a build directory: `mkdir build && cd build`.
   - Run CMake: `cmake ..`.
   - Build the project: `make` (or `cmake --build .`).

4. **Run the Application**: After building, you can run the application from the build directory.

## Usage

The application processes images using Vulkan compute shaders. You can modify the shader code in the `shaders` directory to customize the image processing tasks.

## Contributing

Contributions are welcome! Please feel free to submit a pull request or open an issue for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.