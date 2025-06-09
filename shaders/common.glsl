// This file contains common GLSL code that can be shared between different shaders, such as utility functions or shared constants.

#version 450

// Common utility functions
float linearToGamma(float value) {
    return pow(value, 1.0 / 2.2);
}

float gammaToLinear(float value) {
    return pow(value, 2.2);
}

// Shared constants
const float PI = 3.14159265358979323846;