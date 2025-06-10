#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stddef.h>

// Minimal vector/matrix structs for ESP32
typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t z;
} float16_vec3_t;
uint16_t float32_to_float16(float value) {
  uint32_t f;
  memcpy(&f, &value, sizeof(f));

  uint32_t sign = (f >> 16) & 0x8000;
  uint32_t exponent = ((f >> 23) & 0xFF) - 127 + 15;
  uint32_t mantissa = f & 0x7FFFFF;

  if (exponent <= 0) {
      return (uint16_t)sign;
  } else if (exponent >= 31) {
      return (uint16_t)(sign | 0x7C00); // Inf
  }

  return (uint16_t)(sign | (exponent << 10) | (mantissa >> 13));
}
float float16_to_float32(uint16_t h) {
    uint32_t sign = (h & 0x8000) << 16;
    uint32_t exponent = (h & 0x7C00) >> 10;
    uint32_t mantissa = (h & 0x03FF);

    uint32_t f;

    if (exponent == 0) {
        if (mantissa == 0) {
            f = sign;
        } else {
            // Subnormal number
            exponent = 1;
            while ((mantissa & 0x0400) == 0) {
                mantissa <<= 1;
                exponent--;
            }
            mantissa &= 0x3FF;
            f = sign | ((exponent + 127 - 15) << 23) | (mantissa << 13);
        }
    } else if (exponent == 0x1F) {
        // Inf or NaN
        f = sign | 0x7F800000 | (mantissa << 13);
    } else {
        // Normal number
        f = sign | ((exponent + 127 - 15) << 23) | (mantissa << 13);
    }

    float result;
    memcpy(&result, &f, sizeof(result));
    return result;
}


typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t z;
} float8_vec3_t;
uint8_t float32_to_float8(float f32) {
  uint32_t f;
  memcpy(&f, &f32, 4);

  uint32_t sign = (f >> 31) & 0x1;
  int32_t exponent = ((f >> 23) & 0xFF) - 127;
  uint32_t mantissa = f & 0x7FFFFF;

  // Clamp exponent to [-7, 8]
  if (exponent < -7) return (uint8_t)(sign << 7);          // Zero
  if (exponent > 8) return (uint8_t)((sign << 7) | 0x7F);  // Max value

  uint8_t exp = (exponent + 7) & 0xF;  // 4 bits
  uint8_t man = (mantissa >> (23 - 3)) & 0x7;  // 3 bits

  return (sign << 7) | (exp << 3) | man;
}
float float8_to_float32(uint8_t f8) {
  uint8_t sign = (f8 >> 7) & 0x1;
  uint8_t exp  = (f8 >> 3) & 0xF;
  uint8_t man  = f8 & 0x7;

  int32_t exponent = exp - 7;
  uint32_t mantissa = man << (23 - 3);
  uint32_t f = (sign << 31) | ((exponent + 127) << 23) | mantissa;

  float result;
  memcpy(&result, &f, 4);
  return result;
}


struct Vec3 {
  float x, y, z;
};

struct Mat3 {
  float m[9];
};

static float deg2rad(float deg) {
  return deg * 3.14159265358979323846f / 180.0f;
}

static Vec3 normalize(const Vec3 &v) {
  float len = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
  if (len < 1e-12f) return {0.f, 0.f, 0.f};
  return { v.x / len, v.y / len, v.z / len };
}

static Vec3 mat3_mul_vec3(const Mat3 &M, const Vec3 &v) {
  Vec3 r;
  r.x = M.m[0] * v.x + M.m[1] * v.y + M.m[2] * v.z;
  r.y = M.m[3] * v.x + M.m[4] * v.y + M.m[5] * v.z;
  r.z = M.m[6] * v.x + M.m[7] * v.y + M.m[8] * v.z;
  return r;
}

static float safe_div(float num, float den) {
  float eps = 1e-12f;
  if (fabsf(den) < eps) return 1e9f;
  return num / den;
}

static void matmul3x3(const float A[9], const float B[9], float C[9]) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      C[row * 3 + col] =
        A[row * 3 + 0] * B[0 * 3 + col] +
        A[row * 3 + 1] * B[1 * 3 + col] +
        A[row * 3 + 2] * B[2 * 3 + col];
    }
  }
}

static Mat3 rotation_matrix_yaw_pitch_roll(float yaw_deg, float pitch_deg, float roll_deg) {
  float y = deg2rad(yaw_deg);
  float p = deg2rad(pitch_deg);
  float r = deg2rad(roll_deg);

  float cy = cosf(y), sy = sinf(y);
  float Rz[9] = {
    cy, -sy, 0.f,
    sy,  cy, 0.f,
    0.f, 0.f, 1.f
  };

  float cr = cosf(r), sr = sinf(r);
  float Ry[9] = {
    cr,  0.f, sr,
    0.f, 1.f, 0.f,
    -sr, 0.f, cr
  };

  float cp = cosf(p), sp = sinf(p);
  float Rx[9] = {
    1.f,  0.f,  0.f,
    0.f,  cp,  -sp,
    0.f,  sp,   cp
  };

  float Rtemp[9], Rfinal[9];
  matmul3x3(Rz, Ry, Rtemp);
  matmul3x3(Rtemp, Rx, Rfinal);

  Mat3 out;
  for (int i = 0; i < 9; i++) out.m[i] = Rfinal[i];
  return out;
}
