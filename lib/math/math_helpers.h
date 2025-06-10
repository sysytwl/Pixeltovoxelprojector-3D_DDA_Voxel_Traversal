#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stddef.h>

// Minimal vector/matrix structs for ESP32
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
