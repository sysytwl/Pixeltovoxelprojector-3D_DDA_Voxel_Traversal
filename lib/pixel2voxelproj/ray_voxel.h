/***************************************************
 * ray_voxel.cpp
 *
 * A "complete" C++ example:
 *   1) Parse metadata.json with nlohmann::json
 *   2) Load images (stb_image) in grayscale
 *   3) Do motion detection between consecutive frames
 *      for each camera
 *   4) Cast rays (voxel DDA) for changed pixels
 *   5) Accumulate in a shared 3D voxel grid
 *   6) Save the voxel grid to a .bin file
 ***************************************************/




#include <stdint.h>
#include <math.h>
#include <stddef.h>
#include <vector>

// Minimal vector/matrix structs for ESP32
struct Vec3 {
    float x, y, z;
};

struct Mat3 {
    float m[9];
};

class RayVoxelProcessor {
  public:
    struct ImageGray {
      int width;
      int height;
      const uint8_t *pixels;  // pointer to grayscale image data
    };

    struct MotionMask {
      int width;
      int height;
      // For ESP32, use a pointer to preallocated bool array
      bool *changed;
      float *diff;
    };

    struct RayStep {
      int ix, iy, iz;
      int step_count;
      float distance;
    };

    // User must provide a preallocated voxel grid (float*) of size N*N*N
    RayVoxelProcessor(int grid_N, float voxel_size, Vec3 grid_center, float motion_threshold = 2.0f, float alpha = 0.1f)
      : N(grid_N), voxel_size(voxel_size), grid_center(grid_center), motion_threshold(motion_threshold), alpha(alpha) {
        precompute_ray_cam_lut(image_width, image_height, fov_degrees);
      }

    // Main entry: process two consecutive grayscale images and accumulate into voxel_grid
    void process(
      const ImageGray &prev_img,
      const ImageGray &curr_img,
      const Vec3 &cam_pos,
      float yaw, float pitch, float roll,
      float fov_degrees,
      float *voxel_grid // size N*N*N
    ) {
      // 1. Detect motion
      // User must provide preallocated arrays for mask
      static bool changed[640 * 480];  // adjust as needed
      static float diff[640 * 480];
      MotionMask mm = detect_motion(prev_img, curr_img, changed, diff);

      // 2. Camera rotation matrix
      Mat3 cam_rot = rotation_matrix_yaw_pitch_roll(yaw, pitch, roll);
      float fov_rad = deg2rad(fov_degrees);
      float focal_len = (mm.width * 0.5f) / tanf(fov_rad * 0.5f);

      // 3. For each changed pixel, cast ray and accumulate
      for (int v = 0; v < mm.height; v++) {
        for (int u = 0; u < mm.width; u++) {
          int idx = v * mm.width + u;
          if (!mm.changed[idx]) continue;
          float pix_val = mm.diff[idx];
          if (pix_val < 1e-3f) continue;

          Vec3 ray_cam = ray_cam_lut[v * mm.width + u];
          Vec3 ray_world = mat3_mul_vec3(cam_rot, ray_cam);
          ray_world = normalize(ray_world);

          // DDA
          RayStep steps[128];
          int nsteps = cast_ray_into_grid(cam_pos, ray_world, steps, 128);

          // Accumulate
          for (int s = 0; s < nsteps; s++) {
            float dist = steps[s].distance;
            float val = pix_val;  // optionally apply attenuation
            int grid_idx = steps[s].ix * N * N + steps[s].iy * N + steps[s].iz;
            if (grid_idx >= 0 && grid_idx < N * N * N) {
              voxel_grid[grid_idx] += val;
            }
          }
        }
      }
    }

    // Detect motion between two images, output mask and diff arrays
    MotionMask detect_motion(const ImageGray &prev, const ImageGray &next, bool *changed, float *diff) {
      MotionMask mm;
      mm.width = prev.width;
      mm.height = prev.height;
      mm.changed = changed;
      mm.diff = diff;
      for (int i = 0; i < mm.width * mm.height; i++) {
        float d = fabsf((float)prev.pixels[i] - (float)next.pixels[i]);
        mm.diff[i] = d;
        mm.changed[i] = (d > motion_threshold);
      }
      return mm;
    }

    // Cast ray into grid, return number of steps, fill steps array
    int cast_ray_into_grid(const Vec3 &camera_pos, const Vec3 &dir_normalized, RayStep *steps, int max_steps) {
      float half_size = 0.5f * (N * voxel_size);
      Vec3 grid_min = { grid_center.x - half_size, grid_center.y - half_size, grid_center.z - half_size };
      Vec3 grid_max = { grid_center.x + half_size, grid_center.y + half_size, grid_center.z + half_size };

      float t_min = 0.f;
      float t_max = 1e9f;

      // Ray-box intersection
      for (int i = 0; i < 3; i++) {
        float origin = (i == 0) ? camera_pos.x : ((i == 1) ? camera_pos.y : camera_pos.z);
        float d = (i == 0) ? dir_normalized.x : ((i == 1) ? dir_normalized.y : dir_normalized.z);
        float mn = (i == 0) ? grid_min.x : ((i == 1) ? grid_min.y : grid_min.z);
        float mx = (i == 0) ? grid_max.x : ((i == 1) ? grid_max.y : grid_max.z);

        if (fabsf(d) < 1e-12f) {
          if (origin < mn || origin > mx) {
            return 0;
          }
        } else {
          float t1 = (mn - origin) / d;
          float t2 = (mx - origin) / d;
          float t_near = (t1 < t2) ? t1 : t2;
          float t_far = (t1 > t2) ? t1 : t2;
          if (t_near > t_min) t_min = t_near;
          if (t_far < t_max) t_max = t_far;
          if (t_min > t_max) {
            return 0;
          }
        }
      }
      if (t_min < 0.f) t_min = 0.f;

      // Start voxel
      Vec3 start_world = { camera_pos.x + t_min * dir_normalized.x,
                           camera_pos.y + t_min * dir_normalized.y,
                           camera_pos.z + t_min * dir_normalized.z };
      float fx = (start_world.x - grid_min.x) / voxel_size;
      float fy = (start_world.y - grid_min.y) / voxel_size;
      float fz = (start_world.z - grid_min.z) / voxel_size;

      int ix = (int)fx;
      int iy = (int)fy;
      int iz = (int)fz;
      if (ix < 0 || ix >= N || iy < 0 || iy >= N || iz < 0 || iz >= N) {
        return 0;
      }

      int step_x = (dir_normalized.x >= 0.f) ? 1 : -1;
      int step_y = (dir_normalized.y >= 0.f) ? 1 : -1;
      int step_z = (dir_normalized.z >= 0.f) ? 1 : -1;

      float next_bx = grid_min.x + (ix + (step_x > 0 ? 1 : 0)) * voxel_size;
      float next_by = grid_min.y + (iy + (step_y > 0 ? 1 : 0)) * voxel_size;
      float next_bz = grid_min.z + (iz + (step_z > 0 ? 1 : 0)) * voxel_size;

      float t_max_x = safe_div(next_bx - camera_pos.x, dir_normalized.x);
      float t_max_y = safe_div(next_by - camera_pos.y, dir_normalized.y);
      float t_max_z = safe_div(next_bz - camera_pos.z, dir_normalized.z);

      float t_delta_x = safe_div(voxel_size, fabsf(dir_normalized.x));
      float t_delta_y = safe_div(voxel_size, fabsf(dir_normalized.y));
      float t_delta_z = safe_div(voxel_size, fabsf(dir_normalized.z));

      float t_current = t_min;
      int step_count = 0;
      int nsteps = 0;

      while (t_current <= t_max && nsteps < max_steps) {
        steps[nsteps].ix = ix;
        steps[nsteps].iy = iy;
        steps[nsteps].iz = iz;
        steps[nsteps].step_count = step_count;
        steps[nsteps].distance = t_current;
        nsteps++;

        if (t_max_x < t_max_y && t_max_x < t_max_z) {
          ix += step_x;
          t_current = t_max_x;
          t_max_x += t_delta_x;
        } else if (t_max_y < t_max_z) {
          iy += step_y;
          t_current = t_max_y;
          t_max_y += t_delta_y;
        } else {
          iz += step_z;
          t_current = t_max_z;
          t_max_z += t_delta_z;
        }
        step_count++;
        if (ix < 0 || ix >= N || iy < 0 || iy >= N || iz < 0 || iz >= N) {
          break;
        }
      }
      return nsteps;
    }

    void precompute_ray_cam_lut(int width, int height, float fov_degrees) {
        ray_cam_lut.resize(width * height);
        float fov_rad = deg2rad(fov_degrees);
        float focal_len = (width * 0.5f) / tanf(fov_rad * 0.5f);
        for (int v = 0; v < height; v++) {
            for (int u = 0; u < width; u++) {
                float x = ((float)u - 0.5f * width);
                float y = -((float)v - 0.5f * height);
                float z = -focal_len;
                ray_cam_lut[v * width + u] = normalize({x, y, z});
            }
        }
    }

    // Math helpers
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

  private:
    int N;
    float voxel_size;
    Vec3 grid_center;
    float motion_threshold;
    float alpha;

    std::vector<Vec3> ray_cam_lut;
};
