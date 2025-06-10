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
#include <stdio.h>
#include <math.h>
#include <stddef.h>

#include <Arduino.h>

#include "math_helpers.h"



class RayCasting {
public:
  struct RayStep {
    int ix, iy, iz;
    int step_count;
    float distance;
  };

  void init(int width, int height, float fov_degrees){
    _width = width;
    _height = height;
    precompute_ray_cam_lut(width, height, fov_degrees);
  }

  uint32_t RayMatch(uint8_t *img, float16_vec3_t *RayList, bool world=false, float yaw=0, float pitch=0, float roll=0){
    uint32_t RayListlen = 0;
    for (uint16_t y=0; y<_height; y++){
      for (uint16_t x=0; x<_width; x++){
        if (img[y*_width + x]){
          RayList[RayListlen] = ray_cam_lut[y*_width + x];
          RayListlen++;
        }

      }
    }

    if(world){
      Mat3 cam_rot = rotation_matrix_yaw_pitch_roll(yaw, pitch, roll);
      for (uint32_t i = 0; i < RayListlen; i++) {
        ////RayList[i] = mat3_mul_vec3(cam_rot, RayList[i]);
        ////RayList[i] = normalize(RayList[i]);
      }
    }

    return RayListlen;
  }

          // // DDA
          // RayStep steps[128];
          // int nsteps = cast_ray_into_grid(cam_pos, ray_world, steps, 128);

          // // Accumulate
          // for (int s = 0; s < nsteps; s++) {
          //   float dist = steps[s].distance;
          //   float val = pix_val;  // optionally apply attenuation
          //   int grid_idx = steps[s].ix * N * N + steps[s].iy * N + steps[s].iz;
          //   if (grid_idx >= 0 && grid_idx < N * N * N) {
          //     voxel_grid[grid_idx] += val;
          //   }
          // }

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

private:

  uint16_t _width, _height;

  int N;
  float voxel_size;
  Vec3 grid_center;

  float alpha;
  float16_vec3_t *ray_cam_lut;

  void precompute_ray_cam_lut(int width, int height, float fov_degrees) {
    // Try heap_caps_malloc, fallback to malloc for debugging
    ray_cam_lut = (float16_vec3_t*)heap_caps_malloc(width * height * sizeof(float16_vec3_t), MALLOC_CAP_SPIRAM);
    if (!ray_cam_lut) {
      printf("Both heap_caps_malloc and malloc failed! width=%d height=%d size=%zu\n", width, height, width * height * sizeof(float16_vec3_t));
      while(1);
    }
  
    float fov_rad = deg2rad(fov_degrees);
    float focal_len = (width * 0.5f) / tanf(fov_rad * 0.5f);
    for (int v = 0; v < height; v++) {
      for (int u = 0; u < width; u++) {
        float x = ((float)u - 0.5f * width);
        float y = -((float)v - 0.5f * height);
        float z = -focal_len;

        Vec3 tmpNormaized = normalize({x, y, z});
        ray_cam_lut[v * width + u] = {float32_to_float16(tmpNormaized.x),float32_to_float16(tmpNormaized.y),float32_to_float16(tmpNormaized.z)};
      }
    }
  }


};
