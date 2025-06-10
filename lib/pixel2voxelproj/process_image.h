#include <stdint.h>
#include <math.h>
#include <stddef.h>

struct Vec3 {
    float x, y, z;
};

class ImageVoxelProcessor {
public:
    // Process a grayscale image and accumulate into a voxel grid
    // image: pointer to grayscale image data (uint8_t, 0..255)
    // width, height: image dimensions
    // earth_pos: observer position
    // pointing_dir: normalized direction vector
    // fov_deg: field of view in degrees
    // voxel_grid: pointer to float grid of size nx*ny*nz
    // nx, ny, nz: voxel grid dimensions
    // grid_min, grid_max: bounding box of voxel grid
    void process(
        const uint8_t* image, int width, int height,
        const Vec3& earth_pos,
        const Vec3& pointing_dir,
        float fov_deg,
        float* voxel_grid, int nx, int ny, int nz,
        const Vec3& grid_min, const Vec3& grid_max,
        float max_distance, int num_steps
    ) {
        float fov_rad = deg2rad(fov_deg);
        float focal_length = (width / 2.0f) / tanf(fov_rad / 2.0f);
        float cx = width / 2.0f;
        float cy = height / 2.0f;

        Vec3 z_axis = normalize(pointing_dir);
        Vec3 up = {0.f, 0.f, 1.f};
        if (fabsf(z_axis.x - up.x) < 1e-6f && fabsf(z_axis.y - up.y) < 1e-6f && fabsf(z_axis.z - up.z) < 1e-6f)
            up = {0.f, 1.f, 0.f};

        Vec3 x_axis = cross(up, z_axis);
        x_axis = normalize(x_axis);
        Vec3 y_axis = cross(z_axis, x_axis);

        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                uint8_t pix = image[i * width + j];
                if (pix == 0) continue;

                float x_cam = (float(j) - cx);
                float y_cam = (float(i) - cy);
                float z_cam = focal_length;

                float norm = sqrtf(x_cam*x_cam + y_cam*y_cam + z_cam*z_cam);
                float dir_cam[3] = { x_cam/norm, y_cam/norm, z_cam/norm };

                // Camera to world
                Vec3 dir_world = {
                    x_axis.x*dir_cam[0] + y_axis.x*dir_cam[1] + z_axis.x*dir_cam[2],
                    x_axis.y*dir_cam[0] + y_axis.y*dir_cam[1] + z_axis.y*dir_cam[2],
                    x_axis.z*dir_cam[0] + y_axis.z*dir_cam[1] + z_axis.z*dir_cam[2]
                };
                dir_world = normalize(dir_world);

                // Ray-box intersection
                float t_entry, t_exit;
                if (!ray_aabb_intersection(earth_pos, dir_world, grid_min, grid_max, t_entry, t_exit))
                    continue;

                t_entry = t_entry > 0.f ? t_entry : 0.f;
                t_exit = t_exit < max_distance ? t_exit : max_distance;
                if (t_entry > t_exit) continue;

                float step_size = (t_exit - t_entry) / num_steps;
                for (int s = 0; s < num_steps; ++s) {
                    float t = t_entry + s * step_size;
                    Vec3 p = {
                        earth_pos.x + t * dir_world.x,
                        earth_pos.y + t * dir_world.y,
                        earth_pos.z + t * dir_world.z
                    };
                    int xi = (int)((p.x - grid_min.x) / (grid_max.x - grid_min.x) * nx);
                    int yi = (int)((p.y - grid_min.y) / (grid_max.y - grid_min.y) * ny);
                    int zi = (int)((p.z - grid_min.z) / (grid_max.z - grid_min.z) * nz);
                    if (xi >= 0 && xi < nx && yi >= 0 && yi < ny && zi >= 0 && zi < nz) {
                        int idx = xi * ny * nz + yi * nz + zi;
                        voxel_grid[idx] += (float)pix;
                    }
                }
            }
        }
    }

    // Ray-AABB intersection
    static bool ray_aabb_intersection(const Vec3& origin, const Vec3& dir, const Vec3& box_min, const Vec3& box_max, float& t_entry, float& t_exit) {
        float tmin = -1e9f, tmax = 1e9f;
        for (int i = 0; i < 3; ++i) {
            float o = (&origin.x)[i];
            float d = (&dir.x)[i];
            float mn = (&box_min.x)[i];
            float mx = (&box_max.x)[i];
            if (fabsf(d) > 1e-8f) {
                float t1 = (mn - o) / d;
                float t2 = (mx - o) / d;
                if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
                if (t1 > tmin) tmin = t1;
                if (t2 < tmax) tmax = t2;
                if (tmin > tmax) return false;
            } else {
                if (o < mn || o > mx) return false;
            }
        }
        t_entry = tmin;
        t_exit = tmax;
        return true;
    }

    static Vec3 normalize(const Vec3& v) {
        float len = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
        if (len < 1e-12f) return {0.f, 0.f, 0.f};
        return { v.x/len, v.y/len, v.z/len };
    }
    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {
            a.y*b.z - a.z*b.y,
            a.z*b.x - a.x*b.z,
            a.x*b.y - a.y*b.x
        };
    }
    static float deg2rad(float deg) {
        return deg * 3.14159265358979323846f / 180.0f;
    }
};
