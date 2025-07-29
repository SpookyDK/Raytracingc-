#include <iostream>
#include "stb_image/stb_image.h"
#include <stdlib.h>
#include <chrono>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image/stb_image_write.h"
#include "object-importer.h"


// #include "xsimd/xsimd.hpp"
//
//



int MakeCameraNormal(int width, int height, RGB* pixels, Vec3SoA& Camera_Vectors, float camera_plane_distance, float camera_plane_width, float camera_plane_height)
{
    using batch = xsimd::batch<float>;
    const size_t simd_size = batch::size;

    const size_t amount = width * height;

    const float pixel_width = camera_plane_width / width;
    const float pixel_height = camera_plane_height / height;
    const float half_width = width / 2.0f;
    const float half_height = height / 2.0f;

    std::vector<float> tmp_x(amount);
    std::vector<float> tmp_y(amount);
    std::vector<float> tmp_z(amount);

    // Fill unnormalized direction vectors (scalar)
    for(size_t i = 0; i < amount; i++) {
        int row = i / width;
        int col = i % width;

        float x = (col - half_width) * pixel_width;
        float y = -(row - half_height) * pixel_height;
        float z = camera_plane_distance;

        tmp_x[i] = x;
        tmp_y[i] = y;
        tmp_z[i] = z;
    }

    // Normalize vectors using SIMD
    size_t i = 0;
    for (; i + simd_size <= amount; i += simd_size) {
        batch bx = batch::load_unaligned(&tmp_x[i]);
        batch by = batch::load_unaligned(&tmp_y[i]);
        batch bz = batch::load_unaligned(&tmp_z[i]);

        batch len = xsimd::sqrt(bx * bx + by * by + bz * bz);
        bx /= len;
        by /= len;
        bz /= len;

        bx.store_unaligned(&Camera_Vectors.x[i]);
        by.store_unaligned(&Camera_Vectors.y[i]);
        bz.store_unaligned(&Camera_Vectors.z[i]);
    }

    // Handle remaining pixels (scalar)
    for (; i < amount; ++i) {
        float x = tmp_x[i];
        float y = tmp_y[i];
        float z = tmp_z[i];
        float length = std::sqrt(x*x + y*y + z*z);
        Camera_Vectors.x[i] = x / length;
        Camera_Vectors.y[i] = y / length;
        Camera_Vectors.z[i] = z / length;
    }

    return 1;
}

    constexpr float epsilon = std::numeric_limits<float>::epsilon();

int get_collions(int width, int height, RGB* pixels,
                 const Vec3SoA& Camera_Vectors, const TriangleSoA& triangleSoA) {
    const float originx = 0.0f;
    const float originy = 0.0f;
    const float originz = 0.0f;


    // Get raw restrict pointers for camera rays
    const float* __restrict cam_x = Camera_Vectors.x.data();
    const float* __restrict cam_y = Camera_Vectors.y.data();
    const float* __restrict cam_z = Camera_Vectors.z.data();

    // Get raw restrict pointers for triangle SoA
    const float* __restrict v1x = triangleSoA.v1.x.data();
    const float* __restrict v1y = triangleSoA.v1.y.data();
    const float* __restrict v1z = triangleSoA.v1.z.data();

    const float* __restrict edge1x = triangleSoA.edge1.x.data();
    const float* __restrict edge1y = triangleSoA.edge1.y.data();
    const float* __restrict edge1z = triangleSoA.edge1.z.data();

    const float* __restrict edge2x = triangleSoA.edge2.x.data();
    const float* __restrict edge2y = triangleSoA.edge2.y.data();
    const float* __restrict edge2z = triangleSoA.edge2.z.data();

    const float* __restrict nx = triangleSoA.n.x.data();
    const float* __restrict ny = triangleSoA.n.y.data();
    const float* __restrict nz = triangleSoA.n.z.data();

    const size_t triangle_count = triangleSoA.v1.x.size();
    std::cout << triangle_count << "kage";
    size_t pixel_count = static_cast<size_t>(width) * height;
    std::vector<int> collision_triangles(pixel_count, -1);
    std::vector<float> collision_distance(pixel_count, 1000000.0f);

    for (int j = 0; j < triangle_count; ++j) {
        const float edge_2x = edge2x[j];
        const float edge_2y = edge2y[j];
        const float edge_2z = edge2z[j];
        const float edge_1x = edge1x[j];
        const float edge_1y = edge1y[j];
        const float edge_1z = edge1z[j];
        const float v_1x = v1x[j];
        const float v_1y = v1y[j];
        const float v_1z = v1z[j];


        const float sx = originx - v_1x;
        const float sy = originy - v_1y;
        const float sz = originz - v_1z;

        for (size_t i = 0; i < width * height; ++i) {
        const float dx = cam_x[i];
        const float dy = cam_y[i];
        const float dz = cam_z[i];
            // Cross product: ray x edge2
            const float ray_cross_e2x = dy * edge_2z - dz * edge_2y;
            const float ray_cross_e2y = dz * edge_2x - dx * edge_2z;
            const float ray_cross_e2z = dx * edge_2y - dy * edge_2x;

            // Determinant
            const float det = edge_1x * ray_cross_e2x +
                              edge_1y * ray_cross_e2y +
                              edge_1z * ray_cross_e2z;

            if (fabsf(det) <= epsilon){
                continue;
        }

            const float inv_det = 1.0f / det;

            // s = origin - v1

            // u parameter
            const float u = inv_det * (sx * ray_cross_e2x +
                                       sy * ray_cross_e2y +
                                       sz * ray_cross_e2z);
            if (__builtin_expect(u < -epsilon || u > 1.0f + epsilon, 0)){
                continue;
        }

            // s x edge1
            const float s_cross_e1x = sy * edge_1z - sz * edge_1y;
            const float s_cross_e1y = sz * edge_1x - sx * edge_1z;
            const float s_cross_e1z = sx * edge_1y - sy * edge_1x;

            // v parameter
            const float v = inv_det * (dx * s_cross_e1x +
                                       dy * s_cross_e1y +
                                       dz * s_cross_e1z);

            if (v < -epsilon || (u + v) > 1.0f + epsilon){
            continue;
        }

            // t parameter
            const float t = inv_det * (edge_2x * s_cross_e1x +
                                       edge_2y * s_cross_e1y +
                                       edge_2z * s_cross_e1z);

            if (t < collision_distance[i]) {
                collision_distance[i] = t;
                collision_triangles[i] = j;
                // std::cout << "coliiions \n";
            }
        }
    }

    // Write out colors
    for (int i = 0; i < width * height; ++i) {
        if (collision_triangles[i] >= 0) {
            const int j = collision_triangles[i];
            pixels[i] = {
                (unsigned char)((nx[j] / 2.0f + 1.0f) * 255.0f),
                (unsigned char)((ny[j] / 2.0f + 1.0f) * 255.0f),
                (unsigned char)((nz[j] / 2.0f + 1.0f) * 255.0f)
            };
        } else {
            pixels[i] = {0, 0, 0};
        }
    }

    return 1;
}

int main() {
    auto start = std::chrono::high_resolution_clock::now();
    Model test = Model("../sphere.obj");
    test.move_position(Vec3{0,0,3});
    TriangleSoA trisoa = TriToSoA(test.triangles);
    int width = 1024;
    int height = 1024;
    RGB* pixels = new RGB[width*height];
    Vec3SoA Camera_Vectors(width*height);
    MakeCameraNormal(width, height, pixels, Camera_Vectors, 0.5, 16/5, 16/5);
    get_collions(width,height,pixels,Camera_Vectors, trisoa );

    stbi_write_bmp("Test.bmp", width,height,3,pixels);
    std::cout << std::chrono::high_resolution_clock::now() - start;
    return 0;
 
}
