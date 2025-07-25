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
    for(int i = 0; i < width * height; i++){
        int row = i / width - height / 2;
        int col = i % width - width / 2;
        float pixel_width = camera_plane_width / width;
        float pixel_height = camera_plane_height / height;
        float x = col * pixel_width;
        float y = -row * pixel_height;
        float z = camera_plane_distance;
        float length = std::sqrt(x*x+y*y+z*z);
        x /= length;
        y /= length;
        z /= length;
        Camera_Vectors.set(i, x,y,z);
        // Uncomment for normal output to image
        // unsigned char r = (unsigned char)((x * 0.5 + 0.5)*255);
        // unsigned char g = (unsigned char)((y * 0.5 + 0.5)*255);
        // unsigned char b = (unsigned char)((z * 0.5 + 0.5)*255);
        // pixels[i] = {r, g, b};
    }
    return 1;
}

    constexpr float epsilon = std::numeric_limits<float>::epsilon();

int get_collions(int width, int height, RGB* pixels,
                 const Vec3SoA& Camera_Vectors, const TriangleSoA& triangleSoA) {
    const float originx = 0.0f;
    const float originy = 0.0f;
    const float originz = 0.0f;

    int collision_triangles[width * height];

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

    // Pixel loop
    for (int i = 0; i < width * height; ++i) {
        float min_dist = 100000.0f;
        collision_triangles[i] = -1;

        const float dx = cam_x[i];
        const float dy = cam_y[i];
        const float dz = cam_z[i];

        for (size_t j = 0; j < triangle_count; ++j) {
            // Cross product: ray x edge2
            const float ray_cross_e2x = dy * edge2z[j] - dz * edge2y[j];
            const float ray_cross_e2y = dz * edge2x[j] - dx * edge2z[j];
            const float ray_cross_e2z = dx * edge2y[j] - dy * edge2x[j];

            // Determinant
            const float det = edge1x[j] * ray_cross_e2x +
                              edge1y[j] * ray_cross_e2y +
                              edge1z[j] * ray_cross_e2z;

            if (fabsf(det) <= epsilon)
                continue;

            const float inv_det = 1.0f / det;

            // s = origin - v1
            const float sx = originx - v1x[j];
            const float sy = originy - v1y[j];
            const float sz = originz - v1z[j];

            // u parameter
            const float u = inv_det * (sx * ray_cross_e2x +
                                       sy * ray_cross_e2y +
                                       sz * ray_cross_e2z);
            if (__builtin_expect(u < -epsilon || u > 1.0f + epsilon, 0))
                continue;

            // s x edge1
            const float s_cross_e1x = sy * edge1z[j] - sz * edge1y[j];
            const float s_cross_e1y = sz * edge1x[j] - sx * edge1z[j];
            const float s_cross_e1z = sx * edge1y[j] - sy * edge1x[j];

            // v parameter
            const float v = inv_det * (dx * s_cross_e1x +
                                       dy * s_cross_e1y +
                                       dz * s_cross_e1z);

            if (v < -epsilon || (u + v) > 1.0f + epsilon)
                continue;

            // t parameter
            const float t = inv_det * (edge2x[j] * s_cross_e1x +
                                       edge2y[j] * s_cross_e1y +
                                       edge2z[j] * s_cross_e1z);

            if (t < min_dist) {
                min_dist = t;
                collision_triangles[i] = j;
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
    int width = 512;
    int height = 512;
    RGB* pixels = new RGB[width*height];
    Vec3SoA Camera_Vectors(width*height);
    MakeCameraNormal(width, height, pixels, Camera_Vectors, 0.5, 16/5, 16/5);
    get_collions(width,height,pixels,Camera_Vectors, trisoa );

    stbi_write_bmp("Test.bmp", width,height,3,pixels);
    std::cout << std::chrono::high_resolution_clock::now() - start;
    return 0;
 
}
