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
    std::cout << pixel_width << " " << pixel_height << "\n";
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


int get_collions(int width, int height, RGB* pixels,
                 const Vec3SoA& Camera_Vectors, const TriangleSoA& triangleSoA) {


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
    AlignedVector<int> collision_triangles(pixel_count, -1);
    AlignedVector<float> collision_distance(pixel_count, 1000000.0f);

    using b_float = xsimd::batch<float>;
    using b_int = xsimd::batch<int>;
    const int batch_size = b_float::size;
    int vec_size = pixel_count - (pixel_count % batch_size);

    const b_float originx = b_float(0.0f);
    const b_float originy = b_float(0.0f);
    const b_float originz = b_float(0.0f);

    const b_float epsilon = b_float(std::numeric_limits<float>::epsilon());
    std::cout << "\n batch size = " << batch_size << "\n";


    for (int j = 0; j < triangle_count; ++j) {
        const b_float edge_2x = b_float(edge2x[j]);
        const b_float edge_2y = b_float(edge2y[j]);
        const b_float edge_2z = b_float(edge2z[j]);
        const b_float edge_1x = b_float(edge1x[j]);
        const b_float edge_1y = b_float(edge1y[j]);
        const b_float edge_1z = b_float(edge1z[j]);
        const b_float v_1x = b_float(v1x[j]);
        const b_float v_1y = b_float(v1y[j]);
        const b_float v_1z = b_float(v1z[j]);


        const b_float sx = originx - v_1x;
        const b_float sy = originy - v_1y;
        const b_float sz = originz - v_1z;

        const b_float s_cross_e1x = sy * edge_1z - sz * edge_1y;
        const b_float s_cross_e1y = sz * edge_1x - sx * edge_1z;
        const b_float s_cross_e1z = sx * edge_1y - sy * edge_1x;


        for (size_t i = 0; i < vec_size; i += batch_size) {
        const b_float CamXvec = b_float::load_aligned(&cam_x[i]);
        const b_float CamYvec = b_float::load_aligned(&cam_y[i]);
        const b_float CamZvec = b_float::load_aligned(&cam_z[i]);
            // Cross product: ray x edge2
            b_float ray_cross_e2x = CamYvec * edge_2z - CamZvec * edge_2y;
            b_float ray_cross_e2y = CamZvec * edge_2x - CamXvec * edge_2z;
            b_float ray_cross_e2z = CamXvec * edge_2y - CamYvec * edge_2x;

            // const float ray_cross_e2x = dy * edge_2z - dz * edge_2y;
            // const float ray_cross_e2y = dz * edge_2x - dx * edge_2z;
            // const float ray_cross_e2z = dx * edge_2y - dy * edge_2x;

            // Determinant
            const b_float det = edge_1x * ray_cross_e2x +
                              edge_1y * ray_cross_e2y +
                              edge_1z * ray_cross_e2z;

        //     if (fabsf(det) <= epsilon){
        //         continue;
        // }
            auto valid_det = (det < epsilon);

            const b_float inv_det = xsimd::select(valid_det, 1.0f / det, b_float(0.0f));

            // s = origin - v1

            // u parameter
            const b_float u = inv_det * (sx * ray_cross_e2x +
                                       sy * ray_cross_e2y +
                                       sz * ray_cross_e2z);
            auto valid_u = (u >= -epsilon) & (u <= 1.0f + epsilon);
        //     if (__builtin_expect(u < -epsilon || u > 1.0f + epsilon, 0)){
        //         continue;
        // }

            // s x edge1

            // v parameter
            const b_float v = inv_det * (CamXvec * s_cross_e1x +
                                       CamYvec * s_cross_e1y +
                                       CamZvec * s_cross_e1z);

            auto valid_v = (v >= -epsilon) & ((u + v) <= 1.0f + epsilon);
        //     if (v < -epsilon || (u + v) > 1.0f + epsilon){
        //     continue;
        // }

            // t parameter
            const b_float t = inv_det * (edge_2x * s_cross_e1x +
                                       edge_2y * s_cross_e1y +
                                       edge_2z * s_cross_e1z);
            b_float old_dist = b_float::load_aligned(&collision_distance[i]);

            auto is_closer = t < old_dist;
            auto valid_mask = valid_det & valid_u & valid_v & is_closer;
            auto valid_mask_int = xsimd::bit_cast<typename b_int::batch_bool_type>(valid_mask);
            b_float new_dist = xsimd::select(valid_mask, t, old_dist);
            const b_int old_tri = b_int::load_aligned(&collision_triangles[i]);
            const b_int new_tri = b_int(j);
            b_int updated_tri = xsimd::select(valid_mask_int, new_tri, old_tri);
            updated_tri.store_aligned(&collision_triangles[i]);

            new_dist.store_aligned(&collision_distance[i]);

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
    Model test = Model("../monkey.obj");
    test.move_position(Vec3{0,0,3});
    TriangleSoA trisoa = TriToSoA(test.triangles);
    int width = 1920;
    int height = 1080;
    RGB* pixels = new RGB[width*height];
    Vec3SoA Camera_Vectors(width*height);
    MakeCameraNormal(width, height, pixels, Camera_Vectors, 0.5, 16.0f/8.0f, 9.0f/8.0f);
    get_collions(width,height,pixels,Camera_Vectors, trisoa );

    stbi_write_bmp("Test.bmp", width,height,3,pixels);
    std::cout << std::chrono::high_resolution_clock::now() - start;
    return 0;
 
}
