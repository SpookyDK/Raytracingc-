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



int MakeCameraNormal(int width, int height, RGB* pixels, Vec3SoA Camera_Vectors, float camera_plane_distance, float camera_plane_width, float camera_plane_height)
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
        Camera_Vectors.set(i, Vec3{x,y,z});
        // Uncomment for normal output to image
        // unsigned char r = (unsigned char)((x * 0.5 + 0.5)*255);
        // unsigned char g = (unsigned char)((y * 0.5 + 0.5)*255);
        // unsigned char b = (unsigned char)((z * 0.5 + 0.5)*255);
        // pixels[i] = {r, g, b};
    }
    return 1;
}

    constexpr float epsilon = std::numeric_limits<float>::epsilon();

int get_collions(int width, int height, RGB* pixels, const Vec3SoA Camera_Vectors, const TriangleSoA& triangleSoA){
    Vec3 origin = Vec3{0,0,0};
    int collision_triangles[width*height];



    // Pixels loop
    for (int i = 0; i < width * height; i++){
        float min_dist = 100000;
        collision_triangles[i] = -1;


        // Triangle loop
        for (int j = 0; j < triangleSoA.v1.x.size(); j++){
            float t = 0;
            Vec3 ray_cross_e2 = cross_at(Camera_Vectors, i, triangleSoA.edge2, j);
            // Vec3 ray_cross_e2 = ray.cross(triangleSoA.edge2.get(j));
            float det = dot(triangleSoA.edge1.get(j), ray_cross_e2);

            if (fabsf(det) <= epsilon)
                continue;

            float inv_det = 1.0 / det;
            Vec3 s = origin - triangleSoA.v1.get(j);
            float u = inv_det * dot(s, ray_cross_e2);
            if (__builtin_expect(u < -epsilon || u > 1.0f + epsilon, 0))
                continue;

            Vec3 s_cross_e1 = s.cross(triangleSoA.edge1.get(j));
            float v = inv_det * dot(Camera_Vectors.get(j), s_cross_e1);

            if (v < -epsilon || (u + v) > 1.0f + epsilon)
                continue;

            t = inv_det * dot(triangleSoA.edge2.get(j), s_cross_e1);
            if (t < min_dist){
                min_dist = t;
                collision_triangles[i] = j;
            }
        }
    }
    for (int i = 0; i < width * height; i++)
        if (collision_triangles[i] >= 0)
        {
            pixels[i] = {(unsigned char)((triangleSoA.n.get(collision_triangles[i]).x / 2 + 1) * 255),
                         (unsigned char)((triangleSoA.n.get(collision_triangles[i]).y/ 2 +1) * 255),
                         (unsigned char)((triangleSoA.n.get(collision_triangles[i]).z/ 2 + 1) * 255)};
        }
        else {pixels[i] = {0,0,0};}

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
