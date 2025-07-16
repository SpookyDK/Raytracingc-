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



int MakeCameraNormal(int width, int height, RGB* pixels, Vec3* Camera_Vectors, float camera_plane_distance, float camera_plane_width, float camera_plane_height)
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
        Camera_Vectors[i] = {x,y,z};
        // Uncomment for normal output to image
        // unsigned char r = (unsigned char)((x * 0.5 + 0.5)*255);
        // unsigned char g = (unsigned char)((y * 0.5 + 0.5)*255);
        // unsigned char b = (unsigned char)((z * 0.5 + 0.5)*255);
        // pixels[i] = {r, g, b};
    }
    return 1;
}

    constexpr float epsilon = std::numeric_limits<float>::epsilon();
float GetRayIntersection_T(Vec3 ray, Vec3 origin, Triangle tri){
// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

    Vec3 edge1 = tri.v2 - tri.v1;
    Vec3 edge2 = tri.v3 - tri.v1;
    Vec3 ray_cross_e2 = ray.cross(edge2);
    float det = edge1.dot(ray_cross_e2);

    if (fabsf(det) <= epsilon)
        return 0;

    float inv_det = 1.0 / det;
    Vec3 s = origin - tri.v1;
    float u = inv_det * s.dot(ray_cross_e2);

    if (u < -epsilon || u > 1 + epsilon)
        return 0;

    Vec3 s_cross_e1 = s.cross(edge1);
    float v = inv_det * ray.dot(s_cross_e1);

    if (v < -epsilon || (u + v) > 1.0f + epsilon)
        return 0;

    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = inv_det * edge2.dot(s_cross_e1);

    return t;
}

int get_collions(int width, int height, RGB* pixels, const Vec3* Camera_Vectors, const std::vector<Triangle>& triangles){
    Vec3 origin = Vec3{0,0,0};



    // Pixels loop
    for (int i = 0; i < width * height; i++){
        float min_dist = 100000;
        unsigned char collision_triangle;
        bool collision = false;
        Vec3 ray = Camera_Vectors[i];


        // Triangle loop
        for (int j = 0; j < triangles.size(); j++){
            float t = 0;
            Triangle tri = triangles[j];

            Vec3 edge1 = tri.v2 - tri.v1;
            Vec3 edge2 = tri.v3 - tri.v1;
            Vec3 ray_cross_e2 = ray.cross(edge2);
            float det = dot(edge1, ray_cross_e2);

            if (fabsf(det) <= epsilon)
                continue;

            float inv_det = 1.0 / det;
            Vec3 s = origin - tri.v1;
            float u = inv_det * dot(s, ray_cross_e2);

            if (u < -epsilon || u > 1 + epsilon)
                continue;

            Vec3 s_cross_e1 = s.cross(edge1);
            float v = inv_det * dot(ray, s_cross_e1);

            if (v < -epsilon || (u + v) > 1.0f + epsilon)
                continue;

            t = inv_det * dot(edge2, s_cross_e1);
            if (t < min_dist){
                min_dist = t;
                collision_triangle = j;
                collision = true;
            }
        }
        if (collision){
            pixels[i] = {(unsigned char)((triangles[collision_triangle].n.x / 2 + 1) * 255),
                         (unsigned char)((triangles[collision_triangle].n.y / 2 +1) * 255),
                         (unsigned char)((triangles[collision_triangle].n.z / 2 + 1) * 255)};
        }
        else {pixels[i] = {0,0,0};}
    }
    

    return 1;
}


int main() {
    auto start = std::chrono::high_resolution_clock::now();
    Model test = Model("../sphere.obj");
    test.move_position(Vec3{0,0,3});
    int width = 512;
    int height = 512;
    RGB* pixels = new RGB[width*height];
    Vec3* Camera_Vectors = new Vec3[width*height];
    MakeCameraNormal(width, height, pixels, Camera_Vectors, 0.5, 16/5, 16/5);
    get_collions(width,height,pixels,Camera_Vectors, test.triangles);

    stbi_write_bmp("Test.bmp", width,height,3,pixels);
    std::cout << std::chrono::high_resolution_clock::now() - start;
    return 0;
 
}
