#pragma once
#include <stdlib.h>
#include <cmath>
#include <vector>

struct RGB {
    unsigned char r, g, b;};

struct Vec3{
    float x, y, z;

    Vec3 operator-(const Vec3& other) const {
            return Vec3{
                x - other.x,
                y - other.y,
                z - other.z
            };
    };
    Vec3 operator+=(const Vec3& other){
                x += other.x;
                y += other.y;
                z += other.z;
                return *this;
    };

    Vec3 cross(const Vec3& v) const {
            return Vec3{
                y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x
            };
        }
    Vec3 normalize() const {
            float length = std::sqrt(x*x + y*y + z*z);
            if (length == 0) return Vec3{0, 0, 0}; // Avoid division by zero
            return Vec3{x / length, y / length, z / length};
        }
    float dot(const Vec3& v) const {
        return float(x * v.x + y * v.y + z * v.z);
    }

};

inline __attribute__((always_inline)) float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

struct Triangle{
    Vec3 v1;
    Vec3 n;
    Vec3 edge1, edge2;

    Triangle(const Vec3& a, const Vec3& b, const Vec3& c)
        : v1(a){

        edge1 = b - v1;
        edge2 = c - v1;
        n = edge1.cross(edge2).normalize();
    }


};
