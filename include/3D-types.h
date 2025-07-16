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

struct Vectors3D{
    std::vector<float> x,y,z;
};

inline float dotSoA(Vectors3D& v, int i, int j){
     return v.x[i] * v.x[j] +
            v.y[i] * v.y[j] +
            v.z[i] * v.z[j];
}

struct TriangleSoA{
    std::vector<int> v1,v2,v3;};

struct ModelSoA{
    Vectors3D vectors;
    TriangleSoA tri_index;
    Vectors3D normals;

};
inline __attribute__((always_inline)) float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vectors3D calculate_tri_normals(Vectors3D& vectors, TriangleSoA& tri_index){
    Vectors3D normals;
    for (int i = 0; i < tri_index.v1.size(); i++){
            Vec3 edge1 = Vec3{vectors.x[tri_index.v2[i]] - vectors.x[tri_index.v1[i]],
                         vectors.y[tri_index.v2[i]] - vectors.y[tri_index.v1[i]],
                         vectors.z[tri_index.v2[i]] - vectors.z[tri_index.v1[i]]};
                         
            Vec3 edge2 = Vec3{vectors.x[tri_index.v3[i]] - vectors.x[tri_index.v1[i]],
                         vectors.y[tri_index.v3[i]] - vectors.y[tri_index.v1[i]],
                         vectors.z[tri_index.v3[i]] - vectors.z[tri_index.v1[i]]};
        Vec3 n = edge1.cross(edge2).normalize();
        normals.x.push_back(n.x);
        normals.y.push_back(n.y);
        normals.z.push_back(n.z);

    }

    return normals;
}

struct Triangle{
    Vec3 v1, v2, v3;
    Vec3 n;

    Triangle(const Vec3& a, const Vec3& b, const Vec3& c)
        : v1(a), v2(b), v3(c) {

        Vec3 edge1 = v2 - v1;
        Vec3 edge2 = v3 - v1;
        n = edge1.cross(edge2).normalize();
    }


};
