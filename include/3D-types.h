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
struct Vec3SoA {
    std::vector<float> x, y, z;

    Vec3SoA(size_t count) {
        x.resize(count);
        y.resize(count);
        z.resize(count);
    }

    void set(size_t index, const Vec3& v) {
        x[index] = v.x;
        y[index] = v.y;
        z[index] = v.z;
    }

    Vec3 get(size_t index) const {
        return Vec3{x[index], y[index], z[index]};
    }
};

struct TriangleSoA {
    Vec3SoA v1;
    Vec3SoA edge1;
    Vec3SoA edge2;
    Vec3SoA n;

    TriangleSoA(size_t triangleCount)
        : v1(triangleCount), edge1(triangleCount),
          edge2(triangleCount), n(triangleCount) {}

    void addTriangle(size_t index, const Vec3& a, const Vec3& b, const Vec3& c) {
        Vec3 e1 = b - a;
        Vec3 e2 = c - a;
        Vec3 normal = e1.cross(e2).normalize();

        v1.set(index, a);
        edge1.set(index, e1);
        edge2.set(index, e2);
        n.set(index, normal);
    }
    void addEdgeTriangle(size_t index, const Vec3& _v1, const Vec3& _edge1, const Vec3& _edge2, const Vec3& _normal){
        v1.set(index, _v1);
        edge1.set(index, _edge1);
        edge2.set(index, _edge1);
        n.set(index,_normal);
    }
};

inline float dot_at(const Vec3SoA& a, const Vec3SoA& b, size_t i) {
    return a.x[i] * b.x[i] + a.y[i] * b.y[i] + a.z[i] * b.z[i];
}
inline Vec3 cross_at(const Vec3SoA& a, size_t index1, const Vec3SoA& b, size_t index2) {
    return {
        a.y[index1] * b.z[index2] - a.z[index1] * b.y[index2],
        a.z[index1] * b.x[index2] - a.x[index1] * b.z[index2],
        a.x[index1] * b.y[index2] - a.y[index1] * b.x[index2]
    };
}

inline TriangleSoA TriToSoA(std::vector<Triangle> tris){
    TriangleSoA new_trianglesSoA(tris.size());
    for(int i = 0; i < tris.size(); i++)
    {
        new_trianglesSoA.addEdgeTriangle(i, tris[i].v1, tris[i].edge1, tris[i].edge2, tris[i].n);
    }
    return new_trianglesSoA;
}
