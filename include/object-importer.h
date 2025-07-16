#pragma once
#include <string>
#include <vector>
#include "3D-types.h"

class Model{
public:
    Model(const std::string& filepath);
    std::vector<Triangle> triangles;
    int move_position(Vec3 offset);
private:
    std::vector<Triangle> import_obj(const std::string& filepath);


};
