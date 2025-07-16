#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <filesystem>
#include "object-importer.h"

Model::Model(const std::string& filepath){
            
            std::filesystem::path path(filepath);
            std::string extension = path.extension().string(); // includes the dot, e.g., ".txt"
            if (extension == ".obj"){
                triangles = import_obj(filepath);
            }

        }
        




        std::vector<Triangle> Model::import_obj(const std::string& filepath){
            std::ifstream file(filepath);
            std::string line;
            std::vector<Vec3> points;
            std::vector<Triangle> tris;
             if (!file.is_open()) {
                    std::cerr << "Unable to open file!" << std::endl;
                    return tris;
                }
             
            std::cout << "file is open" << "\n";

            int i = 0;
            while (std::getline(file, line)){
                if (line.empty()) continue;
                if (line[0] == 'v' && line[1] == ' '){
                    std::istringstream iss(line);
                    char prefix;
                    float x,y,z,w;
                    iss >> prefix >> x >> y >> z >> w;
                    points.push_back(Vec3{x,y,z});
                } else if (line[0] == 'f') {
                    std::istringstream iss(line);
                    char prefix;
                    iss >> prefix;  // consume 'f'
                    std::string vertexStr;
                    std::vector<int> vertexIndices;

                        while (iss >> vertexStr) {
                            size_t slashPos = vertexStr.find('/');
                            if (slashPos != std::string::npos) {
                                std::string vertexIndexStr = vertexStr.substr(0, slashPos);
                                int vertexIndex = std::stoi(vertexIndexStr);
                                vertexIndices.push_back(vertexIndex);
                            }
                        }

                    if (vertexIndices.size() >= 3) {
                        // Create triangles from the first 3 vertex indices (assuming triangles)
                        int x = vertexIndices[0];
                        int y = vertexIndices[1];
                        int z = vertexIndices[2];

                        if (x > 0 && y > 0 && z > 0 &&
                            x <= static_cast<int>(points.size()) &&
                            y <= static_cast<int>(points.size()) &&
                            z <= static_cast<int>(points.size())) {

                            tris.push_back(Triangle{
                                points[x - 1],
                                points[y - 1],
                                points[z - 1]
                            });
                        } else {
                            std::cerr << "Face indices out of range in line: " << line << std::endl;
                        }
                    } else {
                        std::cerr << "Invalid face line (less than 3 vertices): " << line << std::endl;
                    }
                }
                i +=1;
            }


            return tris;

        }
int Model::move_position(Vec3 offset){
    for (int i = 0; i < triangles.size(); i++){
        triangles[i].v1 += offset;
        triangles[i].v2 += offset;
        triangles[i].v3 += offset;
    }
    return 0;
}
