#include <Eigen/Core>
#include <fstream>
#include <igl/ray_mesh_intersect.h>
#include <igl/readSTL.h>
#include <iostream>
#include <string>

#include "defines.hpp"
#include "igl/Hit.h"

int main() {
    std::string filename = PRJ_ROOT_DIR + std::string("/mattonella.stl");
    std::cout << "Reading file: " << filename << std::endl;
    std::ifstream file;
    file.open(filename);
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd N;
    if (!igl::readSTL(file, V, F, N)) {
        std::cout << "Failed parsing STL file" << std::endl;
        return 1;
    }
    file.close();

    Eigen::RowVector3d origin(0.0, 0.0, 0.0);
    Eigen::RowVector3d direction(0.0, 0.0, 1.0);

    std::vector<igl::Hit> hits;
    igl::ray_mesh_intersect(origin, direction, V, F, hits);

    // Check if there is an intersection
    if (hits.size() > 0) {
        // Print the coordinates of the first intersection point
        std::cout << "First Intersection point: " << hits.front().v << std::endl;
    } else {
        std::cout << "No intersection." << std::endl;
    }

    return 0;
}
