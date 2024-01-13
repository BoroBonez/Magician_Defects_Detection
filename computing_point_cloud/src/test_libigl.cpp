#include <igl/readSTL.h>
#include <igl/intersect_ray_mesh.h>

#include <Eigen/Core>
#include <iostream>
#include <string>
#include <fstream>

#include "defines.hpp"

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

    Eigen::VectorXi I;
    Eigen::MatrixXd C;
    igl::intersect_ray_mesh(origin, direction, V, F, I, C);

    // Check if there is an intersection
    if (I.size() > 0) {
        // Print the coordinates of the first intersection point
        std::cout << "First Intersection point: " << C.row(0) << std::endl;
    } else {
        std::cout << "No intersection." << std::endl;
    }

    return 0;
}
