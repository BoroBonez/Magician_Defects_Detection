#include <igl/readSTL.h>
#include <igl/unproject.h>

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
    if(!igl::readSTL(file, V, F, N)) {
        std::cout << "Failed parsing STL file" << std::endl;
    }

    file.close();


    // Eigen::Vector3d origin(0.0, 0.0, 0.0);
    // Eigen::Vector3d direction(0.0, 0.0, 1.0);
    // Eigen::RowVector3d intersection;
    // igl::unproject(origin, direction, V, F, intersection);



    return 0;
}
