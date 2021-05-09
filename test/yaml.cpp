#include <yaml-cpp/yaml.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "string.h"

int main(int argc, char** argv)
{
    if(argc != 2)
	{
		std::cerr << std::endl << "Usage: ./yaml_tester path_to_config" << std::endl;
		return 1;
	}

    YAML::Node node = YAML::LoadFile(std::string(argv[1]));
    const std::vector<double> r1 = node["r1"].as< std::vector<double> >();

    Eigen::Matrix<double,1,3,Eigen::RowMajor> Tvs(r1.data());
    std::cout << Tvs << std::endl;
    return 0;
}
