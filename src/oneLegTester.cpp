/******************************************************************************
* File:             test.cpp
*
* Author:           Ruoyang Xu
* Created:          04/28/2021
* Description:      Test file for gtsam compilation
*****************************************************************************/

#include <utils.h>
#include <CLI/CLI.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <gtsam/base/Lie.h>

#include "ContactUtils.h"
#include "BetweenContactFactor.h"
#include "fast-cpp-csv-parser/csv.h"


Eigen::MatrixXd extractMatrix(const YAML::Node& node, std::string attribute) {
    const std::vector<double> vec = node[attribute].as< std::vector<double> >();
    Eigen::Matrix<double, 1, 7, Eigen::RowMajor> mat(vec.data());
    return Eigen::MatrixXd(mat);
}

Eigen::MatrixXd Data2PoseMat(Eigen::MatrixXd vec) {
    gtsam::Quaternion q(vec(6), vec(3), vec(4), vec(5));
    gtsam::Rot3 r(q);
    gtsam::Point3 t(vec(0), vec(1), vec(2));
    gtsam::Pose3 p(r, t);
    return p.matrix();
}

gtsam::LegConfig loadConfig(std::string configPath) {
    // Load Yaml
    YAML::Node node = YAML::LoadFile(configPath);
    gtsam::LegConfig rightBackLeg;
    rightBackLeg.firstJointInBase = Data2PoseMat(extractMatrix(node, "bTbrh"));
    rightBackLeg.jTul = Data2PoseMat(extractMatrix(node, "brhTbrul"));
    rightBackLeg.ulTll = Data2PoseMat(extractMatrix(node, "brulTbrll"));
    rightBackLeg.llTf = Data2PoseMat(extractMatrix(node, "brllTbrf"));
    rightBackLeg.covSlip = Eigen::MatrixXd::Identity(3, 3);

    // std::cout << rightBackLeg.firstJointInBase << std::endl;
    // std::cout << rightBackLeg.jTul << std::endl;
    // std::cout << rightBackLeg.ulTll << std::endl;
    // std::cout << rightBackLeg.llTf << std::endl;
    // std::cout << rightBackLeg.covSlip << std::endl;
    return rightBackLeg;
}

int main(int argc, char* argv[])
{
    utils::setLogPattern();
    CLI::App app{"Test file for Single Leg Contact Factor"};

    std::string configFilePath("");
    std::string datasetFilePath("");
    int maxIdx = 10;
    app.add_option("-c, --config", configFilePath, "Configuration input");
    app.add_option("-d, --data", datasetFilePath, "Dataset input");
    app.add_option("-m, --maxIdx", maxIdx, "max index for number of data to be read");
    CLI11_PARSE(app, argc, argv);

    auto legConfig = loadConfig(configFilePath);
    gtsam::LegMeasurement leg;
    leg.leg = legConfig;
    
    // Read CSV
    // 2 (idx, ts) + 3 (acc) + 3 (angV) + 4 * 3 (leg Encoder) + 4 (Leg Contact)
    io::CSVReader<10> datafile(datasetFilePath);
    datafile.read_header(io::ignore_extra_column,
        "x","y","z","i","j","k","w","br0","br1","br2");
    double x,y,z,i,j,k,w;
    double br0,br1,br2;

    int idx = 0;
    while (datafile.read_row(x,y,z,i,j,k,w,br0,br1,br2) && idx++ < maxIdx) {
        gtsam::Vector3 p(br0, br1, br2);
        std::cout << leg.efInBase(p) << std::endl;
        std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
    }
}
