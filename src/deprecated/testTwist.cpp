/******************************************************************************
 * File:             test.cpp
 *
 * Author:           Ruoyang Xu
 * Created:          04/28/2021
 * Description:      Test files for Point Contact Factor
 *****************************************************************************/

#include <gtsam/base/Lie.h>
#include <string.h>
#include <utils.h>
#include <yaml-cpp/yaml.h>
#include <CLI/CLI.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>

// Factors
#include "BetweenContactFactor.h"
#include "ContactUtils.h"

// Utils
#include "dataloader.h"
#include "fast-cpp-csv-parser/csv.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace gtsam;

int main(int argc, char* argv[])
{
    utils::setLogPattern();
    CLI::App app{ "Test file for Multi Leg Contact Factor" };

    std::string configFilePath("");
    std::string datasetFilePath("");
    std::string imuConfigPath("");
    int maxIdx = 10;
    bool debug = false;
    double fl0, fl1, fl2;
    app.add_option("-a, --fl0", fl0, "Leg Configuration input");
    app.add_option("-s, --fl1", fl1, "Dataset input");
    app.add_option("-d, --fl2", fl2, "IMU Config File Path");
    app.add_option("-c, --config", configFilePath, "Leg Configuration input");
    CLI11_PARSE(app, argc, argv);

    // Load Leg Configs
    std::map<std::string, gtsam::LegConfig> legConfigs = DataLoader::loadLegConfig(configFilePath);
    Vector encoder     = (Vector3() << fl0, fl1, fl2).finished();
    Pose3 baseTcontact = Pose3(LegMeasurement::efInBase(encoder, legConfigs["fl"]));
    Pose3 baseTcontactTest = Pose3(LegMeasurement::efInBaseExpMap(encoder, legConfigs["fl"]));
    std::cout << baseTcontact << std::endl;
    std::cout << baseTcontactTest << std::endl;
}
