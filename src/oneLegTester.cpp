/******************************************************************************
* File:             test.cpp
*
* Author:           Ruoyang Xu
* Created:          04/28/2021
* Description:      Test files for Point Contact Factor
*****************************************************************************/

#include <utils.h>
#include <CLI/CLI.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <gtsam/base/Lie.h>
#include <vector>
#include <string.h>

// Factors
#include "ContactUtils.h"
#include "BetweenContactFactor.h"

// Utils 
#include "fast-cpp-csv-parser/csv.h"
#include "dataloader.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace gtsam;

using symbol_shorthand::B; // Bias
using symbol_shorthand::V; // Velocity
using symbol_shorthand::X; // Body Pose
// Welp, Symbol only takes char
// Gonna use Keyboard location
using symbol_shorthand::Q; // Fl Foot Pose
using symbol_shorthand::P; // FR Foot Pose
using symbol_shorthand::A; // BL Foot Pose
using symbol_shorthand::L; // BR Foot Pose

int main(int argc, char* argv[])
{
    utils::setLogPattern();
    CLI::App app{"Test file for Multi Leg Contact Factor"};

    std::string configFilePath("");
    std::string datasetFilePath("");
    std::string imuConfigPath("");
    int maxIdx = 10;
    app.add_option("-c, --config", configFilePath, "Leg Configuration input");
    app.add_option("-d, --data", datasetFilePath, "Dataset input");
    app.add_option("-i, --IMU", imuConfigPath, "IMU Config File Path");
    app.add_option("-m, --maxIdx", maxIdx, "max index for number of data to be read");
    CLI11_PARSE(app, argc, argv);

    // Setup Dataset
    io::CSVReader<23> datafile(datasetFilePath);
    datafile.read_header(io::ignore_extra_column,
        "ts","wx","wy","wz","ax","ay","az","fl0","fl1","fl2",
        "fr0","fr1","fr2","bl0","bl1","bl2","br0","br1","br2",
        "flc","frc","blc","brc"
    );
    double ts,wx,wy,wz,ax,ay,az,fl0,fl1,fl2,fr0,fr1,fr2,bl0,bl1,bl2,br0,br1,br2;
    int flc,frc,blc,brc;
    int idx = 0;
    bool robotReady = false;

    // Load Leg Configs
    std::map<std::string, gtsam::LegConfig> legConfigs = DataLoader::loadLegConfig(configFilePath);

    // Setup IMU Configs
    auto p = DataLoader::loadIMUConfig(imuConfigPath);
    auto priorBias = DataLoader::getIMUBias(imuConfigPath);
    std::shared_ptr<PreintegrationType> preintegrated = 
        std::make_shared<PreintegratedImuMeasurements>(p, priorBias);
    
    // Setup Prior Values
    Rot3 priorRotation(I_3x3); // Default Always identity
    Point3 priorPoint(0, 0, 0.194213); // This number is approximately rest height
    Pose3 priorPose(priorRotation, priorPoint);
    Eigen::Vector3d priorVelocity(0, 0, 0);
    Values initialValues;
    int stateCount = 0;

    // Those are the approximate rest location at spawn
    Rot3 footRotation(0, -0.303, 0, 0.953);
    Point3 flPoint(0.096, 0.102, 0);
    Pose3 priorfl(footRotation, flPoint);
    Point3 frPoint(0.096, -0.102, 0);
    Pose3 priorfr(footRotation, frPoint);
    Point3 blPoint(-0.135, 0.102, 0);
    Pose3 priorbl(footRotation, flPoint);
    Point3 brPoint(-0.135, -0.102, 0);
    Pose3 priorbr(footRotation, flPoint);
    
    initialValues.insert(X(stateCount), priorPose);
    initialValues.insert(V(stateCount), priorVelocity);
    initialValues.insert(B(stateCount), priorBias);
    initialValues.insert(Q(stateCount), priorfl);
    initialValues.insert(P(stateCount), priorfr);
    initialValues.insert(A(stateCount), priorbl);
    initialValues.insert(L(stateCount), priorbr);

    // Setup FactorGraph
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    double dt = 1. / 200;

    // Setup Prior Factors
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.001, 0.001, 0.001, 0.05, 0.05, 0.05)
          .finished());  // rad,rad,rad, m, m, m
    auto priorVelNoise = noiseModel::Isotropic::Sigma(3, 0.001);
    auto biasNoise = noiseModel::Isotropic::Sigma(6, 1e-3);

    graph->addPrior(X(stateCount), priorPose, priorPoseNoise);
    graph->addPrior(V(stateCount), priorVelocity, priorVelNoise);
    graph->addPrior(B(stateCount), priorBias, biasNoise);
    graph->addPrior(Q(stateCount), priorfl, priorPoseNoise);
    graph->addPrior(P(stateCount), priorfr, priorPoseNoise);
    graph->addPrior(A(stateCount), priorbl, priorPoseNoise);
    graph->addPrior(L(stateCount), priorbr, priorPoseNoise);

    while (datafile.read_row(
        ts,wx,wy,wz,ax,ay,az,fl0,fl1,fl2,fr0,fr1,fr2,bl0,bl1,bl2,br0,br1,br2,flc,frc,blc,brc
        ) && idx++ < maxIdx) {

        // Wait until All Four legs are on the ground
        // since they were initially floating
        if (flc == 1 && frc == 1 && blc == 1 && brc == 1 && !robotReady) {
            // Mark ready
            robotReady = true;
            
            // Create Prior

            continue; // Skip so that factors are not added repeatedly
        } else if (!robotReady) {
            // If not ready and also not on the ground, just skip the rest
            continue;
        }

        // if ready
            // If contact sensor no changes
                // accumulate IMU Measurment
            // if contact sensor changes
                // Create New Node in the Factor Graph
                    // IMU Factor
                    // Point Contact Factor
                    // Forward Kinematic Factor
    }

    std::cout << "Sucessfully Completed" << std::endl;
}
