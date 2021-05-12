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
#include <fstream>
#include <map>
#include <gtsam/base/Lie.h>
#include <vector>
#include <string.h>

// Factors
// #include "ContactUtils.h"
#include "BetweenContactFactor.h"

// Utils
#include "fast-cpp-csv-parser/csv.h"
#include "dataloader.h"
// #include "imu.h"

#include <gtsam/inference/Symbol.h>
// #include <gtsam/navigation/CombinedImuFactor.h>
// #include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace gtsam;

using symbol_shorthand::B; // Bias
using symbol_shorthand::V; // Velocity
using symbol_shorthand::X; // Body Pose

void saveTrajectory(const Values &, uint64_t, std::string);

int main(int argc, char* argv[])
{
    utils::setLogPattern();
    CLI::App app{"Test file for Multi Leg Contact Factor"};

    std::string legConfigFilePath("");
    std::string datasetFilePath("");
    std::string imuConfigPath("");
    std::string outputFile("");
    std::string unOptimizedFile("");
    size_t maxIdx = 10;
    bool debug = false;
    app.add_option("-c, --config", legConfigFilePath, "Leg Configuration input");
    app.add_option("-d, --data", datasetFilePath, "Dataset input");
    app.add_option("-i, --IMU", imuConfigPath, "IMU Config File Path");
    app.add_option("-m, --maxIdx", maxIdx, "max index for number of data to be read");
    app.add_option("-b, --debug", debug, "debug options");
    app.add_option("-o, --output", outputFile, "trajectory Output filename");
    app.add_option("-u, --unopt", unOptimizedFile, "trajectory Output of unoptimized graph");
    CLI11_PARSE(app, argc, argv);

    legged::Dataloader dataloader = legged::Dataloader(imuConfigPath, legConfigFilePath, datasetFilePath);
    legged::LegConfigMap legConfigs = dataloader.getLegConfigs();
    auto imuParam = boost::make_shared<gtsam::PreintegrationCombinedParams>(dataloader.getImuParams());
    auto imuBias = dataloader.getImuBias();

    // Load Leg Configs
    // std::map<std::string, gtsam::LegConfig> legConfigs = DataLoader::loadLegConfig(configFilePath);

    // Setup IMU Configs
    // auto p = DataLoader::loadIMUConfig(imuConfigPath);
    // auto priorBias = DataLoader::getIMUBias(imuConfigPath);
    // imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    // This is for the Base
    std::shared_ptr<PreintegrationType> preintegrated =
        std::make_shared<PreintegratedCombinedMeasurements>(imuParam, imuBias);

    // Setup Prior Values
    Rot3 priorRotation(I_3x3); // Default Always identity
    Point3 priorPoint(0, 0, 0.194213); // This number is approximately rest height
    Pose3 priorPose(priorRotation, priorPoint);
    Eigen::Vector3d priorVelocity(0, 0, 0);
    Values initialValues;
    uint64_t stateCount = 0;
    NavState prevState(priorPose, priorVelocity);
    NavState propState = prevState;
    imuBias::ConstantBias prevBias = imuBias;

    initialValues.insert(X(stateCount), priorPose);
    initialValues.insert(V(stateCount), priorVelocity);
    initialValues.insert(B(stateCount), prevBias);

    // Setup FactorGraph
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    // if (false) {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        ISAM2* isam2 = new ISAM2(parameters);
    // }

    // Setup Prior Factors
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.001, 0.001, 0.001, 0.05, 0.05, 0.05)
          .finished());  // rad,rad,rad, m, m, m
    auto priorVelNoise = noiseModel::Isotropic::Sigma(3, 0.001);
    auto biasNoise = noiseModel::Isotropic::Sigma(6, 1e-3);

    graph->addPrior(X(stateCount), priorPose, priorPoseNoise);
    graph->addPrior(V(stateCount), priorVelocity, priorVelNoise);
    graph->addPrior(B(stateCount), prevBias, biasNoise);
    

    Values finalResult;

    int imu_cycle = 0;
    int max_imu = 200;
    int lp_cycle = 0;
    int max_lp = 10;
    int last_lp_index = 0;
    gtsam::Pose3 lastPose = priorPose;
    size_t index = 0;
    double timestamp;
    gtsam::Pose3 final_pose_reading;
    gtsam::Vector6 imu_reading;
    std::array<gtsam::Vector3, 4> leg_encoder_readings;
    std::array<int, 4> leg_contact_readings;

    double dt = 0.005;

    while (dataloader.readDatasetLine(timestamp, final_pose_reading, imu_reading, 
        leg_encoder_readings, leg_contact_readings) && index++ < maxIdx) {

        if (imu_cycle < max_imu) {
            imu_cycle++;
            preintegrated->integrateMeasurement(imu_reading.head<3>(), imu_reading.tail<3>(), 0.005);
        } else {
            imu_cycle = 0;
            lp_cycle++;
            stateCount++;
            preintegrated->integrateMeasurement(imu_reading.head<3>(), imu_reading.tail<3>(), 0.005);
            auto combinedMeasurement = dynamic_cast<const PreintegratedCombinedMeasurements&>(
                *preintegrated);
            CombinedImuFactor imuFactor(X(stateCount - 1), V(stateCount - 1), 
                X(stateCount), V(stateCount), B(stateCount - 1), B(stateCount),
                combinedMeasurement);
            graph->add(imuFactor);

            propState = preintegrated->predict(prevState, prevBias);
            // std::cout << "Prev State: " << prevState << std::endl;
            // if (index > 500) {
            //     std::cout << "prop State: " << propState << std::endl;
            // }
            // std::cout << "Rotation Difference: " << propState.pose().rotation().between(final_pose_reading.rotation()) << std::endl;
            // std::cout << "angV: " << imu_reading_pl.tail<3>() << std::endl;
            // std::cout << "Pose Reading: " << final_pose_reading << std::endl;
            // std::cout << "-----------------" << std::endl;
            // std::cout << "propState difference: " << prevState.pose().between(propState.pose()) << std::endl;
            // std::cout << "Absolute Difference: " << lastPose.between(final_pose_reading) << std::endl;
            // std::cout << "preintegrated Difference: " << preintegrated->deltaXij() << std::endl;
            // gtsam::Pose3 tempP(r, p);
            // initialValues.insert(X(stateCount), final_pose_reading);
            initialValues.insert(X(stateCount), propState.pose());
            initialValues.insert(V(stateCount), propState.v());
            initialValues.insert(B(stateCount), prevBias);

            if (lp_cycle > max_lp) {
                lp_cycle = 0;
                BetweenFactor<Pose3> lp_factor(X(last_lp_index), X(stateCount), lastPose.between(final_pose_reading),
                    noiseModel::Diagonal::Sigmas((Vector(6) << 0.001, 0.001, 0.001, 0.005, 0.005, 0.005).finished()));
                graph->add(lp_factor);
                lastPose = final_pose_reading;
                last_lp_index = stateCount;
            }



            LevenbergMarquardtParams params;
            // params.setVerbosityLM("SUMMARY");
            LevenbergMarquardtOptimizer optimizer(*graph, initialValues, params);
            Values result = optimizer.optimize();
            prevState = NavState(result.at<Pose3>(X(stateCount)), result.at<Vector3>(V(stateCount)));
            prevBias = result.at<imuBias::ConstantBias>(B(stateCount));
            prevState = propState;

            // Reset the preintegration object.
            preintegrated->resetIntegrationAndSetBias(prevBias);
            // std::cout << "Interpolated Bias" << prevBias << std::endl;
        }
    }

    LevenbergMarquardtOptimizer optimizer(*graph, initialValues);
    finalResult = optimizer.optimize();
    saveTrajectory(finalResult, stateCount, outputFile);
    std::cout << "Sucessfully Completed" << std::endl;

    if (unOptimizedFile.compare("") != 0) {
        std::cout << "Saving Unoptimized trajectory" << std::endl;
        saveTrajectory(initialValues, stateCount, unOptimizedFile);
    }
}

void saveTrajectory(const Values & v, uint64_t stateCount, std::string filename) {
    std::cout << "Saving Base Trajectory to " << filename << std::endl;
    std::ofstream f;
    f.open(filename.c_str());

    Pose3 base;
    f << "x,y,z" << std::endl;
    for (uint64_t i = 0; i < stateCount; i++) {
        base = v.at<Pose3>(X(i));
        auto t = base.translation();
        f << t(0) << "," << t(1) << "," << t(2) << std::endl;
    }
    f.close();
    std::cout << "Trajectory Saved!" << std::endl;

}
