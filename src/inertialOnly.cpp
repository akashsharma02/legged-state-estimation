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

void saveTrajectory(const Values &, uint64_t);

int main(int argc, char* argv[])
{
    utils::setLogPattern();
    CLI::App app{"Test file for Multi Leg Contact Factor"};

    std::string configFilePath("");
    std::string datasetFilePath("");
    std::string imuConfigPath("");
    int maxIdx = 10;
    bool debug = false;
    app.add_option("-c, --config", configFilePath, "Leg Configuration input");
    app.add_option("-d, --data", datasetFilePath, "Dataset input");
    app.add_option("-i, --IMU", imuConfigPath, "IMU Config File Path");
    app.add_option("-m, --maxIdx", maxIdx, "max index for number of data to be read");
    app.add_option("-b, --debug", debug, "debug options");
    CLI11_PARSE(app, argc, argv);

    // Setup Dataset
    io::CSVReader<23 + 7> datafile(datasetFilePath);
    datafile.read_header(io::ignore_extra_column,
        "ts",
        "x","y","z","i","j","k","w",
        "wx","wy","wz","ax","ay","az","fl0","fl1","fl2",
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
    // This is for the Base
    std::shared_ptr<PreintegrationType> preintegrated = 
        std::make_shared<PreintegratedImuMeasurements>(p, priorBias);

    // Logging for the contact Frames, they are not factors, just preintegrated measurements
    auto pQ = DataLoader::loadIMUConfig(imuConfigPath);
    auto pP = DataLoader::loadIMUConfig(imuConfigPath);
    auto pA = DataLoader::loadIMUConfig(imuConfigPath);
    auto pL = DataLoader::loadIMUConfig(imuConfigPath);
    std::shared_ptr<PreintegrationType> imuLogQ = 
        std::make_shared<PreintegratedImuMeasurements>(pQ, priorBias);
    std::shared_ptr<PreintegrationType> imuLogP = 
        std::make_shared<PreintegratedImuMeasurements>(pP, priorBias);
    std::shared_ptr<PreintegrationType> imuLogA = 
        std::make_shared<PreintegratedImuMeasurements>(pA, priorBias);
    std::shared_ptr<PreintegrationType> imuLogL = 
        std::make_shared<PreintegratedImuMeasurements>(pL, priorBias);

    // Setup Prior Values
    Rot3 priorRotation(I_3x3); // Default Always identity
    Point3 priorPoint(0, 0, 0.194213); // This number is approximately rest height
    Pose3 priorPose(priorRotation, priorPoint);
    Eigen::Vector3d priorVelocity(0, 0, 0);
    Values initialValues;
    uint64_t stateCount = 0;
    NavState prevState(priorPose, priorVelocity);
    NavState propState = prevState;
    imuBias::ConstantBias prevBias = priorBias;

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
    // initialValues.insert(Q(stateCount), priorfl);
    // initialValues.insert(P(stateCount), priorfr);
    // initialValues.insert(A(stateCount), priorbl);
    // initialValues.insert(L(stateCount), priorbr);

    // Setup FactorGraph
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    double dt = 1. / 200;
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
    graph->addPrior(B(stateCount), priorBias, biasNoise);
    // graph->addPrior(Q(stateCount), priorfl, priorPoseNoise);
    // graph->addPrior(P(stateCount), priorfr, priorPoseNoise);
    // graph->addPrior(A(stateCount), priorbl, priorPoseNoise);
    // graph->addPrior(L(stateCount), priorbr, priorPoseNoise);

    // Contact State Tracking
    // Leg {(f)ront, (l)eft}, (C)ontact, (L)ast
    int flcl, frcl, blcl, brcl;
    LegMeasurement* fl;
    LegMeasurement* fr;
    LegMeasurement* bl;
    LegMeasurement* br;
    ContactStates flState;

    Values result;
    if (debug) {
        graph->print("\nFactor Graph:\n");
        initialValues.print("\nInitial Estimate:\n"); 
        LevenbergMarquardtOptimizer optimizer(*graph, initialValues);
        result = optimizer.optimize();
        result.print("Final Result:\n");
    }

    double x, y, z, i, j, k, w;

    int imu_cycle = 0;
    int max_imu = 200;

    while (datafile.read_row(
        ts,
        x, y, z, i, j, k, w,
        wx,wy,wz,ax,ay,az,fl0,fl1,fl2,fr0,fr1,fr2,bl0,bl1,bl2,br0,br1,br2,flc,frc,blc,brc
        ) && idx++ < maxIdx) {

        if (imu_cycle < max_imu) {
            imu_cycle++;
            Vector6 imu = (Vector6() << ax, ay, az, wx, wy, wz).finished();
            imu *= 200;
            preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), 1. / 200);
        } else {
            imu_cycle = 0;
            stateCount++;
            auto preintIMU = dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated);
            ImuFactor imuFactor(
                X(stateCount - 1), V(stateCount - 1),
                X(stateCount), V(stateCount),
                B(stateCount - 1), preintIMU
            );

            graph->add(imuFactor);
            auto bias = priorBias;
            imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
            graph->add(BetweenFactor<imuBias::ConstantBias>(
                B(stateCount - 1), B(stateCount), zero_bias, biasNoise
            ));

            propState = preintegrated->predict(prevState, prevBias);
            initialValues.insert(X(stateCount), propState.pose());
            initialValues.insert(V(stateCount), propState.v());
            initialValues.insert(B(stateCount), prevBias);

            isam2->update(*graph, initialValues);
            isam2->update();
            result = isam2->calculateEstimate();
            graph->resize(0);
            initialValues.clear();
            prevState = NavState(result.at<Pose3>(X(stateCount)),
                result.at<Vector3>(V(stateCount)));
            prevBias = result.at<imuBias::ConstantBias>(B(stateCount));
            preintegrated->resetIntegrationAndSetBias(prevBias);
        }
    }

    Eigen::Quaternion finRot3(w, i, j, k);
    Rot3 r(finRot3);
    Point3 t(x, y, z);
    Pose3 g = Pose3(r, t);
    BetweenFactor<Pose3> lp(X(0), X(stateCount), priorPose.between(g), priorPoseNoise);
    graph->add(lp);

    // LevenbergMarquardtOptimizer optimizer(*graph, initialValues);
    // result = optimizer.optimize();
    // prevState = NavState(result.at<Pose3>(X(stateCount)),
    //     result.at<Vector3>(V(stateCount)));
    // prevBias = result.at<imuBias::ConstantBias>(B(stateCount));
    // preintegrated->resetIntegrationAndSetBias(prevBias);
            

    isam2->update(*graph, initialValues);
    isam2->update();
    result = isam2->calculateEstimate();
    graph->resize(0);
    initialValues.clear();
    prevState = NavState(result.at<Pose3>(X(stateCount)),
        result.at<Vector3>(V(stateCount)));
    prevBias = result.at<imuBias::ConstantBias>(B(stateCount));
    preintegrated->resetIntegrationAndSetBias(prevBias);

    saveTrajectory(result, stateCount);
    

    std::cout << "Sucessfully Completed" << std::endl;
}

void saveTrajectory(const Values & v, uint64_t stateCount) {
    std::string filename = "inertialOnlyTrajectory.txt";
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
