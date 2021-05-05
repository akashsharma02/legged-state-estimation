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
#include "IMU.h"

#include <gtsam/inference/Symbol.h>
// #include <gtsam/navigation/CombinedImuFactor.h>
// #include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace gtsam;

// using symbol_shorthand::B; // Bias
// using symbol_shorthand::V; // Velocity
using symbol_shorthand::X; // Body Pose

void saveTrajectory(const Values &, uint64_t, std::string);

int main(int argc, char* argv[])
{
    utils::setLogPattern();
    CLI::App app{"Test file for Multi Leg Contact Factor"};

    std::string configFilePath("");
    std::string datasetFilePath("");
    std::string imuConfigPath("");
    std::string outputFile("");
    int maxIdx = 10;
    bool debug = false;
    app.add_option("-c, --config", configFilePath, "Leg Configuration input");
    app.add_option("-d, --data", datasetFilePath, "Dataset input");
    app.add_option("-i, --IMU", imuConfigPath, "IMU Config File Path");
    app.add_option("-m, --maxIdx", maxIdx, "max index for number of data to be read");
    app.add_option("-b, --debug", debug, "debug options");
    app.add_option("-o, --output", outputFile, "trajectory Output filename");
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

    // Load Leg Configs
    std::map<std::string, gtsam::LegConfig> legConfigs = DataLoader::loadLegConfig(configFilePath);

    // Setup IMU Configs
    auto p = DataLoader::loadIMUConfig(imuConfigPath);
    auto priorBias = DataLoader::getIMUBias(imuConfigPath);
    // imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    // This is for the Base
    std::shared_ptr<PreintegrationType> preintegrated = 
        std::make_shared<PreintegratedCombinedMeasurements>(p, priorBias);

    // Setup Prior Values
    Rot3 priorRotation(I_3x3); // Default Always identity
    Point3 priorPoint(0, 0, 0.194213); // This number is approximately rest height
    Pose3 priorPose(priorRotation, priorPoint);
    Eigen::Vector3d priorVelocity(0, 0, 0);
    Values initialValues;
    uint64_t stateCount = 0;
    NavState prevState(priorPose, priorVelocity);
    NavState propState = prevState;
    // imuBias::ConstantBias prevBias = priorBias;
    
    initialValues.insert(X(stateCount), priorPose);

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

    Values result;
    if (debug) {
        graph->print("\nFactor Graph:\n");
        initialValues.print("\nInitial Estimate:\n"); 
        LevenbergMarquardtOptimizer optimizer(*graph, initialValues);
        result = optimizer.optimize();
        result.print("Final Result:\n");
    }

    double x, y, z, i, j, k, w;
    Pose3 lastPose = priorPose;

    int imu_cycle = 0;
    int max_imu = 200;
    ORB_SLAM2::PreintegratedIMU new_meas;
    new_meas.initialize();
    cv::Mat g = cv::Mat::zeros(3, 1, CV_64F);
    new_meas.setGravity(g);

    cv::Point3d acc, angVel;
    Matrix R;
    Point3 P;
    Matrix v;
    while (datafile.read_row(
        ts,
        x, y, z, i, j, k, w,
        wx,wy,wz,ax,ay,az,fl0,fl1,fl2,fr0,fr1,fr2,bl0,bl1,bl2,br0,br1,br2,flc,frc,blc,brc
        ) && idx++ < maxIdx) {

        if (imu_cycle < max_imu) {
            imu_cycle++;
            // imu *= 200;
            ORB_SLAM2::ImuMeasure meas(ax, ay, az, wx, wy, wz, ts);
            acc = meas._a;
            angVel = meas._w;
            new_meas.integrateMeasurement(acc, angVel, 1./200);
        } else {
            imu_cycle = 0;
            stateCount++;
            ORB_SLAM2::ImuMeasure meas(ax, ay, az, wx, wy, wz, ts);
            acc = meas._a;
            angVel = meas._w;
            new_meas.integrateMeasurement(acc, angVel, 1./200);
            R = toMatrix3d(new_meas.getRotation());
            P = toPoint3(new_meas.getTranslation());
            Pose3 wTb = Pose3(Rot3(R), P);
            BetweenFactor<Pose3> btf(X(stateCount - 1), X(stateCount), lastPose.between(wTb), priorPoseNoise);
            initialValues.insert(X(stateCount), wTb);
            lastPose = wTb;
            graph->add(btf);
        }
    }


    // Add a small final loop closure
    Eigen::Quaternion finRot3(w, i, j, k);
    Rot3 r(finRot3);
    Point3 t(x, y, z);
    Pose3 finalPose = Pose3(r, t);
    // BetweenFactor<Pose3> lp(X(0), X(stateCount), priorPose.between(finalPose), priorPoseNoise);
    // graph->add(lp);

    LevenbergMarquardtOptimizer optimizer(*graph, initialValues);
    result = optimizer.optimize();
    saveTrajectory(initialValues, stateCount, outputFile);

    std::cout << "Sucessfully Completed" << std::endl;
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
