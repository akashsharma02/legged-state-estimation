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

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include "dataloader.h"

// using namespace gtsam;

using gtsam::symbol_shorthand::B; // Bias
using gtsam::symbol_shorthand::V; // Velocity
using gtsam::symbol_shorthand::X; // Body Pose

void saveTrajectory(const gtsam::Values &, uint64_t, std::string);

int main(int argc, char* argv[])
{
    utils::setLogPattern();
    CLI::App app{"Test file for Multi Leg Contact Factor"};

    std::string legConfigFilePath("");
    std::string datasetFilePath("");
    std::string imuConfigPath("");
    std::string outputFile("");
    size_t maxIdx = 10;
    bool debug = false;
    app.add_option("-c, --config", legConfigFilePath, "Leg Configuration input");
    app.add_option("-d, --data", datasetFilePath, "Dataset input");
    app.add_option("-i, --IMU", imuConfigPath, "IMU Config File Path");
    app.add_option("-m, --maxIdx", maxIdx, "max index for number of data to be read");
    app.add_option("-b, --debug", debug, "debug options");
    app.add_option("-o, --output", outputFile, "trajectory Output filename");
    // app.add_option("-q, --accex", a1, "trajectory Output filename");
    // app.add_option("-w, --accey", a2, "trajectory Output filename");
    // app.add_option("-e, --accez", a3, "trajectory Output filename");
    
    CLI11_PARSE(app, argc, argv);

    legged::Dataloader dataloader = legged::Dataloader(imuConfigPath, legConfigFilePath, datasetFilePath);
    legged::LegConfigMap legConfigs = dataloader.getLegConfigs();
    auto imuParam = boost::make_shared<gtsam::PreintegrationCombinedParams>(dataloader.getImuParams());
    auto imuBias = dataloader.getImuBias();
    // Setup Dataset
    // io::CSVReader<23 + 7> datafile(datasetFilePath);
    // datafile.read_header(io::ignore_extra_column,
    //     "ts",
    //     "x","y","z","i","j","k","w",
    //     "wx","wy","wz","ax","ay","az","fl0","fl1","fl2",
    //     "fr0","fr1","fr2","bl0","bl1","bl2","br0","br1","br2",
    //     "flc","frc","blc","brc"
    // );
    // double ts,wx,wy,wz,ax,ay,az,fl0,fl1,fl2,fr0,fr1,fr2,bl0,bl1,bl2,br0,br1,br2;
    // int flc,frc,blc,brc;
    // int idx = 0;

    // Load Leg Configs
    // std::map<std::string, gtsam::LegConfig> legConfigs = DataLoader::loadLegConfig(configFilePath);

    // Setup IMU Configs
    // auto p = DataLoader::loadIMUConfig(imuConfigPath);
    // auto priorBias = DataLoader::getIMUBias(imuConfigPath);
    // imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    // This is for the Base
    std::shared_ptr<gtsam::PreintegrationType> preintegrated =
        std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParam, imuBias);

    // Setup Prior Values
    gtsam::Rot3 priorRotation(gtsam::I_3x3); // Default Always identity
    gtsam::Point3 priorPoint(0, 0, 0.194213); // This number is approximately rest height
    gtsam::Pose3 priorPose(priorRotation, priorPoint);
    Eigen::Vector3d priorVelocity(0, 0, 0);
    gtsam::Values initialValues;
    uint64_t stateCount = 0;
    gtsam::NavState prevState(priorPose, priorVelocity);
    gtsam::NavState propState = prevState;
    // imuBias::ConstantBias prevBias = priorBias;

    initialValues.insert(X(stateCount), priorPose);
    initialValues.insert(V(stateCount), priorVelocity);
    initialValues.insert(B(stateCount), imuBias);

    // Setup FactorGraph
    gtsam::NonlinearFactorGraph* graph = new gtsam::NonlinearFactorGraph();
    // if (false) {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        gtsam::ISAM2* isam2 = new gtsam::ISAM2(parameters);
    // }

    // Setup Prior Factors
    auto priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.001, 0.001, 0.001, 0.05, 0.05, 0.05)
          .finished());  // rad,rad,rad, m, m, m
    auto priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.001);
    auto biasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

    graph->addPrior(X(stateCount), priorPose, priorPoseNoise);

    gtsam::Values result;
    if (debug) {
        graph->print("\nFactor Graph:\n");
        initialValues.print("\nInitial Estimate:\n");
        gtsam::LevenbergMarquardtOptimizer optimizer(*graph, initialValues);
        result = optimizer.optimize();
        result.print("Final Result:\n");
    }

    // double x, y, z, i, j, k, w;
    // Pose3 lastPose = priorPose;

    // int imu_cycle = 0;
    // int max_imu = 200;
    // ORB_SLAM2::PreintegratedIMU new_meas;
    // new_meas.initialize();
    // cv::Mat g = cv::Mat::zeros(3, 1, CV_64F);
    // new_meas.setGravity(g);

    // cv::Point3d acc, angVel;
    // Matrix R;
    // Point3 P;
    // Matrix v;

    size_t index = 0;
    double timestamp;
    gtsam::Pose3 final_pose_reading;
    gtsam::Vector6 imu_reading;
    std::array<gtsam::Vector3, 4> leg_encoder_readings;
    std::array<int, 4> leg_contact_readings;

    while (dataloader.readDatasetLine(timestamp, final_pose_reading, imu_reading, 
        leg_encoder_readings, leg_contact_readings) && index++ < maxIdx) {

        // if (imu_cycle < max_imu) {
        //     imu_cycle++;
        //     // imu *= 200;
        //     ORB_SLAM2::ImuMeasure meas(ax, ay, az, wx, wy, wz, ts);
        //     acc = meas._a;
        //     angVel = meas._w;
        //     new_meas.integrateMeasurement(acc, angVel, 1./200);
        //     Vector6 imu = (Vector6() << ax, ay, az, wx, wy, wz).finished();
        //     preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), 1./200);
        // } else {
        //     imu_cycle = 0;
        //     stateCount++;
        //     ORB_SLAM2::ImuMeasure meas(ax, ay, az, wx, wy, wz, ts);
        //     acc = meas._a;
        //     angVel = meas._w;
        //     new_meas.integrateMeasurement(acc, angVel, 1./200);
        //     R = toMatrix3d(new_meas.getRotation());
        //     P = toPoint3(new_meas.getTranslation());
        //     Pose3 wTb = Pose3(Rot3(R), P);
        //     Vector6 imu = (Vector6() << ax, ay, az, wx, wy, wz).finished();
        //     std::cout << "IMU: " << imu << std::endl;
        //     std::cout << "My integration: " << new_meas.getRotation() << std::endl;
        //     preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), 1./200);
        //     std::cout << "Preintegrated measurement generated: " << preintegrated->deltaRij() << std::endl;
        //     BetweenFactor<Pose3> btf(X(stateCount - 1), X(stateCount), lastPose.between(wTb), priorPoseNoise);
        //     initialValues.insert(X(stateCount), wTb);
        //     lastPose = wTb;
        //     graph->add(btf);
        // }
    }

    // Vector6 imu = (Vector6() << a1, a2, a3, 0.1, 0.2, 0.3).finished();
    // preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), 1);
    // preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), 1);
    // std::cout << preintegrated->deltaXij() << std::endl;

    // NavState prev_state(priorPose, priorVelocity);
    // preintegrated->resetIntegration();
    // preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), 2);
    // std::cout << preintegrated->deltaXij() << std::endl;
    // NavState newstate = preintegrated->predict(prev_state, priorBias);
    // std::cout << newstate << std::endl;
    // new_meas.initialize();
    // ORB_SLAM2::ImuMeasure meas(a1, a2, a3, 0.1, 0.2, 0.3, 1);
    // acc = meas._a;
    // angVel = meas._w;
    // new_meas.integrateMeasurement(acc, angVel, 1);
    // new_meas.integrateMeasurement(acc, angVel, 1);
    // std::cout << new_meas.getRotation() << std::endl;
    // std::cout << new_meas.getTranslation() << std::endl;
    // new_meas.initialize();
    // new_meas.integrateMeasurement(acc, angVel, 2);
    // std::cout << new_meas.getRotation() << std::endl;
    // std::cout << new_meas.getTranslation() << std::endl;
    // R: [
    // 	0.751909, -0.507379, 0.42095;
    // 	0.583715, 0.809161, -0.0673456;
    // 	-0.306446, 0.296353, 0.90458
    // ]
    // p: 1.97591  1.1136  1.4323
    // v: 1.95182  1.2272 1.36459
    // R: [
    // 	0.751909, -0.507379, 0.42095;
    // 	0.583715, 0.809161, -0.0673456;
    // 	-0.306446, 0.296353, 0.90458
    // ]
    // p:   2   1 1.5
    // v:   2   1 1.5
    // GTSAM_TANGENT_PREINTEGRATION
    

    // Add a small final loop closure
    // Eigen::Quaternion finRot3(w, i, j, k);
    // Rot3 r(finRot3);
    // Point3 t(x, y, z);
    // Pose3 finalPose = Pose3(r, t);
    // BetweenFactor<Pose3> lp(X(0), X(stateCount), priorPose.between(finalPose), priorPoseNoise);
    // graph->add(lp);

    gtsam::LevenbergMarquardtOptimizer optimizer(*graph, initialValues);
    result = optimizer.optimize();
    saveTrajectory(initialValues, stateCount, outputFile);

    std::cout << "Sucessfully Completed" << std::endl;
}

void saveTrajectory(const gtsam::Values & v, uint64_t stateCount, std::string filename) {
    std::cout << "Saving Base Trajectory to " << filename << std::endl;
    std::ofstream f;
    f.open(filename.c_str());

    gtsam::Pose3 base;
    f << "x,y,z" << std::endl;
    for (uint64_t i = 0; i < stateCount; i++) {
        base = v.at<gtsam::Pose3>(X(i));
        auto t = base.translation();
        f << t(0) << "," << t(1) << "," << t(2) << std::endl;
    }
    f.close();
    std::cout << "Trajectory Saved!" << std::endl;

}
