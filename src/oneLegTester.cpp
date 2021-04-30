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
    initialValues.insert(Q(stateCount), priorfl);
    initialValues.insert(P(stateCount), priorfr);
    initialValues.insert(A(stateCount), priorbl);
    initialValues.insert(L(stateCount), priorbr);

    // Setup FactorGraph
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    double dt = 1. / 200;
    if (false) {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        ISAM2* isam2 = new ISAM2(parameters);
    }

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

    // Contact State Tracking
    // Leg {(f)ront, (l)eft}, (C)ontact, (L)ast
    int flcl, frcl, blcl, brcl;
    LegMeasurement* fl;
    LegMeasurement* fr;
    LegMeasurement* bl;
    LegMeasurement* br;
    ContactStates flState;
    while (datafile.read_row(
        ts,wx,wy,wz,ax,ay,az,fl0,fl1,fl2,fr0,fr1,fr2,bl0,bl1,bl2,br0,br1,br2,flc,frc,blc,brc
        ) && idx++ < maxIdx) {

        // Wait until All Four legs are on the ground
        // since they were initially floating
        if (flc == 1 && frc == 1 && blc == 1 && brc == 1 && !robotReady) {
            // Mark ready
            robotReady = true;

            // Priors are already added
        } else if (!robotReady) {
            // If not ready and also not on the ground, skip the rest
            continue;
        }

        if (flcl == flc && frcl == frc && blcl == blc && brcl == brc) {
            // Contact Sensor has no change
            Vector6 imu = (Vector6() << ax, ay, az, wx, wy, wz).finished();
            preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
            // All other update
            imuLogQ->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
            imuLogP->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
            imuLogA->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
            imuLogL->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
        } else {

            // Handle IMU Factors
            stateCount++;
            Vector6 imu = (Vector6() << ax, ay, az, wx, wy, wz).finished();
            preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
            
            auto preintIMU = dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated);
            ImuFactor imuFactor(
                X(stateCount - 1), V(stateCount - 1),
                X(stateCount), V(stateCount - 1),
                B(stateCount - 1), preintIMU
            );
            graph->add(imuFactor);
            auto bias = priorBias;
            graph->add(BetweenFactor<imuBias::ConstantBias>(
                B(stateCount - 1), B(stateCount), bias, biasNoise
            ));

            propState = preintegrated->predict(prevState, prevBias);
            initialValues.insert(X(stateCount), propState.pose());
            initialValues.insert(V(stateCount), propState.v());
            initialValues.insert(B(stateCount), prevBias);

            // *******************************************
            // Handle the Legs
            if (flcl == 1 && flc == 0) {
                // Front Left Leg breaks contact
                if (!fl) {
                    std::cout << "FL Pointer null" << std::endl;
                    return 1;
                }

                Vector encoder = (Vector3() << fl0, fl1, fl2).finished();
                imuLogQ->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
                fl->integrateNewMeasurement(imuLogQ->deltaRij(), encoder, double(ts) / 200);
                Matrix noiseMatrix = fl->getNoise();
                // noiseModel::Gaussian::Covariance noise(noiseMatrix);
                BetweenPointContactFactor contactFactor(X(flState.baseMakeContact), X(stateCount), 
                    Q(flState.contactMakeContact), Q(stateCount), noiseMatrix);
                graph->add(contactFactor);

                imuLogQ->resetIntegrationAndSetBias(prevBias);
            } else if (flcl == 0 && flc == 1) {
                // Front Left Leg makes contact

                // *************************
                // Add Forward kinematics factor

                // Insert initialValues etc
                // *************************
                flState.baseMakeContact = stateCount;
                flState.contactMakeContact = stateCount;

                // Makes Contact Factor
                Vector encoder = (Vector3() << fl0, fl1, fl2).finished();
                Pose3 baseTcontact = LegMeasurement::efInBase(encoder, legConfigs["fl"]);
                baseTcontact = propState.pose().compose(baseTcontact);
                // This gives Contact Frame Relative to World position
                fl = new LegMeasurement(legConfigs["fl"], propState.pose(), baseTcontact, dt, double(ts) / 200.);
                imuLogQ->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
            } else {
                // No change
                imuLogQ->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
            }


            // *******************************************
            
            Values result;
            LevenbergMarquardtOptimizer optimizer(*graph, initialValues);
            result = optimizer.optimize();
            prevState = NavState(result.at<Pose3>(X(stateCount)),
                result.at<Vector3>(V(stateCount)));
            prevBias = result.at<imuBias::ConstantBias>(B(stateCount));
            preintegrated->resetIntegrationAndSetBias(prevBias);

        }

        // if ready
            // If contact sensor no changes
                // accumulate IMU Measurment
            // if contact sensor changes
                // Create New Node in the Factor Graph
                    // IMU Factor
                    // Point Contact Factor
                    // Forward Kinematic Factor

        flcl = flc;
        frcl = frc;
        blcl = blc;
        brcl = brc;
    }

    std::cout << "Sucessfully Completed" << std::endl;
}
