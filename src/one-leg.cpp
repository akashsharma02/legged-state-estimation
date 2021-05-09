/******************************************************************************
 * File:             test.cpp
 *
 * Author:           Ruoyang Xu
 * Created:          04/28/2021
 * Description:      Test files for Point Contact Factor
 *****************************************************************************/

//! Utilities
#include <utils.h>

//! STL libs
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

//! Dependencies libs
#include <fast-cpp-csv-parser/csv.h>
#include <yaml-cpp/yaml.h>
#include <CLI/CLI.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

//! GTSAM libs
#include <gtsam/base/Lie.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

//! Local project headers
#include "BetweenContactFactor.h"
#include "dataloader.h"
#include "leg-utils.h"

using namespace gtsam;

using symbol_shorthand::B;  // Bias
using symbol_shorthand::V;  // Velocity
using symbol_shorthand::X;  // Body Pose
// Welp, Symbol only takes char
// Gonna use Keyboard location
using symbol_shorthand::A;  // BL Foot Pose
using symbol_shorthand::L;  // BR Foot Pose
using symbol_shorthand::P;  // FR Foot Pose
using symbol_shorthand::Q;  // Fl Foot Pose

void saveTrajectory(const Values&, uint64_t);

int main(int argc, char* argv[])
{
    //! Command line setup
    utils::setLogPattern();
    CLI::App app{ "Test file for Multi Leg Contact Factor" };

    std::string leg_config_path;
    app.add_option("-l, --leg-config", leg_config_path, "Path to Leg configuration yaml file");
    std::string dataset_file_path;
    app.add_option("-d, --data", dataset_file_path, "Path to dataset csv file");
    std::string imu_config_path;
    app.add_option("-i, --imu-config", imu_config_path, "Path to IMU configuration yaml file");
    size_t max_index = 10;
    app.add_option("-m, --max_index", max_index, "maximum number of lines to read in datafile");

    CLI11_PARSE(app, argc, argv);

    legged::Dataloader dataloader = legged::Dataloader(imu_config_path, leg_config_path, dataset_file_path);

    legged::LegConfigMap leg_configs = dataloader.getLegConfigs();
    auto imu_params                  = dataloader.getImuParams();
    auto imu_bias                    = dataloader.getImuBias();

    // Setup Dataset
    bool robotReady = false;

    // This is for the Base
    std::shared_ptr<PreintegrationType> preintegrated = std::make_shared<PreintegratedImuMeasurements>(imu_params, imu_bias);

    // Logging for the contact Frames, they are not factors, just preintegrated measurements
    auto pQ                                     = dataloader.getLegConfigs();
    auto pP                                     = dataloader.getLegConfigs();
    auto pA                                     = dataloader.getLegConfigs();
    auto pL                                     = dataloader.getLegConfigs();
    std::shared_ptr<PreintegrationType> imuLogQ = std::make_shared<PreintegratedImuMeasurements>(pQ, imu_bias);
    std::shared_ptr<PreintegrationType> imuLogP = std::make_shared<PreintegratedImuMeasurements>(pP, imu_bias);
    std::shared_ptr<PreintegrationType> imuLogA = std::make_shared<PreintegratedImuMeasurements>(pA, imu_bias);
    std::shared_ptr<PreintegrationType> imuLogL = std::make_shared<PreintegratedImuMeasurements>(pL, imu_bias);

    // Setup Prior Values
    Rot3 priorRotation(I_3x3);          // Default Always identity
    Point3 priorPoint(0, 0, 0.194213);  // This number is approximately rest height
    Pose3 priorPose(priorRotation, priorPoint);
    Eigen::Vector3d priorVelocity(0, 0, 0);
    Values initialValues;
    uint64_t stateCount = 0;
    NavState prevState(priorPose, priorVelocity);
    NavState propState             = prevState;
    imuBias::ConstantBias prevBias = imu_bias;

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
    /* initialValues.insert(Q(stateCount), priorfl); */
    /* initialValues.insert(P(stateCount), priorfr); */
    /* initialValues.insert(A(stateCount), priorbl); */
    /* initialValues.insert(L(stateCount), priorbr); */

    // Setup FactorGraph
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    double dt                   = 1. / 200;
    // if (false) {
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip      = 1;
    ISAM2* isam2                    = new ISAM2(parameters);
    // }

    // Setup Prior Factors
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.001, 0.001, 0.001, 0.05, 0.05, 0.05).finished());  // rad,rad,rad, m, m, m
    auto priorVelNoise = noiseModel::Isotropic::Sigma(3, 0.001);
    auto biasNoise     = noiseModel::Isotropic::Sigma(6, 1e-3);

    graph->addPrior(X(stateCount), priorPose, priorPoseNoise);
    graph->addPrior(V(stateCount), priorVelocity, priorVelNoise);
    graph->addPrior(B(stateCount), priorBias, biasNoise);
    /* graph->addPrior(Q(stateCount), priorfl, priorPoseNoise); */
    /* graph->addPrior(P(stateCount), priorfr, priorPoseNoise); */
    /* graph->addPrior(A(stateCount), priorbl, priorPoseNoise); */
    /* graph->addPrior(L(stateCount), priorbr, priorPoseNoise); */

    // Contact State Tracking
    // Leg {(f)ront, (l)eft}, (C)ontact, (L)ast
    int flc, frc, blc, brc;
    int flcl, frcl, blcl, brcl;
    legged::LegMeasurement* fl;
    legged::LegMeasurement* fr;
    legged::LegMeasurement* bl;
    legged::LegMeasurement* br;
    ContactStates flState;

    Values result;
    if (debug)
    {
        graph->print("\nFactor Graph:\n");
        initialValues.print("\nInitial Estimate:\n");
        LevenbergMarquardtOptimizer optimizer(*graph, initialValues);
        result = optimizer.optimize();
        result.print("Final Result:\n");
    }

    size_t index = 0;
    double timestamp;
    gtsam::Pose3 final_pose_reading;
    gtsam::Vector6 imu_reading;
    std::array<gtsam::Vector3, 4> leg_encoder_readings;
    gtsam::Vector encoder;
    std::array<int, 4> leg_contact_readings;

    while (
        dataloader.readDatasetLine(timestamp, final_pose_reading, imu_reading, leg_encoder_readings, leg_contact_readings) &&
        index++ < max_index)
    {
        // Wait until All Four legs are on the ground
        // since they were initially floating
        flc = leg_contact_readings.at(0);
        frc = leg_contact_readings.at(1);
        blc = leg_contact_readings.at(2);
        brc = leg_contact_readings.at(3);
        
        if (flc == 1 && frc == 1 && blc == 1 && brc == 1 && !robotReady)
        {
            // Mark ready
            robotReady = true;
            std::cout << "Marked Ready on " << idx << " th line" << std::endl;

            // Priors are already added
        }
        else if (!robotReady)
        {
            // If not ready and also not on the ground, skip the rest
            continue;
        }

        if (flcl == flc)
        {
            //  && frcl == frc && blcl == blc && brcl == brc) {
            // Contact Sensor has no change
            // Vector6 imu = (Vector6() << ax, ay, az, wx, wy, wz).finished();
            preintegrated->integrateMeasurement(imu_reading.head<3>(), imu_reading.tail<3>(), dt);
            // All other update
            imuLogQ->integrateMeasurement(imu_reading.head<3>(), imu_reading.tail<3>(), dt);
            /* imuLogP->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt); */
            /* imuLogA->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt); */
            /* imuLogL->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt); */
        }
        else
        {
            // std::cout << "Handle IMU Factor" << std::endl;

            // Handle IMU Factors
            stateCount++;
            // Vector6 imu = (Vector6() << ax, ay, az, wx, wy, wz).finished();
            preintegrated->integrateMeasurement(imu_reading.head<3>(), imu_reading.tail<3>(), dt);

            auto preintIMU = dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated);

            ImuFactor imuFactor(
                X(stateCount - 1), V(stateCount - 1), X(stateCount), V(stateCount), B(stateCount - 1), preintIMU);
            graph->add(imuFactor);
            auto bias = priorBias;

            graph->add(BetweenFactor<imuBias::ConstantBias>(B(stateCount - 1), B(stateCount), bias, biasNoise));
            propState = preintegrated->predict(prevState, prevBias);
            initialValues.insert(X(stateCount), propState.pose());
            initialValues.insert(V(stateCount), propState.v());
            initialValues.insert(B(stateCount), prevBias);

            INFO("Graph size before making contact: {}", graph->size());
            graph->print();
            INFO("initialValues size: {}", initialValues.size());
            initialValues.print();
            // *******************************************
            // Handle the Legs
            if (flcl == 1 && flc == 0)
            {
                // Front Left Leg breaks contact
                if (!fl)
                {
                    std::cout << "FL Pointer null" << std::endl;
                    return 1;
                }

                INFO("Break contact at timestamp: {}", idx);

                // Vector encoder = (Vector3() << fl0, fl1, fl2).finished();
                encoder = leg_encoder_readings.at(0);
                imuLogQ->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
                fl->integrateNewMeasurement(imuLogQ->deltaRij(), encoder, double(ts) / 200);
                Matrix noiseMatrix = fl->getNoise();
                BetweenPointContactFactor contactFactor(noiseMatrix,
                                                        X(flState.baseMakeContact),
                                                        X(stateCount),
                                                        Q(flState.contactMakeContact),
                                                        Q(stateCount),
                                                        Pose3());
                graph->add(contactFactor);

                // Need to add initial estimate
                Pose3 baseTcontact = Pose3(LegMeasurement::efInBaseExpMap(encoder, legConfigs["fl"]));
                baseTcontact       = propState.pose().compose(baseTcontact);
                /* initialValues.insert(Q(flState.contactMakeContact), fl->makeContact); */

                initialValues.insert(Q(stateCount), baseTcontact);

                imuLogQ->resetIntegrationAndSetBias(prevBias);
            }
            else if (flcl == 0 && flc == 1)
            {
                // Front Left Leg makes contact

                // *************************
                // Add Forward kinematics factor

                INFO("Started contact at timestamp: {}", idx);
                // Insert initialValues etc
                // *************************
                flState.baseMakeContact    = stateCount;
                flState.contactMakeContact = stateCount;

                // Makes Contact Factor
                encoder = leg_encoder_readings.at(0);
                gtsam::Matrix base_T_contact_jac = LegMeasurement::baseToContactJacobian(encoder, legConfigs["fl"]);
                std::cout << base_T_contact_jac << std::endl;
                gtsam::Matrix3 encoder_covariance_matrix = Eigen::Matrix3d::Identity() * 0.0174;
                gtsam::Matrix6 FK_covariance =
                    base_T_contact_jac * encoder_covariance_matrix * base_T_contact_jac.transpose();

                Pose3 baseTcontact = Pose3(LegMeasurement::efInBaseExpMap(encoder, legConfigs["fl"]));

                // This gives Contact Frame Relative to World position
                fl = new LegMeasurement(legConfigs["fl"], propState.pose(), baseTcontact, dt, double(ts) / 200.);
                imuLogQ->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

                initialValues.insert(Q(stateCount), propState.pose().compose(baseTcontact));

                std::cout << FK_covariance << std::endl;
                std::cout << noiseModel::Gaussian::Covariance(FK_covariance)->covariance() << std::endl;
                BetweenFactor<Pose3> fk_factor(
                    X(stateCount), Q(stateCount), baseTcontact, noiseModel::Gaussian::Covariance(FK_covariance));

                graph->add(fk_factor);

                INFO("Graph size: {}", graph->size());
                graph->print();
                INFO("Initial values size: {}", initialValues.size());
            }
            else
            {
                // No change
                imuLogQ->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

                INFO("No change");
            }

            // *******************************************

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
            prevState = NavState(result.at<Pose3>(X(stateCount)), result.at<Vector3>(V(stateCount)));
            prevBias  = result.at<imuBias::ConstantBias>(B(stateCount));
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

    // Eigen::Quaternion finRot3(w, i, j, k);
    // Rot3 r(finRot3);
    // Point3 t(x, y, z);
    // Pose3 g = Pose3(r, t);
    // std::cout << "Final Pose: " << g << std::endl;
    // BetweenFactor<Pose3> lp(X(0), X(stateCount), priorPose.between(g), priorPoseNoise);
    // graph->add(lp);

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
    prevState = NavState(result.at<Pose3>(X(stateCount)), result.at<Vector3>(V(stateCount)));
    prevBias  = result.at<imuBias::ConstantBias>(B(stateCount));
    preintegrated->resetIntegrationAndSetBias(prevBias);

    saveTrajectory(result, stateCount);
    std::cout << "Sucessfully Completed" << std::endl;
}

void saveTrajectory(const Values& v, uint64_t stateCount)
{
    std::string filename = "Trajectory.txt";
    std::cout << "Saving Base Trajectory to " << filename << std::endl;
    std::ofstream f;
    f.open(filename.c_str());

    Pose3 base;
    f << "x,y,z" << std::endl;
    for (uint64_t i = 0; i < stateCount; i++)
    {
        base   = v.at<Pose3>(X(i));
        auto t = base.translation();
        f << t(0) << "," << t(1) << "," << t(2) << std::endl;
    }
    f.close();
    std::cout << "Trajectory Saved!" << std::endl;
}
