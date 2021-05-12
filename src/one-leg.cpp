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

void saveTrajectory(const Values&, uint64_t);

int main(int argc, char* argv[])
{
    using namespace gtsam;

    //! Command line setup
    utils::setLogPattern();
    CLI::App app{ "Test file for Multi Leg Contact Factor" };

    std::string leg_config_path;
    app.add_option("-l, --leg-config", leg_config_path, "Path to Leg configuration yaml file");
    std::string dataset_csv_path;
    app.add_option("-d, --data", dataset_csv_path, "Path to dataset csv file");
    std::string imu_config_path;
    app.add_option("-i, --imu-config", imu_config_path, "Path to IMU configuration yaml file");
    size_t max_index = 10;
    app.add_option("-m, --max_index", max_index, "maximum number of lines to read in datafile");

    CLI11_PARSE(app, argc, argv);

    //! Load configuration and setup data
    legged::Dataloader dataloader = legged::Dataloader(imu_config_path, leg_config_path, dataset_csv_path);

    legged::LegConfigMap leg_configs = dataloader.getLegConfigs();
    auto imu_params                  = boost::make_shared<PreintegrationCombinedParams>(dataloader.getImuParams());
    auto imu_bias                    = dataloader.getImuBias();
    auto imu_rate                    = 1.0 / 200.0; //! TODO: Move this to dataloader?

    //! Pre-integrate IMU measurements for the base frame
    std::shared_ptr<PreintegrationType> preintegrated = std::make_shared<PreintegratedImuMeasurements>(imu_params, imu_bias);

    //! Create NonlinearFactorGraph and set of values for the optimization
    Values initial_values;
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    size_t index                = 0;

    //! Symbol shorthand names to refer to common state variables
    // clang-format off
    using symbol_shorthand::X;  //! Body Pose
    using symbol_shorthand::V;  //! Velocity
    using symbol_shorthand::B;  //! Bias
    using symbol_shorthand::Q;  //! Front Left Foot Pose
    using symbol_shorthand::P;  //! Front Right Foot Pose
    using symbol_shorthand::A;  //! Back Left Foot Pose
    using symbol_shorthand::L;  //! Back Right Foot Pose
    // clang-format on

    //! Initialize prior state of the robot
    //! Prior pose of the base frame is always R = I, and t = (0, 0, 0.195) approximate robot rest height
    Pose3 prior_pose(I_3x3, Point3(0, 0, 0.194213));
    auto prior_pose_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.001, 0.001, 0.001, 0.05, 0.05, 0.05).finished());  // rad,rad,rad, m, m, m

    Vector3 prior_velocity(0, 0, 0);
    auto prior_velocity_noise = noiseModel::Isotropic::Sigma(3, 0.001);

    imuBias::ConstantBias prior_bias = imu_bias;
    auto bias_noise                  = noiseModel::Isotropic::Sigma(6, 1e-3);

    //! Not used yet
    //! Those are the approximate rest location at spawn (Read frontleftfoot_T_world)
    Pose3 prior_frontleftfoot(Rot3(0, -0.303, 0, 0.953), Point3(0.096, 0.102, 0));
    Pose3 prior_frontrightfoot(Rot3(0, -0.303, 0, 0.953), Point3(0.096, -0.102, 0));
    Pose3 prior_backleftfoot(Rot3(0, -0.303, 0, 0.953), Point3(-0.135, 0.102, 0));
    Pose3 prior_backrightfoot(Rot3(0, -0.303, 0, 0.953), Point3(-0.135, -0.102, 0));

    initial_values.insert(X(index), prior_pose);
    initial_values.insert(V(index), prior_velocity);
    initial_values.insert(B(index), prior_bias);
    graph->addPrior(X(index), prior_pose, prior_pose_noise);
    graph->addPrior(V(index), prior_velocity, prior_velocity_noise);
    graph->addPrior(B(index), prior_bias, bias_noise);

    //! Read data into these variables
    double timestamp;
    gtsam::Pose3 final_pose_reading;
    gtsam::Vector6 imu_reading;
    std::array<gtsam::Vector3, 4> leg_encoder_readings;
    gtsam::Vector encoder;
    legged::LegContactMeasurements contact, prev_contact;
    bool is_robot_ready = false;

    while (dataloader.readDatasetLine(timestamp, final_pose_reading, imu_reading, leg_encoder_readings, contact) &&
           index++ < max_index)
    {
        // Wait until all four legs are on the ground
        // since they are initially floating
        if (contact.frontleft == contact.frontright == contact.backleft == contact.backright == true && !is_robot_ready)
        {
            is_robot_ready = true;
            INFO("Robot marked ready at {}", index);
        }
        else if (!is_robot_ready)
            continue;

        //! No change in contact state for front left leg
        if (prev_contact.frontleft == contact.frontleft)
        {
            preintegrated->integrateMeasurement(imu_reading.head<3>(), imu_reading.tail<3>(), imu_rate);
        }
        else
        {
            preintegrated->integrateMeasurement(imu_reading.head<3>(), imu_reading.tail<3>(), imu_rate);

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
                encoder                          = leg_encoder_readings.at(0);
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
