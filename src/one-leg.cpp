/******************************************************************************
 * File:             one-leg.cpp
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

void saveTrajectory(const gtsam::Values&, uint64_t);

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
    auto imu_rate                    = 1.0 / 200.0;  //! TODO: Move this to dataloader?

    //! Create NonlinearFactorGraph and set of values for the optimization
    Values initial_values;
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    size_t state_idx            = 0;

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
    Pose3 prior_pose(Rot3(I_3x3), Point3(0, 0, 0.194213));
    auto prior_pose_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.001, 0.001, 0.001, 0.05, 0.05, 0.05).finished());  // rad,rad,rad, m, m, m

    Vector3 prior_velocity(0, 0, 0);
    auto prior_velocity_noise = noiseModel::Isotropic::Sigma(3, 0.001);

    imuBias::ConstantBias prior_bias = imu_bias;
    auto bias_noise                  = noiseModel::Isotropic::Sigma(6, 1e-3);

    NavState prior_state(prior_pose, prior_velocity);

    //! Not used yet
    //! Those are the approximate rest location at spawn (Read frontleftfoot_T_world)
    Pose3 prior_frontleftfoot(Rot3(0, -0.303, 0, 0.953), Point3(0.096, 0.102, 0));
    Pose3 prior_frontrightfoot(Rot3(0, -0.303, 0, 0.953), Point3(0.096, -0.102, 0));
    Pose3 prior_backleftfoot(Rot3(0, -0.303, 0, 0.953), Point3(-0.135, 0.102, 0));
    Pose3 prior_backrightfoot(Rot3(0, -0.303, 0, 0.953), Point3(-0.135, -0.102, 0));

    initial_values.insert(X(state_idx), prior_state.pose());
    initial_values.insert(V(state_idx), prior_state.v());
    initial_values.insert(B(state_idx), prior_bias);
    graph->addPrior(X(state_idx), prior_pose, prior_pose_noise);
    graph->addPrior(V(state_idx), prior_velocity, prior_velocity_noise);
    graph->addPrior(B(state_idx), prior_bias, bias_noise);

    //! Initialize previous state_idx variables
    NavState prev_state(prior_state);
    imuBias::ConstantBias prev_bias = prior_bias;

    //! Read data into these variables
    size_t index = 0;
    double timestamp;
    gtsam::Pose3 final_pose_reading;
    gtsam::Vector6 imu_reading;
    legged::LegMeasurements leg_readings, prev_leg_readings;
    bool is_robot_ready = false;

    //! Pre-integrate IMU measurements for the base frame
    std::shared_ptr<PreintegratedImuMeasurements> preintegrated =
        std::make_shared<PreintegratedImuMeasurements>(imu_params, imu_bias);
    //! Pre-integrate leg contact measurements for point-contact factor
    std::shared_ptr<legged::PreintegratedContactMeasurement> frontleft_contact_pim =
        std::make_shared<legged::PreintegratedContactMeasurement>(leg_configs.at("front_left"), imu_rate);

    while (dataloader.readDatasetLine(timestamp, final_pose_reading, imu_reading, leg_readings) && index++ < max_index)
    {
        // Wait until all four legs are on the ground
        // since they are initially floating
        if (leg_readings[0]->getContactState() == true && leg_readings[1]->getContactState() == true &&
            leg_readings[2]->getContactState() == true && leg_readings[3]->getContactState() == true &&
            !is_robot_ready)
        {
            is_robot_ready = true;
            INFO("Robot marked ready at {}", index);
        }
        else if (!is_robot_ready)
            continue;

        preintegrated->integrateMeasurement(imu_reading.head<3>(), imu_reading.tail<3>(), imu_rate);

        //! If front left contact state changes
        if (prev_contact.frontleft != contact.frontleft)
        {
            state_idx++;
            auto bias = prior_bias;

            //! Add IMU + Bias factor
            NavState pred_state = preintegrated->predict(prev_state, prev_bias);

            ImuFactor imu_factor(
                X(state_idx - 1), V(state_idx - 1), X(state_idx), V(state_idx), B(state_idx - 1), *preintegrated);
            BetweenFactor<imuBias::ConstantBias> bias_factor(B(state_idx - 1), B(state_idx), bias, bias_noise);

            graph->add(imu_factor);
            graph->add(bias_factor);

            initial_values.insert(X(state_idx), pred_state.pose());
            initial_values.insert(V(state_idx), pred_state.v());
            initial_values.insert(B(state_idx), prev_bias);

            //! Front left leg breaks contact
            if (prev_contact.frontleft && !contact.frontleft)
            {
                const auto& encoder = leg_encoder_readings.frontleft;

                //! TODO: (Ruoyang Xu) Add contact factor and contact frame pose
                /* fl->integrateNewMeasurement(imuLogQ->deltaRij(), encoder, double(ts) / 200); */
                /* Matrix noiseMatrix = fl->getNoise(); */
                /* BetweenPointContactFactor contactFactor(noiseMatrix, */
                /*                                         X(flState.baseMakeContact), */
                /*                                         X(state_idx), */
                /*                                         Q(flState.contactMakeContact), */
                /*                                         Q(state_idx), */
                /*                                         Pose3()); */
                /* graph->add(contactFactor); */
                /* // Need to add initial estimate */
                /* Pose3 baseTcontact = Pose3(LegMeasurement::efInBaseExpMap(encoder, legConfigs["fl"])); */
                /* baseTcontact       = propState.pose().compose(baseTcontact); */
            }
            //! Front left leg makes contact
            else
            {
                //! Add forward kinematics factor
                const auto& pose_measurement = leg_pose_readings.frontleft;

                gtsam::Matrix J_base_T_contact = LegMeasurement::baseToContactJacobian(encoder, legConfigs["fl"]);

                gtsam::noiseModel::Gaussian fk_noise(6, encoder_noise.Whiten(J_base_T_contact))
                    base_T_contact_jac** base_T_contact_jac.transpose();
                BetweenFactor<Pose3> frontleft_fk_factor(
                    X(state_idx), Q(state_idx), pose_measurement, noiseModel::Gaussian::Covariance(fk_covariance));

                /* NavState pred_state = preintegrated->predict(prev_state, prev_bias); */
                /* frontleft_contact_pim->initialize(pred_state.pose(), pred_state.pose().compose(baseTcontact.inverse()); */
                /* // Insert initialValues etc */
                /* // ************************* */
                /* flState.baseMakeContact    = state_idx; */
                /* flState.contactMakeContact = state_idx; */

                /* // Makes Contact Factor */
                /* encoder                          = leg_encoder_readings.at(0); */
                /* Pose3 baseTcontact = Pose3(LegMeasurement::efInBaseExpMap(encoder, legConfigs["fl"])); */

                /* // This gives Contact Frame Relative to World position */
                /* fl = new LegMeasurement(legConfigs["fl"], propState.pose(), baseTcontact, dt, double(ts) / 200.); */
                /* imuLogQ->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt); */

                /* initialValues.insert(Q(state_idx), propState.pose().compose(baseTcontact)); */

                /* std::cout << FK_covariance << std::endl; */
                /* std::cout << noiseModel::Gaussian::Covariance(FK_covariance)->covariance() << std::endl; */
                /* BetweenFactor<Pose3> fk_factor( */
                /*     X(state_idx), Q(state_idx), baseTcontact, noiseModel::Gaussian::Covariance(FK_covariance)); */

                /* graph->add(fk_factor); */

                /* INFO("Graph size: {}", graph->size()); */
                /* graph->print(); */
                /* INFO("Initial values size: {}", initialValues.size()); */
            }

            // *******************************************

            // LevenbergMarquardtOptimizer optimizer(*graph, initialValues);
            // result = optimizer.optimize();
            // prevState = NavState(result.at<Pose3>(X(state_idx)),
            //     result.at<Vector3>(V(state_idx)));
            // prevBias = result.at<imuBias::ConstantBias>(B(state_idx));
            // preintegrated->resetIntegrationAndSetBias(prevBias);

            /* isam2->update(*graph, initialValues); */
            /* isam2->update(); */
            /* result = isam2->calculateEstimate(); */
            /* graph->resize(0); */
            /* initialValues.clear(); */
            /* prevState = NavState(result.at<Pose3>(X(state_idx)), result.at<Vector3>(V(state_idx))); */
            /* prevBias  = result.at<imuBias::ConstantBias>(B(state_idx)); */
            /* preintegrated->resetIntegrationAndSetBias(prevBias); */
        }

        // if ready
        // If contact sensor no changes
        // accumulate IMU Measurment
        // if contact sensor changes
        // Create New Node in the Factor Graph
        // IMU Factor
        // Point Contact Factor
        // Forward Kinematic Factor

        prev_contact = contact;
    }

    /* isam2->update(*graph, initialValues); */
    /* isam2->update(); */
    /* result = isam2->calculateEstimate(); */
    /* graph->resize(0); */
    /* initialValues.clear(); */
    /* prevState = NavState(result.at<Pose3>(X(state_idx)), result.at<Vector3>(V(state_idx))); */
    /* prevBias  = result.at<imuBias::ConstantBias>(B(state_idx)); */
    /* preintegrated->resetIntegrationAndSetBias(prevBias); */

    /* saveTrajectory(result, state_idx); */
    /* std::cout << "Sucessfully Completed" << std::endl; */
}

/* void saveTrajectory(const gtsam::Values& v, uint64_t state_idx) */
/* { */
/*     std::string filename = "Trajectory.txt"; */
/*     std::cout << "Saving Base Trajectory to " << filename << std::endl; */
/*     std::ofstream f; */
/*     f.open(filename.c_str()); */

/*     gtsam::Pose3 base; */
/*     f << "x,y,z" << std::endl; */
/*     for (uint64_t i = 0; i < state_idx; i++) */
/*     { */
/*         base   = v.at<gtsam::Pose3>(X(i)); */
/*         auto t = base.translation(); */
/*         f << t(0) << "," << t(1) << "," << t(2) << std::endl; */
/*     } */
/*     f.close(); */
/*     std::cout << "Trajectory Saved!" << std::endl; */
/* } */
