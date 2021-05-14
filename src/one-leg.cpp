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

void saveTrajectory(const gtsam::Values&, uint64_t, std::string, std::map<std::string, double>&);

int main(int argc, char* argv[])
{
    using namespace gtsam;

    //! Command line setup
    utils::setLogPattern();
    CLI::App app{ "Test file for Multi Leg Contact Factor" };

    std::string leg_config_path;
    std::string dataset_csv_path;
    std::string imu_config_path;
    size_t max_index = 10;
    std::string output_file("");
    std::string unoptimized_file("");
    bool vio = false;
    bool leg = false;
    app.add_option("-c, --config", leg_config_path, "Path to Leg configuration yaml file");
    app.add_option("-d, --data", dataset_csv_path, "Path to dataset csv file");
    app.add_option("-i, --imu-config", imu_config_path, "Path to IMU configuration yaml file");
    app.add_option("-m, --max_index", max_index, "maximum number of lines to read in datafile");
    app.add_option("-o, --output", output_file, "trajectory Output filename");
    app.add_option("-u, --unopt", unoptimized_file, "trajectory Output of unoptimized graph");
    app.add_option("-v, --visual", vio, "enable visual loop closures");
    app.add_option("-l, --leg", leg, "Leg Factors");

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
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip      = 1;
    ISAM2* isam2                    = new ISAM2(parameters);
    size_t state_idx                = 0;

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

    NavState prev_state(prior_pose, prior_velocity);
    NavState pred_state;

    //! Not used yet
    //! Those are the approximate rest location at spawn (Read frontleftfoot_T_world)
    Pose3 prior_frontleftfoot(Rot3(0, -0.303, 0, 0.953), Point3(0.096, 0.102, 0));
    Pose3 prior_frontrightfoot(Rot3(0, -0.303, 0, 0.953), Point3(0.096, -0.102, 0));
    Pose3 prior_backleftfoot(Rot3(0, -0.303, 0, 0.953), Point3(-0.135, 0.102, 0));
    Pose3 prior_backrightfoot(Rot3(0, -0.303, 0, 0.953), Point3(-0.135, -0.102, 0));

    Values bookKeeping;
    initial_values.insert(X(state_idx), prev_state.pose());
    initial_values.insert(V(state_idx), prev_state.v());
    initial_values.insert(B(state_idx), prior_bias);
    bookKeeping.insert(X(state_idx), prev_state.pose());
    bookKeeping.insert(V(state_idx), prev_state.v());
    bookKeeping.insert(B(state_idx), prior_bias);
    graph->addPrior(X(state_idx), prior_pose, prior_pose_noise);
    graph->addPrior(V(state_idx), prior_velocity, prior_velocity_noise);
    graph->addPrior(B(state_idx), prior_bias, bias_noise);

    //! Initialize previous state_idx variables
    imuBias::ConstantBias prev_bias = prior_bias;

    //! Read data into these variables
    size_t index = 0;
    double timestamp;
    gtsam::Pose3 final_pose_reading;
    gtsam::Pose3 last_pose = prior_pose;
    size_t lp_count        = 0;
    size_t max_lp_count    = 25;
    uint64_t last_lp_idx   = 0;
    gtsam::Vector6 imu_reading;
    std::array<gtsam::Vector3, 4> leg_encoder_readings;
    legged::LegContactMeasurements contact, prev_contact;
    bool is_robot_ready = false;

    //! Pre-integrate IMU measurements for the base frame
    std::shared_ptr<PreintegrationType> preintegrated =
        std::make_shared<PreintegratedCombinedMeasurements>(imu_params, imu_bias);
    //! Pre-integrate IMU measurements for the front left frame
    std::shared_ptr<PreintegrationType> imuLogQ = std::make_shared<PreintegratedCombinedMeasurements>(imu_params, imu_bias);
    //! Pre-integrate leg contact measurements for point-contact factor
    std::shared_ptr<legged::PreintegratedContactMeasurement> frontleft_contact_pim =
        std::make_shared<legged::PreintegratedContactMeasurement>(leg_configs.at("front_left"), imu_rate);

    legged::ContactStates* flState;

    std::map<std::string, double> tsMap;

    while (dataloader.readDatasetLine(timestamp, final_pose_reading, imu_reading, leg_encoder_readings, contact) &&
           index++ < max_index)
    {
        // Wait until all four legs are on the ground
        // since they are initially floating
        if (contact.frontleft == true && contact.frontright == true && contact.backleft == true &&
            contact.backright == true && !is_robot_ready)
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
            imuLogQ->integrateMeasurement(imu_reading.head<3>(), imu_reading.tail<3>(), imu_rate);
        }
        else
        {
            state_idx++;
            {
                preintegrated->integrateMeasurement(imu_reading.head<3>(), imu_reading.tail<3>(), imu_rate);
                imuLogQ->integrateMeasurement(imu_reading.head<3>(), imu_reading.tail<3>(), imu_rate);
            }

            auto combined_measurement = dynamic_cast<const PreintegratedCombinedMeasurements&>(*preintegrated);
            CombinedImuFactor imu_factor(X(state_idx - 1),
                                         V(state_idx - 1),
                                         X(state_idx),
                                         V(state_idx),
                                         B(state_idx - 1),
                                         B(state_idx),
                                         combined_measurement);
            graph->add(imu_factor);

            if (vio)
            {
                lp_count++;
                if (lp_count > max_lp_count)
                {
                    lp_count = 0;
                    BetweenFactor<Pose3> btf(
                        X(last_lp_idx), X(state_idx), last_pose.between(final_pose_reading), prior_pose_noise);
                    graph->add(btf);
                    // graph->addPrior(X(state_idx), final_pose_reading, prior_pose_noise);
                    last_pose   = final_pose_reading;
                    last_lp_idx = state_idx;
                }
            }

            pred_state = preintegrated->predict(prev_state, prev_bias);

            initial_values.insert(X(state_idx), pred_state.pose());
            initial_values.insert(V(state_idx), pred_state.v());
            initial_values.insert(B(state_idx), prev_bias);

            std::string mapKey = std::to_string(state_idx);
            tsMap.insert({mapKey, timestamp});

            bookKeeping.insert(X(state_idx), pred_state.pose());
            bookKeeping.insert(V(state_idx), pred_state.v());
            bookKeeping.insert(B(state_idx), prev_bias);

            // pred_state = prev_state;

            //! Front left leg breaks contact
            if (leg)
            {
                if (prev_contact.frontleft == true && contact.frontleft == false)
                {
                    const auto& encoder = leg_encoder_readings.at(0);

                    // No need to do NavState predict since we only need the Rotation part
                    frontleft_contact_pim->integrateMeasurement(imuLogQ->deltaRij(), encoder);
                    Matrix noiseMatrix = frontleft_contact_pim->preintMeasCov();
                    BetweenPointContactFactor contactFactor(noiseMatrix,
                                                            flState->base_make_contact_frame_,
                                                            flState->foot_make_contact_frame_,
                                                            Q(state_idx),
                                                            Pose3());
                    graph->add(contactFactor);

                    // Need to add initial estimate
                    // Pose3 baseTcontact = Pose3(LegMeasurement::efInBaseExpMap(encoder, legConfigs["front_left"]));
                    Pose3 baseTcontact = frontleft_contact_pim->getBaseTContactFromEncoder(encoder);

                    gtsam::Rot3 tempR = pred_state.pose().compose(baseTcontact).rotation();
                    gtsam::Point3 tempP = (pred_state.pose().compose(baseTcontact)).translation();
                    tempP = gtsam::Point3(tempP.x(), tempP.y(), 0.);

                    // initial_values.insert(Q(state_idx), gtsam::Pose3(tempR, tempP));
                    graph->addPrior(Q(state_idx), gtsam::Pose3(tempR, tempP), prior_pose_noise);
                    initial_values.insert(Q(state_idx), pred_state.pose().compose(baseTcontact));
                    bookKeeping.insert(Q(state_idx), pred_state.pose().compose(baseTcontact));

                    gtsam::Matrix63 base_T_contact_jac = Eigen::MatrixXd::Ones(6, 3);
                    // std::cout << base_T_contact_jac << std::endl;
                    gtsam::Matrix3 encoder_covariance_matrix = Eigen::Matrix3d::Identity() * 0.0174;
                    gtsam::Matrix6 FK_covariance =
                        base_T_contact_jac * encoder_covariance_matrix * base_T_contact_jac.transpose();
                    // std::cout << FK_covariance << std::endl;
                    FK_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.0174;
                    BetweenFactor<Pose3> fk_factor(
                        X(state_idx), Q(state_idx), baseTcontact, noiseModel::Gaussian::Covariance(FK_covariance));
                    graph->add(fk_factor);

                    imuLogQ->resetIntegrationAndSetBias(prev_bias);
                    //! Front left leg makes contact
                }
                else if (prev_contact.frontleft == false && contact.frontleft == true)
                {
                    // *************************
                    // Add Forward kinematics factor

                    // INFO("Started contact at timestamp: {}", index);

                    // Insert initialValues etc
                    // *************************
                    flState = new legged::ContactStates(X(state_idx), Q(state_idx));
                    // flState.base_make_contact_frame_ = X(state_idx);
                    // flState.foot_make_contact_frame_ = Q(state_idx);

                    // Makes Contact Factor
                    // gtsam::Matrix base_T_contact_jac = LegMeasurement::baseToContactJacobian(encoder, legConfigs["fl"]);
                    gtsam::Matrix63 base_T_contact_jac = Eigen::MatrixXd::Zero(6, 3);
                    // std::cout << base_T_contact_jac << std::endl;
                    gtsam::Matrix3 encoder_covariance_matrix = Eigen::Matrix3d::Identity() * 0.0174;
                    gtsam::Matrix6 FK_covariance =
                        base_T_contact_jac * encoder_covariance_matrix * base_T_contact_jac.transpose();
                    const auto encoder = leg_encoder_readings.at(0);
                    // std::cout << FK_covariance << std::endl;
                    FK_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.0174;
                    // Pose3 baseTcontact = Pose3(LegMeasurement::getBaseTContactFromEncoder(encoder, legConfigs["fl"]));
                    Pose3 baseTcontact = frontleft_contact_pim->getBaseTContactFromEncoder(encoder);
                    BetweenFactor<Pose3> fk_factor(
                        X(state_idx), Q(state_idx), baseTcontact, noiseModel::Gaussian::Covariance(FK_covariance));
                    graph->add(fk_factor);

                    gtsam::Rot3 tempR = pred_state.pose().compose(baseTcontact).rotation();
                    gtsam::Point3 tempP = (pred_state.pose().compose(baseTcontact)).translation();
                    tempP = gtsam::Point3(tempP.x(), tempP.y(), 0.);

                    graph->addPrior(Q(state_idx), gtsam::Pose3(tempR, tempP), prior_pose_noise);
                    initial_values.insert(Q(state_idx), pred_state.pose().compose(baseTcontact));
                    bookKeeping.insert(Q(state_idx), pred_state.pose().compose(baseTcontact));
                    // initial_values.insert(Q(state_idx), gtsam::Pose3(tempR, tempP));
                    // frontleft_contact_pim->initialize(pred_state.pose(), gtsam::Pose3(tempR, tempP));
                    frontleft_contact_pim->initialize(pred_state.pose(), pred_state.pose().compose(baseTcontact));

                    // This gives Contact Frame Relative to World position
                    // fl = new LegMeasurement(legConfigs["fl"], propState.pose(), baseTcontact, dt, double(ts) / 200.);
                    // imuLogQ->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

                    // std::cout << FK_covariance << std::endl;
                    // std::cout << noiseModel::Gaussian::Covariance(FK_covariance)->covariance() << std::endl;

                    // INFO("Graph size: {}", graph->size());
                    // graph->print();
                    // initial_values.print("InitialValues: ");
                }
                // INFO("Initial values size: {}", initial_values.size());
            }
            // *******************************************

            // LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
            // Values result = optimizer.optimize();
            // prev_state = NavState(result.at<Pose3>(X(state_idx)),
            //     result.at<Vector3>(V(state_idx)));
            // prev_bias = result.at<imuBias::ConstantBias>(B(state_idx));
            // preintegrated->resetIntegrationAndSetBias(prev_bias);

            // std::cout << "First Optimize" << std::endl;
            isam2->update(*graph, initial_values);
            isam2->update();
            Values result = isam2->calculateEstimate();
            graph->resize(0);
            initial_values.clear();
            prev_state = NavState(result.at<Pose3>(X(state_idx)), result.at<Vector3>(V(state_idx)));
            prev_bias  = result.at<imuBias::ConstantBias>(B(state_idx));
            preintegrated->resetIntegrationAndSetBias(prev_bias);
        }
        prev_contact = contact;
    }

    INFO("Graph size: {}", graph->size());
    // BetweenFactor<Pose3> fk_factor(
    //     X(0), X(state_idx), prior_pose.between(last_pose), prior_pose_noise);
    // graph->add(fk_factor);

    // LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
    // Values finalResult = optimizer.optimize();

    isam2->update(*graph, initial_values);
    // isam2->update();
    Values finalResult = isam2->calculateEstimate();
    graph->resize(0);
    saveTrajectory(finalResult, state_idx, output_file, tsMap);
    std::cout << "Sucessfully Completed" << std::endl;
    // graph->print();

    if (unoptimized_file.compare("") != 0)
    {
        std::cout << "Saving Unoptimized trajectory" << std::endl;
        saveTrajectory(bookKeeping, state_idx, unoptimized_file, tsMap);
    }
}

void saveTrajectory(const gtsam::Values& v, uint64_t stateCount, std::string filename, std::map<std::string, double>& tsMap)
{
    using gtsam::symbol_shorthand::X;  //! Body Pose
    std::cout << "Saving Base Trajectory to " << filename << std::endl;
    std::ofstream f;
    f.open(filename.c_str());

    gtsam::Pose3 base;
    f << "# ts,x,y,z,i,j,k,w" << std::endl;
    for (uint64_t i = 0; i < stateCount; i++)
    {
        base   = v.at<gtsam::Pose3>(X(i));
        auto t = base.translation();
        auto q = base.rotation().quaternion();
        std::string key = std::to_string(i);
        f << tsMap[key] << ",";
        f << t(0) << "," << t(1) << "," << t(2) << ","; 
        f << q(1) << "," << q(2) << "," << q(3) << "," << q(0) << std::endl;
    }
    f.close();
    std::cout << "Trajectory Saved!" << std::endl;
}
