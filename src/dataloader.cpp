/******************************************************************************
 * File:             dataloader.cpp
 *
 * Author:           Ruoyang Xu & Akash Sharma
 * Created:          05/08/21
 * Description:
 *****************************************************************************/
#include "dataloader.h"

namespace legged
{
    Dataloader::Dataloader(const std::string& imu_config_path,
                           const std::string& leg_config_path,
                           const std::string& dataset_csv_path)
        : imu_config_path_(imu_config_path),
          leg_config_path_(leg_config_path),
          dataset_csv_path_(dataset_csv_path),
          csv_reader_(dataset_csv_path_)
    {
        loadImuConfig();
        loadLegConfigs();

        readCsvHeader();
    }

    void Dataloader::loadLegConfigs()
    {
        YAML::Node node = YAML::LoadFile(leg_config_path_);

        std::vector<std::string> leg_names{ "front_left", "front_right", "back_left", "back_right" };
        for (auto name : leg_names)
        {
            auto leg = loadLegConfig(node, name);
            leg_config_map_.insert(std::pair<std::string, LegConfig>(name, leg));
        }
    }

    LegConfig Dataloader::loadLegConfig(const YAML::Node& node, const std::string& leg_name)
    {
        std::vector<gtsam::Vector6> twists;

        twists.push_back(extractTwist(node, "r1", leg_name + "hip_T_ul", leg_name + "hip_T_ul"));
        twists.push_back(extractTwist(node, "r2", leg_name + "hip_T_ul", leg_name + "ul_T_ll"));
        twists.push_back(extractTwist(node, "r3", leg_name + "hip_T_ul", leg_name + "ll_T_foot"));

        gtsam::Pose3 hip_T_ul   = extractPose(node, leg_name + "hip_T_ul");
        gtsam::Pose3 ul_T_ll    = extractPose(node, leg_name + "ul_T_ll");
        gtsam::Pose3 ll_T_foot  = extractPose(node, leg_name + "ll_T_foot");
        gtsam::Pose3 hip_T_foot = extractPose(node, leg_name + "hip_T_foot");

        //! The actual contact pad is off by 0.0234 meters
        gtsam::Pose3 end_eff_config_t0 = hip_T_foot * gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -0.0234, 0));

        gtsam::Pose3 base_T_joint = extractPose(node, leg_name + "base_T_joint");

        LegConfig config(twists, base_T_joint, end_eff_config_t0, gtsam::Matrix33::Identity());
        return config;
    }

    gtsam::Vector6 Dataloader::extractTwist(const YAML::Node& node,
                                            std::string attribute,
                                            std::string hip_attribute,
                                            std::string joint_attribute) const
    {
        const std::vector<double> axis       = node[attribute].as<std::vector<double>>();
        const std::vector<double> hip_data   = node[hip_attribute].as<std::vector<double>>();
        const std::vector<double> joint_data = node[joint_attribute].as<std::vector<double>>();

        Eigen::Vector3d omega(axis[0], axis[1], axis[2]);
        Eigen::Vector3d position(joint_data[0] - hip_data[0], joint_data[1] - hip_data[1], joint_data[2] - hip_data[2]);

        Eigen::Vector3d rotated_pos = position.cross(omega);

        return (gtsam::Vector6(6) << omega, rotated_pos).finished();
    }

    gtsam::Pose3 Dataloader::extractPose(const YAML::Node& node, std::string attribute) const
    {
        const std::vector<double> vec = node[attribute].as<std::vector<double>>();
        Eigen::Matrix<double, 1, 7, Eigen::RowMajor> mat(vec.data());
        Eigen::MatrixXd data = Eigen::MatrixXd(mat);

        gtsam::Quaternion q(data(6), data(3), data(4), data(5));
        gtsam::Rot3 r(q);
        gtsam::Point3 t(data(0), data(1), data(2));

        return gtsam::Pose3(r, t);
    }

    void Dataloader::loadImuConfig()
    {
        YAML::Node node = YAML::LoadFile(imu_config_path_);
        //! Set gravity to 0.0
        imu_params_ = gtsam::PreintegrationCombinedParams(gtsam::Vector3(0, 0, 0));

        //! Set white noise on the bias and measurements
        gtsam::Matrix33 measured_omega_cov = gtsam::I_3x3 * pow(node["NoiseGyro"].as<double>(), 2);
        gtsam::Matrix33 measured_acc_cov   = gtsam::I_3x3 * pow(node["NoiseAcc"].as<double>(), 2);

        gtsam::Matrix33 bias_acc_cov   = gtsam::I_3x3 * pow(node["AccWalk"].as<double>(), 2);
        gtsam::Matrix33 bias_omega_cov = gtsam::I_3x3 * pow(node["GyroWalk"].as<double>(), 2);

        imu_params_.setAccelerometerCovariance(measured_acc_cov);
        imu_params_.setGyroscopeCovariance(measured_omega_cov);
        imu_params_.setIntegrationCovariance(gtsam::I_3x3 * 1e-8);  //! Magic number from ImuFactorsExample

        imu_params_.setBiasAccCovariance(bias_acc_cov);
        imu_params_.setBiasOmegaCovariance(bias_omega_cov);
        imu_params_.setBiasAccOmegaInt(gtsam::I_6x6 * 1e-5);  //! Magic number from ImuFactorsExample

        const std::vector<double> vecGyro = node["biasGyro"].as<std::vector<double>>();
        const std::vector<double> vecAcce = node["biasAcce"].as<std::vector<double>>();

        Eigen::Matrix<double, 1, 3, Eigen::RowMajor> gyroBiasInitial(vecGyro.data());
        Eigen::Matrix<double, 1, 3, Eigen::RowMajor> acceBiasInitial(vecAcce.data());

        prior_imu_bias_ = gtsam::imuBias::ConstantBias(acceBiasInitial, gyroBiasInitial);
    }

    void Dataloader::readCsvHeader()
    {
        // clang-format off
        csv_reader_.read_header(io::ignore_extra_column,
                                "ts",
                                "x", "y", "z", "i", "j", "k", "w",
                                "wx", "wy", "wz", "ax", "ay", "az",
                                "fl0", "fl1", "fl2",
                                "fr0", "fr1", "fr2",
                                "bl0", "bl1", "bl2",
                                "br0", "br1", "br2",
                                "flc", "frc", "blc", "brc");
        //clang-format on
    }

    bool Dataloader::readDatasetLine(double& timestamp,
                                     gtsam::Pose3& final_pose_reading,
                                     gtsam::Vector6& imu_reading,
                                     std::array<gtsam::Vector3, 4>& leg_encoder_readings,
                                     legged::LegContactMeasurements& leg_contact_readings)
    {
        double ts;
        double x, y, z, i, j, k, w;
        double wx, wy, wz, ax, ay, az;
        double fl0, fl1, fl2;
        double fr0, fr1, fr2;
        double bl0, bl1, bl2;
        double br0, br1, br2;

        int flc, frc, blc, brc;

        // clang-format off
        bool success = csv_reader_.read_row(ts,
                                           x, y, z, i, j, k, w,
                                           wx, wy, wz, ax, ay, az,
                                           fl0, fl1, fl2,
                                           fr0, fr1, fr2,
                                           bl0, bl1, bl2,
                                           br0, br1, br2,
                                           flc, frc, blc, brc);
        // clang-format on
        timestamp = ts;

        gtsam::Quaternion final_quat(w, i, j, k);
        gtsam::Vector3 final_translation(x, y, z);
        final_pose_reading = gtsam::Pose3(final_quat, final_translation);

        imu_reading << ax, ay, az, wx, wy, wz;

        leg_encoder_readings.at(0) = gtsam::Vector3(fl0, fl1, fl2);
        leg_encoder_readings.at(1) = gtsam::Vector3(fr0, fr1, fr2);
        leg_encoder_readings.at(2) = gtsam::Vector3(bl0, bl1, bl2);
        leg_encoder_readings.at(3) = gtsam::Vector3(br0, br1, br2);

        leg_contact_readings.frontleft  = flc;
        leg_contact_readings.frontright = frc;
        leg_contact_readings.backleft   = blc;
        leg_contact_readings.backright  = brc;

        return success;
    }

}  // namespace legged
