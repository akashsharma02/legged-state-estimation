#include "dataloader.h"

using namespace gtsam;

namespace DataLoader
{
    Eigen::MatrixXd extractMatrix(const YAML::Node& node, std::string attribute)
    {
        const std::vector<double> vec = node[attribute].as<std::vector<double>>();
        Eigen::Matrix<double, 1, 7, Eigen::RowMajor> mat(vec.data());
        return Eigen::MatrixXd(mat);
    }

    Eigen::VectorXd extractTwist(const YAML::Node& node,
                                 std::string attribute,
                                 std::string hip_attribute,
                                 std::string joint_attribute)
    {
        const std::vector<double> axis       = node[attribute].as<std::vector<double>>();
        const std::vector<double> hip_data   = node[hip_attribute].as<std::vector<double>>();
        const std::vector<double> joint_data = node[joint_attribute].as<std::vector<double>>();

        Eigen::Vector3d omega(axis[0], axis[1], axis[2]);
        Eigen::Vector3d position(joint_data[0] - hip_data[0], joint_data[1] - hip_data[1], joint_data[2] - hip_data[2]);

        Eigen::Vector3d rotated_pos = position.cross(omega);

        std::cout << "Omega: " << omega << "\n"
                  << "Position: " << position << "\n"
                  << "Rotated Position: " << rotated_pos << std::endl;
        return (Eigen::VectorXd(6) << omega, rotated_pos).finished();
    }

    gtsam::Pose3 dataToPose(Eigen::MatrixXd vec)
    {
        gtsam::Quaternion q(vec(6), vec(3), vec(4), vec(5));
        gtsam::Rot3 r(q);
        gtsam::Point3 t(vec(0), vec(1), vec(2));
        return gtsam::Pose3(r, t);
    }

    gtsam::LegConfig getConfig(const YAML::Node& node, const std::string& leg_name)
    {
        std::vector<Eigen::VectorXd> twists;
        twists.push_back(extractTwist(node, "r1", leg_name + "hTul", leg_name + "hTul"));
        twists.push_back(extractTwist(node, "r2", leg_name + "hTul", leg_name + "ulTll"));
        twists.push_back(extractTwist(node, "r3", leg_name + "hTul", leg_name + "llTf"));

        gtsam::Pose3 jTul  = dataToPose(extractMatrix(node, leg_name + "hTul"));
        gtsam::Pose3 ulTll = dataToPose(extractMatrix(node, leg_name + "ulTll"));
        gtsam::Pose3 llTf  = dataToPose(extractMatrix(node, leg_name + "llTf"));
        gtsam::Pose3 hTf   = dataToPose(extractMatrix(node, leg_name + "hTf"));

        // TODO(Ask ruoyang): The Contact surface is a cylinder
        gtsam::Pose3 endEffectorConfiguration0 = hTf * gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -0.0234, 0));

        gtsam::Pose3 firstJointInBase = dataToPose(extractMatrix(node, leg_name + "bTh"));

        gtsam::LegConfig config(twists, firstJointInBase, endEffectorConfiguration0, Eigen::Matrix3d::Identity());
        return config;
    }

    std::map<std::string, gtsam::LegConfig> loadLegConfig(const std::string& configPath)
    {
        // Load Yaml
        YAML::Node node = YAML::LoadFile(configPath);
        std::map<std::string, gtsam::LegConfig> configs;
        // FL, FR, BL, BR
        std::vector<std::string> leg_names{ "fl", "fr", "bl", "br" };
        for (auto n : leg_names)
        {
            auto leg = getConfig(node, n);
            configs.insert(std::pair<std::string, gtsam::LegConfig>(n, leg));
        }

        return configs;
    }

    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> loadIMUConfig(const std::string& path)
    {
        YAML::Node node = YAML::LoadFile(path);
        auto p          = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);  // This 0.0 sets gravity to 0
        gtsam::Matrix33 measured_omega_cov = gtsam::I_3x3 * pow(node["NoiseGyro"].as<double>(), 2);
        gtsam::Matrix33 measured_acc_cov   = gtsam::I_3x3 * pow(node["NoiseAcc"].as<double>(), 2);
        gtsam::Matrix33 bias_acc_cov       = gtsam::I_3x3 * pow(node["AccWalk"].as<double>(), 2);
        gtsam::Matrix33 bias_omega_cov     = gtsam::I_3x3 * pow(node["GyroWalk"].as<double>(), 2);
        p->gyroscopeCovariance             = measured_omega_cov;
        p->accelerometerCovariance         = measured_acc_cov;
        p->integrationCovariance           = gtsam::I_3x3 * 1e-8;  // Magic Number from ImuFactorsExample.cpp
        p->biasAccCovariance               = bias_acc_cov;
        p->biasOmegaCovariance             = bias_omega_cov;
        p->biasAccOmegaInt                 = gtsam::I_6x6 * 1e-5;  // Magic Number from ImuFactorsExample.cpp
        return p;
    }

    gtsam::imuBias::ConstantBias getIMUBias(const std::string& path)
    {
        YAML::Node node                   = YAML::LoadFile(path);
        const std::vector<double> vecGyro = node["biasGyro"].as<std::vector<double>>();
        Eigen::Matrix<double, 1, 3, Eigen::RowMajor> gyroBiasInitial(vecGyro.data());
        const std::vector<double> vecAcce = node["biasAcce"].as<std::vector<double>>();
        Eigen::Matrix<double, 1, 3, Eigen::RowMajor> acceBiasInitial(vecAcce.data());
        gtsam::imuBias::ConstantBias priorImuBias(acceBiasInitial, gyroBiasInitial);
        return priorImuBias;
    }

}  // namespace DataLoader
