#include "dataloader.h"

using namespace gtsam;

namespace DataLoader {
    Eigen::MatrixXd extractMatrix(const YAML::Node& node, std::string attribute) {
        const std::vector<double> vec = node[attribute].as< std::vector<double> >();
        Eigen::Matrix<double, 1, 7, Eigen::RowMajor> mat(vec.data());
        return Eigen::MatrixXd(mat);
    }

    Eigen::MatrixXd Data2PoseMat(Eigen::MatrixXd vec) {
        gtsam::Quaternion q(vec(6), vec(3), vec(4), vec(5));
        gtsam::Rot3 r(q);
        gtsam::Point3 t(vec(0), vec(1), vec(2));
        gtsam::Pose3 p(r, t);
        return p.matrix();
    }

    gtsam::LegConfig getConfig(const YAML::Node& node, std::string legName) {

        gtsam::LegConfig config;
        config.firstJointInBase = Data2PoseMat(extractMatrix(node, legName + "bTh"));
        config.jTul = Data2PoseMat(extractMatrix(node, legName + "hTul"));
        config.ulTll = Data2PoseMat(extractMatrix(node,legName + "ulTll"));
        config.llTf = Data2PoseMat(extractMatrix(node, legName + "llTf"));
        // The Contact surface is a cylinder
        config.llTf(2, 3) -= 0.0234;
        config.covSlip = Eigen::MatrixXd::Identity(3, 3);
        return config;
    }

    std::map<std::string, gtsam::LegConfig> loadLegConfig(std::string configPath) {
        // Load Yaml
        YAML::Node node = YAML::LoadFile(configPath);
        std::map<std::string, gtsam::LegConfig> configs;
        // FL, FR, BL, BR
        std::vector<std::string> legNames{"fl", "fr", "bl", "br"};
        for (auto n : legNames) {
            auto leg = getConfig(node, n);
            configs.insert(std::pair<std::string, gtsam::LegConfig>(n, leg));
        }

        return configs;
    }
} // DataLoader
