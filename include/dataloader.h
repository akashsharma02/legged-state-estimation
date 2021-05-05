#ifndef DL_H
#define DL_H

#include <gtsam/base/Lie.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <string.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <map>
#include <vector>
#include "ContactUtils.h"

namespace DataLoader
{
    Eigen::MatrixXd extractMatrix(const YAML::Node &node, std::string);
    Eigen::VectorXd extractTwist(const YAML::Node& node, std::string attribute);
    gtsam::Pose3 dataToPose(Eigen::MatrixXd vec);
    gtsam::LegConfig getConfig(const YAML::Node &node, const std::string& leg_name);
    std::map<std::string, gtsam::LegConfig> loadLegConfig(const std::string &configPath);
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> loadIMUConfig(const std::string &);
    gtsam::imuBias::ConstantBias getIMUBias(const std::string &path);
}  // namespace DataLoader

#endif
