#ifndef DL_H
#define DL_H

#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <gtsam/base/Lie.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <vector>
#include <string.h>
#include "ContactUtils.h"

namespace DataLoader {
    Eigen::MatrixXd extractMatrix(const YAML::Node & node, std::string);
    Eigen::MatrixXd Data2PoseMat(Eigen::MatrixXd vec);
    gtsam::LegConfig getConfig(const YAML::Node & node, std::string);
    std::map<std::string, gtsam::LegConfig> loadLegConfig(const std::string & configPath);
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> loadIMUConfig(const std::string &);
    gtsam::imuBias::ConstantBias getIMUBias(const std::string & path);
}

#endif