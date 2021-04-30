#ifndef DL_H
#define DL_H

#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <gtsam/base/Lie.h>
#include <vector>
#include <string.h>
#include "ContactUtils.h"

namespace DataLoader {
    Eigen::MatrixXd extractMatrix(const YAML::Node & node, std::string);
    Eigen::MatrixXd Data2PoseMat(Eigen::MatrixXd vec);
    gtsam::LegConfig getConfig(const YAML::Node & node, std::string);
    std::map<std::string, gtsam::LegConfig> loadLegConfig(std::string configPath);
}

#endif