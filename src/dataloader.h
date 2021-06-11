/******************************************************************************
 * File:             dataloader.h
 *
 * Author:           Ruoyang Xu & Akash Sharma
 * Created:          05/08/21
 * Description:
 *****************************************************************************/

#ifndef LEGGED_ESTIMATION_DATALOADER_H
#define LEGGED_ESTIMATION_DATALOADER_H

#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include <fast-cpp-csv-parser/csv.h>
#include <yaml-cpp/yaml.h>

#include "leg-utils.h"

namespace legged
{
    /*! \class Dataloader
     *  \brief Loads the IMU configuration and Leg configuration from respective yaml files
     *         and reads csv file for new measurements
     *
     *  Detailed description
     */
    class Dataloader
    {
       public:
        Dataloader(const std::string &imu_config_path,
                   const std::string &leg_config_path,
                   const std::string &dataset_csv_path);

        virtual ~Dataloader(){};

        inline LegConfigMap getLegConfigs() const { return leg_config_map_; }
        inline gtsam::PreintegrationCombinedParams getImuParams() const { return imu_params_; }
        inline gtsam::imuBias::ConstantBias getImuBias() const { return prior_imu_bias_; }

        bool readDatasetLine(double &timestamp,
                             gtsam::Pose3 &final_pose_reading,
                             gtsam::Vector6 &imu_reading,
                             legged::LegMeasurements &leg_readings);

       protected:
        void loadLegConfigs();
        void loadImuConfig();

        LegConfig loadLegConfig(const YAML::Node &node, const std::string &leg_name);

        gtsam::Vector6 extractTwist(const YAML::Node &node,
                                    std::string attribute,
                                    std::string hip_attribute,
                                    std::string joint_attribute) const;

        gtsam::Pose3 extractPose(const YAML::Node &node, std::string attribute) const;

        void readCsvHeader();

       private:
        const std::string imu_config_path_;
        const std::string leg_config_path_;
        const std::string dataset_csv_path_;

        LegConfigMap leg_config_map_;
        gtsam::PreintegrationCombinedParams imu_params_;
        gtsam::imuBias::ConstantBias prior_imu_bias_;

        io::CSVReader<23 + 7> csv_reader_;
    };
}  // namespace legged

#endif /* ifndef LEGGED_ESTIMATION_DATALOADER_H */
