/******************************************************************************
 * File:             ContaactUtils.h
 *
 * Author:           Ruoyang Xu
 * Created:          04/27/2021
 * Description:      Utility Functions
 *****************************************************************************/

#ifndef POINTCONTACTFACTOR_H
#define POINTCONTACTFACTOR_H

#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "opencv2/core/core.hpp"

namespace legged
{
    /*! \class LegConfig
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class LegConfig
    {
       public:
        LegConfig(const std::vector<gtsam::Vector6>& twists,
                  gtsam::Pose3 joint_T_base,
                  gtsam::Pose3 eff_config_t0,
                  gtsam::Matrix3 slip_covariance)
            : twists_(twists),
              joint_T_base_(joint_T_base),
              end_eff_config_t0_(eff_config_t0),
              slip_covariance_(slip_covariance)
        {
        }
        ~LegConfig() {}

       protected:
        std::vector<gtsam::Vector6> twists_;
        gtsam::Pose3 joint_T_base_;
        gtsam::Pose3 end_eff_config_t0_;
        gtsam::Matrix33 slip_covariance_;
    };

    using LegConfigMap = std::map<std::string, LegConfig>;

    /* class LegMeasurement */
    /* { */
    /*    public: */
    /*     LegMeasurement() {} */
    /*     LegMeasurement(LegConfig leg_, Pose3 base, Pose3 contact, double dt, double ts) */
    /*     { */
    /*         leg          = leg_; */
    /*         Matrix B     = base.rotation().transpose() * contact.rotation().transpose() * dt; */
    /*         measureNoise = B * leg.covSlip * B.transpose(); */
    /*         ts_          = ts; */
    /*         makeContact  = contact; */
    /*     } */

    /*     ~LegMeasurement() {} */

    /*     static Matrix efInBaseExpMap(Vector encoder, const LegConfig &leg) */
    /*     { */
    /*         Pose3 g = leg.firstJointInBase; */
    /*         for (size_t i = 0; i < encoder.size(); i++) */
    /*         { */
    /*             auto twist = leg.twists.at(i) * encoder(i); */
    /*             g          = g * Pose3::Expmap(twist); */
    /*         } */
    /*         g = g * leg.endEffectorConfiguration0; */
    /*         return g.matrix(); */
    /*     } */

    /*     static Matrix efInBase(Vector encoder, const LegConfig &leg) */
    /*     { */
    /*         auto g = leg.firstJointInBase; */
    /*         g      = g * gtsam::Pose3(gtsam::Rot3::Rx(encoder(0)), gtsam::Point3()); */
    /*         g      = g * leg.jTul; */
    /*         g      = g * gtsam::Pose3(gtsam::Rot3::Ry(encoder(1)), gtsam::Point3()); */
    /*         g      = g * leg.ulTll; */
    /*         g      = g * gtsam::Pose3(gtsam::Rot3::Ry(encoder(2)), gtsam::Point3()); */
    /*         g      = g * leg.llTf; */
    /*         return g.matrix(); */
    /*     } */

    /*     static Matrix baseToContactJacobian(Vector encoder, const LegConfig &leg) */
    /*     { */
    /*         Matrix spatial_jacobian = Matrix::Zero(6, 3); */
    /*         Pose3 g                 = leg.firstJointInBase; */

    /*         spatial_jacobian.col(0) = leg.twists.at(0); */
    /*         for (size_t i = 1; i < encoder.size(); i++) */
    /*         { */
    /*             auto twist = leg.twists.at(i) * encoder(i); */
    /*             std::cout << leg.twists.at(i) << std::endl; */
    /*             spatial_jacobian.col(i) = g.Adjoint(leg.twists.at(i)); */
    /*             g                       = g * Pose3::Expmap(twist); */
    /*         } */
    /*         return spatial_jacobian; */
    /*     } */

    /*     // Update Covariance */
    /*     // Pose3 iGj i -> j, Pose J in frame of Pose i */
    /*     // Obtained through IMU estimation */
    /*     void integrateNewMeasurement(Rot3 iGj, Vector encoder, double ts) */
    /*     { */
    /*         double dt = ts - ts_; */
    /*         Matrix B  = iGj.matrix() * efInBase(encoder, leg).block(0, 0, 3, 3) * dt; */
    /*         measureNoise += B * leg.covSlip * B.transpose(); */
    /*     } */

    /*     Matrix getNoise() { return measureNoise; }; */

    /*    public: */
    /*     LegConfig leg; */
    /*     Matrix measureNoise; */
    /*     double ts_; */
    /*     Pose3 makeContact; */
    /* }; */

    /* struct ContactStates */
    /* { */
    /*     uint64_t baseMakeContact; */
    /*     uint64_t baseBreakContact; */
    /*     uint64_t contactMakeContact; */
    /*     uint64_t contactBreakContact; */
    /* }; */

    /* Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3) */
    /* { */
    /*     Eigen::Matrix<double, 3, 3> M; */

    /*     M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2), cvMat3.at<float>(1, 0), */
    /*         cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2), cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), */
    /*         cvMat3.at<float>(2, 2); */

    /*     return M; */
    /* } */

    /* inline Point3 toPoint3(const cv::Mat &cvMat) */
    /* { */
    /*     Point3 p(cvMat.at<double>(0, 0), cvMat.at<double>(1, 0), cvMat.at<double>(2, 0)); */
    /*     return p; */
    /* } */

}  // namespace legged

#endif