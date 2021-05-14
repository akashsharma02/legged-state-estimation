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
     *  \brief Maintains a full description of a particular leg of the robot
     *
     */
    class LegConfig
    {
       public:
        LegConfig(const std::vector<gtsam::Vector6>& twists,
                  gtsam::Pose3 base_T_joint,
                  gtsam::Pose3 eff_config_t0,
                  gtsam::Matrix3 slip_covariance)
            : twists_(twists),
              base_T_joint_(base_T_joint),
              end_eff_config_t0_(eff_config_t0),
              slip_covariance_(slip_covariance)
        {
        }
        ~LegConfig() {}

        inline gtsam::Matrix33 getSlipCovariance() const { return slip_covariance_; }
        inline gtsam::Pose3 getBaseTJoint() const { return base_T_joint_; }
        inline std::vector<gtsam::Vector6> getTwists() const { return twists_; }
        inline gtsam::Pose3 getEndEffectorConfig() const { return end_eff_config_t0_; }
        gtsam::Matrix63 baseToContactJacobian(gtsam::Vector3 encoder)  {   
            gtsam::Matrix spatial_jacobian = gtsam::Matrix::Zero(6, 3);
            gtsam::Pose3 g                 = base_T_joint_;

            spatial_jacobian.col(0) = twists_.at(0);
            for (Eigen::Index i = 1; i < encoder.size(); i++)
            {
                auto twist = twists_.at(static_cast<size_t>(i)) * encoder(i);
                // std::cout << leg.twists.at(i) << std::endl;
                spatial_jacobian.col(i) = g.Adjoint(twists_.at(static_cast<size_t>(i)));
                g                       = g * gtsam::Pose3::Expmap(twist);
            }
            return spatial_jacobian;
        }

       public:
        std::vector<gtsam::Vector6> twists_;
        gtsam::Pose3 base_T_joint_;
        gtsam::Pose3 end_eff_config_t0_;
        gtsam::Matrix33 slip_covariance_;
    };

    using LegConfigMap = std::map<std::string, LegConfig>;

    /*! \class LegContactMeasurements
     *  \brief Brief class description
     *
     *  Detailed description
     */
    struct LegContactMeasurements
    {
        bool frontleft;
        bool frontright;
        bool backleft;
        bool backright;
    };

    /*! \struct ContactStates
     *  \brief Maintains the keys of the frames that participate in a make-break contact during robot gait
     *
     */
    struct ContactStates
    {
        ContactStates(const gtsam::Key& base_make_contact_frame,
                      const gtsam::Key& foot_make_contact_frame)
            : base_make_contact_frame_(base_make_contact_frame),
              foot_make_contact_frame_(foot_make_contact_frame){};

        ~ContactStates() {};

        gtsam::Key base_make_contact_frame_;
        gtsam::Key foot_make_contact_frame_;
        // gtsam::Key foot_break_contact_frame_;
    };

    /*! \class PreintegratedContactMeasurement
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class PreintegratedContactMeasurement
    {
       public:
        PreintegratedContactMeasurement(const LegConfig& leg, double imu_rate) : leg_(leg), imu_rate_(imu_rate)
        {
            resetIntegration();
        }

        virtual ~PreintegratedContactMeasurement() {};

        void initialize(const gtsam::Pose3& base_frame, const gtsam::Pose3& contact_frame)
        {
            gtsam::Pose3 contact_T_base = base_frame.compose(contact_frame);
            gtsam::Matrix33 B           = contact_T_base.rotation().matrix() * imu_rate_;
            measure_noise_ += B * leg_.getSlipCovariance() * B.transpose();
        }

        void resetIntegration()
        {
            measure_noise_.setZero(); /* TODO: Not sure if should add the first measurement information */
        }

        inline gtsam::Matrix33 preintMeasCov() const { return measure_noise_; }

        inline void integrateMeasurement(const gtsam::Rot3& relative_imu_pim, const gtsam::Vector3 leg_encoder_reading)
        {
            gtsam::Pose3 base_T_contact = getBaseTContactFromEncoder(leg_encoder_reading);

            //! \delta \tilde {R_{ik}} f_R(\tilde{\alpha_k}) \delta t
            gtsam::Matrix33 B = relative_imu_pim.matrix() * base_T_contact.rotation().matrix() * imu_rate_;

            measure_noise_ += B * leg_.getSlipCovariance() * B.transpose();
        }

    //    protected:
        inline gtsam::Pose3 getBaseTContactFromEncoder(const gtsam::Vector3& leg_encoder_reading)
        {
            gtsam::Pose3 base_T_contact = leg_.getBaseTJoint();
            for (Eigen::Index i = 0; i < leg_encoder_reading.size(); i++)
            {
                auto twist     = leg_.getTwists().at(static_cast<long unsigned int>(i)) * leg_encoder_reading(i);
                base_T_contact = base_T_contact * gtsam::Pose3::Expmap(twist);
            }
            base_T_contact = base_T_contact * leg_.getEndEffectorConfig();
            return base_T_contact;
        }

        const LegConfig& leg_;
        double imu_rate_;
        gtsam::Matrix33 measure_noise_;
    };


}  // namespace legged

#endif
