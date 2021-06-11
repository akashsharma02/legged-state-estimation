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
        LegConfig(const std::string& leg_name,
                  const std::vector<gtsam::Vector6>& twists,
                  gtsam::Pose3 base_T_hip,
                  gtsam::Pose3 eff_zero_config,
                  gtsam::Matrix33 slip_covariance,
                  gtsam::Matrix33 encoder_covariance)
            : leg_name_(leg_name),
              twists_(twists),
              base_T_hip_(base_T_hip),
              eff_zero_config_(eff_zero_config),
              slip_covariance_(slip_covariance),
              encoder_covariance_(encoder_covariance)

        {
        }
        ~LegConfig() {}

        inline gtsam::Matrix33 getSlipCovariance() const { return slip_covariance_; }
        inline gtsam::Matrix33 getEncoderCovariance() const { return encoder_covariance_; }
        inline gtsam::Pose3 getBaseTHip() const { return base_T_hip_; }
        inline gtsam::Pose3 getBaseTContactZeroConfig() const { return base_T_hip_ * eff_zero_config_; }
        inline std::vector<gtsam::Vector6> getTwists() const { return twists_; }
        inline gtsam::Pose3 getHipTContactZeroConfig() const { return eff_zero_config_; }

       public:
        std::string leg_name_;
        std::vector<gtsam::Vector6> twists_;
        gtsam::Pose3 base_T_hip_;
        gtsam::Pose3 eff_zero_config_;
        gtsam::Matrix33 slip_covariance_;
        gtsam::Matrix33 encoder_covariance_;
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

    struct LegEncoderMeasurements
    {
        gtsam::Vector3 frontleft;
        gtsam::Vector3 frontright;
        gtsam::Vector3 backleft;
        gtsam::Vector3 backright;
    };

    struct LegPoseMeasurements
    {
        gtsam::Pose3 frontleft;
        gtsam::Pose3 frontright;
        gtsam::Pose3 backleft;
        gtsam::Pose3 backright;
    };

    /*! \class LegMeasurement
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class LegMeasurement
    {
       public:
        typedef std::shared_ptr<LegMeasurement> shared_ptr;

        LegMeasurement(bool contact, const gtsam::Vector3& encoder_measurement, const gtsam::Pose3& contact_pose_measurement)
            : contact_(contact), encoder_meas_(encoder_measurement), contact_pose_meas_(contact_pose_measurement)
        {
        }
        virtual ~LegMeasurement(){};

        inline bool getContactState() const { return contact_; }
        inline gtsam::Vector3 getEncoderMeasurement() const { return encoder_meas_; }
        inline gtsam::Pose3 getContactPoseMeasurement() const { return contact_pose_meas_; }

        gtsam::Pose3 getbaseTContactFromEncoder(const LegConfig& leg_config) const
        {
            //! g_st(0)
            gtsam::Pose3 base_T_contact = leg_config.getBaseTContactZeroConfig();
            for (size_t i = static_cast<size_t>(encoder_meas_.size() - 1); i >= 0; i--)
            {
                auto twist = leg_config.getTwists().at(i) * encoder_meas_(static_cast<Eigen::Index>(i));
                //! g_st(theta) = exp(\xi_n \alpha_n) * g_st(0)
                base_T_contact = gtsam::Pose3::Expmap(twist) * base_T_contact;
            }

            return base_T_contact;
        }

        gtsam::Matrix63 getBaseTContactJacobian(const LegConfig& leg_config) const
        {
            //! This is the body manipulator jacobian of the leg
            gtsam::Matrix63 J = gtsam::Matrix::Zero(6, 3);

            //! g_st(0)
            gtsam::Pose3 base_T_contact = leg_config.getBaseTContactZeroConfig();
            size_t n                    = static_cast<size_t>(encoder_meas_.size());
            for (size_t i = n - 1; i >= 0; i--)
            {
                auto xi                             = leg_config.getTwists().at(i);
                auto twist                          = xi * encoder_meas_(static_cast<Eigen::Index>(i));
                base_T_contact                      = gtsam::Pose3::Expmap(twist) * base_T_contact;
                auto contact_T_base                 = base_T_contact.inverse();
                J.col(static_cast<Eigen::Index>(i)) = contact_T_base.Adjoint(xi);
            }
            return J;
        }

       private:
        bool contact_;
        gtsam::Vector3 encoder_meas_;
        gtsam::Pose3 contact_pose_meas_;
    };

    using LegMeasurements = std::array<LegMeasurement::shared_ptr, 4>;

    /*! \struct ContactStates
     *  \brief Maintains the keys of the frames that participate in a make-break contact during robot gait
     *
     */
    struct ContactStates
    {
        ContactStates(const gtsam::Key& base_make_contact_frame,
                      const gtsam::Key& foot_make_contact_frame,
                      const gtsam::Key& foot_break_contact_frame)
            : base_make_contact_frame_(base_make_contact_frame),
              foot_make_contact_frame_(foot_make_contact_frame),
              foot_break_contact_frame_(foot_break_contact_frame){};

        virtual ~ContactStates();

        gtsam::Key base_make_contact_frame_;
        gtsam::Key foot_make_contact_frame_;
        gtsam::Key foot_break_contact_frame_;
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

        virtual ~PreintegratedContactMeasurement(){};

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
            gtsam::Pose3 contact_T_base = getBaseTContactFromEncoder(leg_encoder_reading).inverse();

            //! \delta \tilde {R_{ik}} f_R(\tilde{\alpha_k}) \delta t
            gtsam::Matrix33 B = relative_imu_pim.matrix() * contact_T_base.rotation().matrix() * imu_rate_;

            measure_noise_ += B * leg_.getSlipCovariance() * B.transpose();
        }

       protected:
        inline gtsam::Pose3 getBaseTContactFromEncoder(const gtsam::Vector3& leg_encoder_reading)
        {
            gtsam::Pose3 base_T_contact = leg_.getBaseTJoint();
            for (size_t i = 0; i < static_cast<size_t>(leg_encoder_reading.size()); i++)
            {
                auto twist     = leg_.getTwists().at(i) * leg_encoder_reading(static_cast<Eigen::Index>(i));
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
