/******************************************************************************
 * File:             imu.h
 *
 * Author:           Ruoyang Xu & Akash Sharma
 * Created:          05/08/21
 * Description:
 *****************************************************************************/

#ifndef LEGGED_ESTIMATION_IMU_H
#define LEGGED_ESTIMATION_IMU_H

#include <sstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace legged
{
    class ImuMeasurement
    {
       public:
        ImuMeasurement(const double &acc_x,
                       const double &acc_y,
                       const double &acc_z,
                       const double &ang_vel_x,
                       const double &ang_vel_y,
                       const double &ang_vel_z,
                       const double &timestamp)
            : acc_(acc_x, acc_y, acc_z), omega_(ang_vel_x, ang_vel_y, ang_vel_z), timestamp_(timestamp){};

        ImuMeasurement(const Eigen::Vector3d &acc, const Eigen::Vector3d &omega, const double &timestamp)
            : acc_(a.x, a.y, a.z), omega_(omega.x, omega.y, omega.z), timestamp_(timestamp){};

        ImuMeasurement(const cv::Point3d &acc, const cv::Point3d &omega, const double &timestamp)
            : acc_(acc.x, acc.y, acc.z), omega_(omega.x, omega.y, omega.z), timestamp_(timestamp){};

        inline std::string print() const
        {
            std::stringstream message;
            message << "Acceleration: " << acc_ << " Angular Velocity: " << omega_;
            return message.str();
        }

       public:
        Eigen::Vector3d acc_;
        Eigen::Vector3d omega_;
        double timestamp_;
    };

    class ImuBias
    {
       public:
        ImuBias() : bias_acc_(Eigen::Vector3d::Zero()), biad_omega_(Eigen::Vector3d::Zero()) {}

        ImuBias(const float &b_acc_x,
                const float &b_acc_y,
                const float &b_acc_z,
                const float &b_ang_vel_x,
                const float &b_ang_vel_y,
                const float &b_ang_vel_z)
            : bias_acc_(b_acc_x, b_acc_y, b_acc_z), bias_omega_(b_ang_vel_x, b_ang_vel_y, b_ang_vel_z){};

        inline void setImuBias(const Eigen::Vector3d &bias_acc, const Eigen::Vector3d &bias_omega)
        {
            bias_acc_   = bias_acc;
            bias_omega_ = bias_omega;
        }

        inline Eigen::Vector<double, 6> Bias::getBiasVector() const
        {
            Eigen::VectorXd b;
            b << bias_acc_ << bias_omega_;
            return b;
        }

        inline Eigen::Matrix<double, 3, 1> Bias::getAccelBias() const { return bias_acc_; }

        inline Eigen::Vector3d Bias::getAngVelBias() const { return bias_omega_; }

        inline std::string print() const
        {
            std::stringstream message;
            message << "Accel Bias: " << bias_acc_ << " Omega Bias: " << bias_omega_;
            return message.str();
        }

       private:
        Eigen::Vector3d bias_acc_;
        Eigen::Vector3d bias_omega_;
    };

    class IMUPosition
    {
       public:
        IMUPosition(){};
        IMUPosition(const Eigen::Vector3d &p) : pos_(p){};

        Eigen::Vector3d getImuPosition() const { return pos_; }
        void setPos(const double px, const double py, const double pz) { pos_(px, py, pz); }

       public:
        Eigen::Vector3d pos_;
    };

    class IMUVel
    {
       public:
        IMUVel(){};
        IMUVel(const Eigen::Vector3d &v) : vx(v(0)), vy(v(1)), vz(v(2)){};

        Eigen::Matrix<double, 6, 1> getVelVec();
        void setVel(const double _vx, const double _vy, const double _vz);
        void setVel(const Eigen::Vector3d &v);

       public:
        double vx, vy, vz;
    };

    class Calib
    {
        // Calibration, from my understanding, is effectively location from camera to imu (?)
       public:
        Calib();
        Calib(const Calib &calib);
        Calib(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw)
        {
            setCalib(Tbc_, ng, na, ngw, naw);
        }

        void setCalib(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw);

       public:
        cv::Mat Tcb;
        cv::Mat Tbc;
        cv::Mat Cov, CovWalk;
    };

    class PreintegratedIMU
    {
       public:
        // Constructor
        PreintegratedIMU() { setup = false; };
        PreintegratedIMU(const PreintegratedIMU &);
        PreintegratedIMU(ImuBias b_, Calib c_) : b(b_), c(c_) { initialize(); };
        void copyFrom(const PreintegratedIMU &);

        PreintegratedIMU &operator=(const PreintegratedIMU &other);
        friend std::ostream &operator<<(std::ostream &os, const PreintegratedIMU &);

        void initialize();
        // Integrate Measurement
        void integrateMeasurement(const cv::Point3d a, const cv::Point3d w, double dt);
        void mergePrior(const PreintegratedIMU &);
        void updateImuBias(const ImuBias &);
        cv::Mat getMeasurementNoise();

        void updateParentKFDeleted(const PreintegratedIMU &);
        void updateChildrenKFDeleted(const PreintegratedIMU &);

        void resetIntegrationAndSetImuBias(ImuBias b);

        // This should be used when the Gravity measurement / pose estimation is available initially
        // So that the gravity vector can be properly taken from the IMU
        void setInitialState(cv::Mat R_, cv::Mat p, cv::Mat v, double ts);

        // Getter and Setters
        void setKeyFrameTS(double ts) { currentTimestamp = ts; }
        void setGravity(cv::Mat g) { g_w = g.clone(); }
        void setImuBias(ImuBias b_) { b = b_; }  // This does not invoke measurement update through jacobian
        void setlastFrameFinalTs(double ts) { lastFrameFinalTs = ts; };

        double getTS();
        cv::Mat getRotation();
        cv::Mat getTranslation();
        cv::Mat getVelocity();

        // Get transformation between last to current
        cv::Mat getDeltaR();
        cv::Mat getDeltaP();
        cv::Mat getDeltaV();
        double getDeltaTime() { return currentTimestamp - lastTimestamp; }

        // Utility Functions
        cv::Mat ExpMap(cv::Mat angV, double t);
        cv::Mat convertRotm2Quat(cv::Mat) const;
        cv::Mat convertQuat2Rotm(cv::Mat) const;

        cv::Mat getJRg() { return JRg.clone(); }
        cv::Mat getJva() { return Jva.clone(); }
        cv::Mat getJvg() { return Jvg.clone(); }
        cv::Mat getJpa() { return Jpa.clone(); }
        cv::Mat getJpg() { return Jpg.clone(); }
        cv::Mat getg_w() { return g_w.clone(); }
        void updateDeltaTransformation();

        Eigen::Matrix<double, 6, 6> getSR();  // get random walk mat
        Eigen::Matrix<double, 9, 9> getSI();  // get info mat

       public:
        double lastFrameFinalTs = 0;
        double currentTimestamp = 0;
        double lastTimestamp    = 0;
        std::vector<ImuMeasurement> imuLog;
        ImuBias b;
        bool setup = true;

       private:
        // w -> b; World to Body
        cv::Mat Rwb, wVb, wPb;
        // l -> c; last to current
        cv::Mat Rlc, lVc, lPc;
        // w -> l; world to last
        cv::Mat Rwl, wVl, wPl;

        // Jacobians
        // Jacobian of {Rotation, Velocity, Position} relative to {Gyro, acce} bias
        cv::Mat JRg, Jva, Jvg, Jpa, Jpg;
        // Noise
        cv::Mat nphi, nv, np;

        cv::Mat g_w;
        Calib c;
    };

    cv::Mat rightJacobian(cv::Mat phi);
    cv::Mat hatOperator(cv::Mat);
    cv::Mat ExpMap(cv::Mat angV, double t);

}  // namespace legged

#endif /* ifndef LEGGED_ESTIMATION_IMU_H */
