#ifndef IMU_H
#define IMU_H

#include <vector>
#include <iostream>
// #include<utility>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

namespace ORB_SLAM2{
    // Imu Measurement
    class ImuMeasure {
        public:
            ImuMeasure( const float& a_x,
                        const float& a_y,
                        const float& a_z,
                        const float& ang_vel_x,
                        const float& ang_vel_y,
                        const float& ang_vel_z,
                        const double& ts):
            _a(a_x, a_y, a_z), _w(ang_vel_x, ang_vel_y, ang_vel_z),
            _ts(ts) {};
            
            ImuMeasure(const cv::Point3f a, const cv::Point3f g, const double& ts):
            _a(a.x, a.y, a.z), _w(g.x, g.y, g.z), _ts(ts){};

            friend std::ostream & operator << (std::ostream & os, const ImuMeasure & );
        public:
            cv::Point3f _a;
            cv::Point3f _w;
            double _ts;
    };

    class Bias {
        // Contains
        public:   
            Bias():bax(0),bay(0),baz(0),bwx(0),bwy(0),bwz(0){}
            Bias(const float &b_acc_x, const float &b_acc_y, const float &b_acc_z,
                const float &b_ang_vel_x, const float &b_ang_vel_y, const float &b_ang_vel_z):
                bax(b_acc_x), bay(b_acc_y), baz(b_acc_z), bwx(b_ang_vel_x), bwy(b_ang_vel_y), bwz(b_ang_vel_z){};
            friend std::ostream & operator << (std::ostream & os, const Bias & );

        public:
            float bax, bay, baz;
            float bwx, bwy, bwz;
    };

    class Calib {
        // Calibration, from my understanding, is effectively location from camera to imu (?)
        public:
            Calib();
            Calib(const Calib &calib);
            Calib(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw)
            {
                setCalib(Tbc_,ng,na,ngw,naw);
            }

            void setCalib(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw);

        public:
            cv::Mat Tcb;
            cv::Mat Tbc;
            cv::Mat Cov, CovWalk;
    };


    class PreintegratedIMU {
        public: 
            // Constructor
            PreintegratedIMU() {};
            PreintegratedIMU(Bias b_, Calib c_) : b(b_), c(c_) {};

            friend std::ostream & operator << (std::ostream & os, const PreintegratedIMU & );

            void initialize();
            // Integrate Measurement
            void integrateMeasurement(const cv::Point3f a, const cv::Point3f w, float dt);
            void mergePrior(const PreintegratedIMU&);
            void updateBias(const Bias&);
            cv::Mat getMeasurementNoise();

            // This should be used when the Gravity measurement / pose estimation is available initially
            // So that the gravity vector can be properly taken from the IMU
            void setInitialState(cv::Mat R_, cv::Mat p, cv::Mat v, float ts);

            // Getter and Setters
            void setKeyFrameTS(float ts) {currentTimestamp = ts;}
            void setGravity(cv::Mat g) {g_w = g;};
            double getTS();
            cv::Mat getRotation();
            cv::Mat getTranslation();

            // Get transformation between last to current
            cv::Mat getDeltaR();
            cv::Mat getDeltaP();
            cv::Mat getDeltaV();
            
            // Utility Functions
            cv::Mat ExpMap(cv::Mat angV, float t);
            cv::Mat convertRotm2Quat(cv::Mat) const;
            cv::Mat convertQuat2Rotm(cv::Mat) const;

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
            
            float currentTimestamp = 0;
            float lastTimestamp = 0;
            cv::Mat g_w;
            Bias b;
            Calib c;

            void updateDeltaTransformation();
    };

    cv::Mat rightJacobian(cv::Mat phi);
    cv::Mat hatOperator(cv::Mat);

}

#endif



