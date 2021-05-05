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
            ImuMeasure( const double& a_x,
                        const double& a_y,
                        const double& a_z,
                        const double& ang_vel_x,
                        const double& ang_vel_y,
                        const double& ang_vel_z,
                        const double& ts):
            _a(a_x, a_y, a_z), _w(ang_vel_x, ang_vel_y, ang_vel_z),
            _ts(ts) {};

            ImuMeasure(const cv::Point3d a, const cv::Point3d g, const double& ts):
            _a(a.x, a.y, a.z), _w(g.x, g.y, g.z), _ts(ts){};

            ImuMeasure(const cv::Point3d a, const cv::Point3d g, const double& dt, bool dt_true):
            _a(a.x, a.y, a.z), _w(g.x, g.y, g.z), _dt(dt){};

            friend std::ostream & operator << (std::ostream & os, const ImuMeasure & );
        public:
            cv::Point3d _a;
            cv::Point3d _w;
            double _ts;
            double _dt;
    };

    class Bias {
        // Contains
        public:   
            Bias():bax(0),bay(0),baz(0),bwx(0),bwy(0),bwz(0){}
            Bias(const float &b_acc_x, const float &b_acc_y, const float &b_acc_z,
                const float &b_ang_vel_x, const float &b_ang_vel_y, const float &b_ang_vel_z):
                bax(b_acc_x), bay(b_acc_y), baz(b_acc_z), bwx(b_ang_vel_x), bwy(b_ang_vel_y), bwz(b_ang_vel_z){};
            friend std::ostream & operator << (std::ostream & os, const Bias & );

            void setBiasVec(const Eigen::Matrix<double, 6, 1> &vec);
            Eigen::Matrix<double, 6, 1> getBiasVec();
            Eigen::Matrix<double, 3, 1> getGyroVec();
            Eigen::Matrix<double, 3, 1> getAccelVec();

        public:
            float bax, bay, baz;
            float bwx, bwy, bwz;
    };

    class IMUPos {
        public:
            IMUPos(){};
            IMUPos(const Eigen::Vector3d &p)
            : px(p(0)), py(p(1)), pz(p(2)){};
            
            Eigen::Matrix<double, 6, 1> getPosVec();
            void setPos(const double _px, const double _py, const double _pz)
            {px = _px; py = _py; pz = _pz;};
            void setPos(const Eigen::Vector3d &p);

        public:
            double px, py, pz;
    };

    class IMUVel {
        public:
            IMUVel(){};
            IMUVel(const Eigen::Vector3d &v)
            : vx(v(0)), vy(v(1)), vz(v(2)){};

            Eigen::Matrix<double, 6, 1> getVelVec();
            void setVel(const double _vx, const double _vy, const double _vz);
            void setVel(const Eigen::Vector3d &v);

        public:
            double vx, vy, vz;
    };

    class IMUAcc {
        public:
            IMUAcc(){};
            IMUAcc(const Eigen::Vector3d &a)
            : ax(a(0)), ay(a(1)), az(a(2)){};
            
            Eigen::Matrix<double, 6, 1> getAccVec();
            void setAcc(const double _ax, const double _ay, const double _az);
            void setAcc(const Eigen::Vector3d &a);
            
        public:
            double ax, ay, az;
    };

    class IMURot {
        public:
            IMURot(){};
            IMURot(const Eigen::Matrix3d &R)
            : rot(R){};

            Eigen::Matrix3d getRotMat();
            void setRot(const Eigen::Matrix3d &R);

        public:
            Eigen::Matrix3d rot;
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
            PreintegratedIMU() {setup = false;};
            PreintegratedIMU(const PreintegratedIMU&);
            PreintegratedIMU(Bias b_, Calib c_) : b(b_), c(c_) {initialize();};
            void copyFrom(const PreintegratedIMU&);

            PreintegratedIMU & operator=(const PreintegratedIMU & other);
            friend std::ostream & operator << (std::ostream & os, const PreintegratedIMU & );

            void initialize();
            // Integrate Measurement
            void integrateMeasurement(const cv::Point3d a, const cv::Point3d w, double dt);
            void mergePrior(const PreintegratedIMU&);
            void updateBias(const Bias&);
            cv::Mat getMeasurementNoise();

            void updateParentKFDeleted(const PreintegratedIMU&);
            void updateChildrenKFDeleted(const PreintegratedIMU&);

            void resetIntegrationAndSetBias(Bias b);

            // This should be used when the Gravity measurement / pose estimation is available initially
            // So that the gravity vector can be properly taken from the IMU
            void setInitialState(cv::Mat R_, cv::Mat p, cv::Mat v, double ts);
            
            // Getter and Setters
            void setKeyFrameTS(double ts) {currentTimestamp = ts;}
            void setGravity(cv::Mat g) {g_w = g.clone();}
            void setBias(Bias b_) {b = b_;} // This does not invoke measurement update through jacobian
            void setlastFrameFinalTs(double ts) {lastFrameFinalTs = ts;};

            double getTS();
            cv::Mat getRotation();
            cv::Mat getTranslation();
            cv::Mat getVelocity();

            // Get transformation between last to current
            cv::Mat getDeltaR();
            cv::Mat getDeltaP();
            cv::Mat getDeltaV();
            double getDeltaTime() {return currentTimestamp - lastTimestamp;}
            
            // Utility Functions
            cv::Mat ExpMap(cv::Mat angV, double t);
            cv::Mat convertRotm2Quat(cv::Mat) const;
            cv::Mat convertQuat2Rotm(cv::Mat) const;

            cv::Mat getJRg() {return JRg.clone();}
            cv::Mat getJva() {return Jva.clone();}
            cv::Mat getJvg() {return Jvg.clone();}
            cv::Mat getJpa() {return Jpa.clone();}
            cv::Mat getJpg() {return Jpg.clone();}
            cv::Mat getg_w() {return g_w.clone();}
            void updateDeltaTransformation();

            Eigen::Matrix<double, 6, 6> getSR();    // get random walk mat
            Eigen::Matrix<double, 9, 9> getSI();    // get info mat

        public:
            double lastFrameFinalTs = 0;
            double currentTimestamp = 0;
            double lastTimestamp = 0;
            std::vector<ImuMeasure> imuLog;
            Bias b;
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

} // ORB_SLAM2

#endif



