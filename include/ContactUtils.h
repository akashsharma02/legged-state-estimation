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
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include "opencv2/core/core.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

namespace gtsam {

    Matrix customExpmap(Vector xi, double theta) {
        // Xi: Twist w {x,y,z}; v {x,y,z}
        // For gtsam::Pose3 it seems it's wx, wy, wz, vx, vy, vz
        auto omega = (Eigen::Vector3d() << xi(0), xi(1), xi(2)).finished();
        auto v = (Eigen::Vector3d() << xi(3), xi(4), xi(5)).finished();
        Matrix omega_hat = (Matrix(3, 3) << 0,-xi(2),xi(1), xi(2),0,-xi(0), -xi(1),xi(0),0).finished();
        Matrix R = Matrix::Identity(3, 3) + omega_hat * sin(theta) + omega_hat * omega_hat * (1 - cos(theta));
        Vector3 t = (Matrix::Identity(3, 3) - R) * omega.cross(v);
        
        // R of Type Rot, t of type Point3, which is of type Vector3
        // Pose3 g(Rot3(R), t);
        Matrix g = Matrix::Identity(4, 4);
        g.block(0, 0, 3, 3) = R;
        g.block(0, 3, 3, 1) = t;
        return g;
    }

    class LegConfig {
        public:
            LegConfig() {}
            LegConfig(std::vector<Vector> xi_, Matrix fiB, Matrix efC, Matrix covS)
                : xi(xi_), firstJointInBase(fiB), endEffectorConfiguration0(efC), covSlip(covS) {}
            ~LegConfig() {}

        public: 
            std::vector<Vector> xi;
            Matrix firstJointInBase; // Joint in Frame of Base
            Matrix jTul; // Upper leg
            Matrix ulTll; // Upper Leg Lower leg
            Matrix llTf; // Lower Leg Foot
            Matrix endEffectorConfiguration0;
            Matrix covSlip;
    };

    class LegMeasurement {
        public:
            LegMeasurement() {}
            LegMeasurement(LegConfig leg_, Pose3 base, Pose3 contact, double dt, double ts) {
                leg = leg_;
                Matrix B = base.rotation().transpose() * contact.rotation().transpose() * dt;
                measureNoise = B * leg.covSlip * B.transpose();
                ts_ = ts;
                makeContact = contact;
            }

            ~LegMeasurement() {}

            Matrix efInBaseExpMap(Vector encoder) {
                auto g = leg.firstJointInBase;
                for (auto i = 0; i < encoder.size(); i++) {
                    g = g * customExpmap(leg.xi.at(size_t(i)), encoder(i));
                }
                g *= leg.endEffectorConfiguration0;
                return g;
            }

            static Matrix efInBase(Vector encoder, const LegConfig& leg) {
                auto g = leg.firstJointInBase;
                g = g * gtsam::Pose3(gtsam::Rot3::Rx(encoder(0)), gtsam::Point3()).matrix();
                g *= leg.jTul;
                g *= gtsam::Pose3(gtsam::Rot3::Ry(encoder(1)), gtsam::Point3()).matrix();
                g *= leg.ulTll;
                g *= gtsam::Pose3(gtsam::Rot3::Ry(encoder(2)), gtsam::Point3()).matrix();
                g *= leg.llTf;
                return g;
            }

            // Update Covariance
            // Pose3 iGj i -> j, Pose J in frame of Pose i
            // Obtained through IMU estimation
            void integrateNewMeasurement(Rot3 iGj, Vector encoder, double ts) {
                double dt = ts - ts_;
                Matrix B = iGj.matrix() * efInBase(encoder, leg).block(0, 0, 3, 3) * dt;
                measureNoise += B * leg.covSlip * B.transpose();
            }

            Matrix getNoise() {return measureNoise;};

        public:
            LegConfig leg;
            Matrix measureNoise;
            double ts_;
            Pose3 makeContact;
    };

    struct ContactStates
    {
        uint64_t baseMakeContact;
        uint64_t baseBreakContact;
        uint64_t contactMakeContact;
        uint64_t contactBreakContact;
    };

    Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3) {
        Eigen::Matrix<double,3,3> M;

        M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
            cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
            cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

        return M;
    }

    inline Point3 toPoint3(const cv::Mat &cvMat) {
        Point3 p(cvMat.at<double>(0, 0), cvMat.at<double>(1, 0), cvMat.at<double>(2, 0));
        return p;
    }
    
}

#endif