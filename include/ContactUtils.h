#ifndef POINTCONTACTFACTOR_H
#define POINTCONTACTFACTOR_H

#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>


namespace gtsam {

    Matrix Expmap(Vector xi, double theta) {
        // Xi: Twist w {x,y,z}; v {x,y,z}
        // For gtsam::Pose3 it seems it's wx, wy, wz, vx, vy, vz
        auto omega = (Eigen::Vector3d << xi(0), xi(1), xi(2)).finished();
        auto v = (Eigen::Vector3d << xi(3), xi(4), xi(5)).finished();
        Matrix omega_hat = (Matrix(3, 3) << 0,-xi(2),xi(1), xi(2),0,-xi(0), -xi(1),xi(0),0).finished();
        Matrix R = Matrix::Identity(3, 3) + omega_hat * sin(theta) + omega_hat * omega_hat * (1 - cos(t));
        Vector3 t = (Matrix::Identity(3, 3) - R) * omega.cross(v);
        
        // R of Type Rot, t of type Point3, which is of type Vector3
        // Pose3 g(Rot3(R), t);
        Matrix g = Matrix::Identity(4, 4);
        g.block(0, 0, 3, 3) = R;
        g.block(0, 3, 3, 1) = t

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
            Matrix endEffectorConfiguration0;
            Matrix covSlip;
    };

    class LegMeasurement {
        public:
            LegMeasurement() {}
            LegMeasurement(LegConfig leg_, Pose3 base, Pose3 contact, double dt) {
                leg = leg_;
                Matrix B = base.rotation().transpose() * contact.rotation().transpose() * dt;
                measureNoise = B * leg.covSlip * B.transpose();
            }

            ~LegMeasurement() {}

            Matrix efInBase(Vector encoder) {
                auto g = leg.firstJointInBase;
                for (int i = 0; i < encoder.size(); i++) {
                    g = g * Expmap(leg.xi.at(i), encoder(i));
                }
                g *= leg.endEffectorConfiguration0;
                return g;
            }

            // Update Covariance
            // Pose3 iGj i -> j, Pose J in frame of Pose i
            void integrateNewMeasurement(Pose3 iGj, Vector encoder, double dt) {
                Matrix B = iGj.rotation() * efInBase(encoder).block(0, 0, 3, 3) * dt;
                measureNoise += B * leg.covSlip * B.transpose();
            }

        public:
            LegConfig leg;
            Matrix measureNoise;
    };
}

#endif