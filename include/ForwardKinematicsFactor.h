/******************************************************************************
* File:             ForwardKinematicsFactor.h
*
* Author:           Akash Sharma
* Created:          05/05/21
* Description:      Implementation of Forward Kinematics Factor for legged robots
*****************************************************************************/
#ifndef FORWARD_KINEMATICS_FACTOR_H
#define FORWARD_KINEMATICS_FACTOR_H

#include <gtsam/base/Lie.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <vector>
#include "ContactUtils.h"

namespace gtsam
{
    template<class POSE>
    class ForwardKinematicsFactor : public NoiseModelFactor2<POSE, POSE>
    {
       public:
        // typedef POSE T;

       private:
        typedef ForwardKinematicsFactor<POSE> This;
        typedef NoiseModelFactor2<POSE, POSE> Base;

       public:
        typedef typename boost::shared_ptr<ForwardKinematicsFactor> shared_ptr;

        // Serialization
        ForwardKinematicsFactor() {}

        ForwardKinematicsFactor(Matrix covariance, Key base_frame, Key contact_frame, const POSE& measured)
            : Base(noiseModel::Gaussian::Covariance(covariance), base1, base2, contact1, contact2)
        {
        }

        virtual ~ForwardKinematicsFactor() {}

        // Return deep copy by constructing a shared pointer
        gtsam::NonlinearFactor::shared_ptr clone() const override
        {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override
        {
            // Stream to s, etc
            // TODO:
            std::cout << s << "ForwardKinematicsFactor(" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) ")\n";
            this->noiseModel_->print(" noise model:  ");
        }

        bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override
        {
            // Cast to the same type of pointer
            // 1. Pointer not null
            // 2. The same
            // const This *e = dynamic_cast<const This*> (&expected);
            // return e != NULL && Base::equals(*e. tol) && this->measured_.equals(e->measured_, tol);
            const This* e = dynamic_cast<const This*>(&expected);
            // No measurement since, there's no measurement
            return e != NULL && Base::equals(*e, tol);
        }

        typedef Eigen::Matrix<double, 3, 1> Vector3;
        Vector evaluateError(const POSE& p1,
                             const POSE& p2,
                             boost::optional<Matrix&> H1 = boost::none,
                             boost::optional<Matrix&> H2 = boost::none) const override
        {


            //! R_i^\top C_i and R_i^\top (d_i - p_i)
            POSE hx = p1.between(p2, H1, H2);

            measured_
            Vector error = posei.rotation().transpose() * (contactj.translation() - contacti.translation());

            // Rx Ry Rz, vx vy vz.
            // Potentially to swap, based on w and v locations
            if (H1)
            {
                *H1 = Matrix::Zero(3, 6);
                H1->block(0, 0, 3, 3) =
                    (Matrix(3, 3) << 0, -error(2), error(1), error(2), 0, -error(0), -error(1), error(0), 0).finished();
            }

            if (H2)
            {
                *H2 = Matrix::Zero(3, 6);
            }

            if (H3)
            {
                *H3                   = Matrix::Zero(3, 6);
                H3->block(0, 3, 3, 3) = -posei.rotation().transpose() * contacti.rotation().matrix();
            }

            if (H4)
            {
                *H4                   = Matrix::Zero(3, 6);
                H4->block(0, 3, 3, 3) = -posei.rotation().transpose() * contactj.rotation().matrix();
            }

            return error;
        }

        const POSE& measured() const {
             return measured_;
        }

        std::size_t size() const { return 4; }

       private:
        // Serialization
        // TODO:
    };
}  // namespace gtsam

#endif

#endif /* ifndef FORWARD_KINEMATICS_FACTOR_H */
