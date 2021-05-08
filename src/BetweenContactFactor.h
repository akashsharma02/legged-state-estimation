/******************************************************************************
 * File:             BetweenContactFactor.h
 *
 * Author:           Ruoyang Xu
 * Created:          04/27/2021
 * Description:      Implementation of Contact Factor
 *****************************************************************************/

#ifndef BETWEENCONTACT_FACTOR
#define BETWEENCONTACT_FACTOR

#include <gtsam/base/Lie.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <vector>

#include "leg-utils.h"

namespace gtsam
{
    template<class POSE>
    class BetweenPointContactFactor : public NoiseModelFactor4<POSE, POSE, POSE, POSE>
    {
       public:
        // typedef POSE T;

       private:
        typedef BetweenPointContactFactor<POSE> This;
        typedef NoiseModelFactor4<POSE, POSE, POSE, POSE> Base;

       public:
        typedef typename boost::shared_ptr<BetweenPointContactFactor> shared_ptr;

        // Serialization
        BetweenPointContactFactor() {}

        BetweenPointContactFactor(Matrix covariance, Key base1, Key base2, Key contact1, Key contact2, const POSE& measured)
            : Base(noiseModel::Gaussian::Covariance(covariance), base1, base2, contact1, contact2)
        {
        }

        virtual ~BetweenPointContactFactor() {}

        // Return deep copy by constructing a shared pointer
        gtsam::NonlinearFactor::shared_ptr clone() const override
        {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override
        {
            // Stream to s, etc
            // TODO:
            std::cout << s << "BetweenPointContactFactor(" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2())
                      << "," << keyFormatter(this->key3()) << "," << keyFormatter(this->key4()) << ")\n";
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
        Vector evaluateError(const Pose3& posei,
                             const Pose3& posej,
                             const Pose3& contacti,
                             const Pose3& contactj,
                             boost::optional<Matrix&> H1 = boost::none,
                             boost::optional<Matrix&> H2 = boost::none,
                             boost::optional<Matrix&> H3 = boost::none,
                             boost::optional<Matrix&> H4 = boost::none) const override
        {
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

        // const VALUE& measured() const {
        //     return measured_;
        // }

        std::size_t size() const { return 4; }

       private:
        // Serialization
        // TODO:
    };
}  // namespace gtsam

#endif
