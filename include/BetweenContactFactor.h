#ifndef BETWEENCONTACT_FACTOR
#define BETWEENCONTACT_FACTOR

#include <vector>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "ContactUtils.h"

namespace gtsam {

    template<class POSE>
    class BetweenPointContactFactor : public NoiseModelFactor4<POSE, POSE, POSE, POSE> {

        public:
            typedef POSE T;

        private:
            typedef BetweenPointContactFactor<POSE> This;
            typedef NoiseModelFactor4<POSE, POSE, POSE, POSE> Base;

        public:
            typedef typename boost::shared_ptr<BetweenPointContactFactor> shared_ptr;

            // Serialization
            BetweenPointContactFactor() {}

            BetweenPointContactFactor(const Key& base1, const Key& base2, const Key& contact1,
            const Key& contact2, const SharedNoiseModel& model) :
            Base(model, base1, base2, contact1, contact2) {}

            virtual ~BetweenPointContactFactor() {}

            // Return deep copy by constructing a shared pointer 
            gtsam::NonlinearFactor::shared_ptr clone() const overrider {
                return boost::static_pointer_cast<gtsam::NonlinearFactor> (
                    gtsam::NonlinearFactor::shared_ptr(new This(*this));
                )
            }

            void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
                // Stream to s, etc
                // TODO:
            }

            bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
                // Cast to the same type of pointer
                // TODO: Properly implement this
                // const This *e = dynamic_cast<const This*> (&expected);
                // return e != NULL && Base::equals(*e. tol) && this->measured_.equals(e->measured_, tol);
            }

            typedef Eigen::Matrix<double, 3, 1> Vector3;
            Vector evaluateError(const Pose3& posei, const Pose3& posej, const Pose3& contacti, const Pose3& contactj,
                boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none,
                boost::optional<Matrix&> H3 = boost::none, boost::optional<Matrix&> H4 = boost::none) const override {

                Vector error = posei.rotation().transpose() * (contactj.translation() - contacti.translation());

                // Potentially I have to swap 
                if (H1) {
                    *H1 = Matrix::zeros(3, 6);
                    H1->block(0, 3, 3, 3) = (Matrix(3, 3) << 0, -error(2), error(1),  error(2), 0, -error(0), -error(1), error(0), 0).finished();
                }

                if (H2) {
                    *H2 = Matrix::zeros(3, 6);
                }

                if (H3) {
                    *H3 = Matrix::zeros(3, 6);
                    H3->block(0, 0, 3, 3) = -posei.rotation().transpose() * contacti.rotation();
                }

                if (H4) {
                    *H4 = Matrix::zeros(3, 6);
                    H4->block(0, 0, 3, 3) = -posei.rotation().transpose() * contactj.rotation();
                }

                return error;
            }

            // const VALUE& measured() const {
            //     return measured_;
            // }

            std::size_t size() const {
                return 4;
            }



        private:
            // Serialization
            // TODO:
 
    };
}

#endif