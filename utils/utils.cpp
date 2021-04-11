/******************************************************************************
 * File:             utils.cpp
 *
 * Author:           Akash Sharma
 * Created:          03/07/21
 * Description:      Common utility functions
 *****************************************************************************/
#include "utils.h"
#include <fstream>
#include <iostream>

namespace utils
{

    void writeMultiRobotG2o(const gtsam::NonlinearFactorGraph &graph,
                            const gtsam::Values &estimate,
                            const std::string &filename)
    {
        using namespace gtsam;
        using namespace std;

        fstream stream(filename.c_str(), fstream::out);

        /* // Use a lambda here to more easily modify behavior in future. */
        /* auto index = [](gtsam::Key key) { return Symbol(key).index(); }; */

        // save 2D poses
        for (const auto key_value : estimate)
        {
            auto p = dynamic_cast<const GenericValue<Pose2> *>(&key_value.value);
            if (!p)
                continue;
            const Pose2 &pose = p->value();
            stream << "VERTEX_SE2 " << key_value.key << " " << pose.x() << " " << pose.y() << " " << pose.theta() << endl;
        }

        // save 3D poses
        for (const auto key_value : estimate)
        {
            auto p = dynamic_cast<const GenericValue<Pose3> *>(&key_value.value);
            if (!p)
                continue;
            const Pose3 &pose = p->value();
            const Point3 t    = pose.translation();
            const auto q      = pose.rotation().toQuaternion();
            stream << "VERTEX_SE3:QUAT " << key_value.key << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x()
                   << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // save 2D landmarks
        for (const auto key_value : estimate)
        {
            auto p = dynamic_cast<const GenericValue<Point2> *>(&key_value.value);
            if (!p)
                continue;
            const Point2 &point = p->value();
            stream << "VERTEX_XY " << key_value.key << " " << point.x() << " " << point.y() << endl;
        }

        // save 3D landmarks
        for (const auto key_value : estimate)
        {
            auto p = dynamic_cast<const GenericValue<Point3> *>(&key_value.value);
            if (!p)
                continue;
            const Point3 &point = p->value();
            stream << "VERTEX_TRACKXYZ " << key_value.key << " " << point.x() << " " << point.y() << " " << point.z()
                   << endl;
        }

        // save edges (2D or 3D)
        for (const auto &factor_ : graph)
        {
            auto factor = boost::dynamic_pointer_cast<BetweenFactor<Pose2>>(factor_);
            if (factor)
            {
                SharedNoiseModel model = factor->noiseModel();
                auto gaussianModel     = boost::dynamic_pointer_cast<noiseModel::Gaussian>(model);
                if (!gaussianModel)
                {
                    model->print("model\n");
                    throw invalid_argument("writeG2o: invalid noise model!");
                }
                Matrix3 Info = gaussianModel->R().transpose() * gaussianModel->R();
                Pose2 pose   = factor->measured();  //.inverse();
                stream << "EDGE_SE2 " << factor->key1() << " " << factor->key2() << " " << pose.x() << " " << pose.y() << " "
                       << pose.theta();
                for (long int i = 0; i < 3; i++)
                {
                    for (long int j = i; j < 3; j++)
                    {
                        stream << " " << Info(i, j);
                    }
                }
                stream << endl;
            }

            auto factor3D = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor_);

            if (factor3D)
            {
                SharedNoiseModel model = factor3D->noiseModel();

                boost::shared_ptr<noiseModel::Gaussian> gaussianModel =
                    boost::dynamic_pointer_cast<noiseModel::Gaussian>(model);
                if (!gaussianModel)
                {
                    model->print("model\n");
                    throw invalid_argument("writeG2o: invalid noise model!");
                }
                Matrix6 Info       = gaussianModel->R().transpose() * gaussianModel->R();
                const Pose3 pose3D = factor3D->measured();
                const Point3 p     = pose3D.translation();
                const auto q       = pose3D.rotation().toQuaternion();
                stream << "EDGE_SE3:QUAT " << factor3D->key1() << " " << factor3D->key2() << " " << p.x() << " " << p.y()
                       << " " << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();

                Matrix6 InfoG2o           = I_6x6;
                InfoG2o.block<3, 3>(0, 0) = Info.block<3, 3>(3, 3);  // cov translation
                InfoG2o.block<3, 3>(3, 3) = Info.block<3, 3>(0, 0);  // cov rotation
                InfoG2o.block<3, 3>(0, 3) = Info.block<3, 3>(0, 3);  // off diagonal
                InfoG2o.block<3, 3>(3, 0) = Info.block<3, 3>(3, 0);  // off diagonal

                for (size_t i = 0; i < 6; i++)
                {
                    for (size_t j = i; j < 6; j++)
                    {
                        stream << " " << InfoG2o(i, j);
                    }
                }
                stream << endl;
            }
        }
        stream.close();
    }

}  // namespace utils
