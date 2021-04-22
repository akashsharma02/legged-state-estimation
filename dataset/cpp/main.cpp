#include "IMU.h"


#include <cxxopts.hpp>
#include <iostream>
#include <fstream>

#define CSV_IO_NO_THREAD

#include "fast-cpp-csv-parser/csv.h"

void estimateGravity(cv::Mat orientation) {

}

int main(int argc, char** argv) {

    // auto options("PreIMU Test", "Testing IMU Preintegration");
    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    options.add_options() 
        ("f,file", "File name", cxxopts::value<std::string>()->default_value("../data/sequence3.txt"))
        ("m,maxIdx", "Max Index", cxxopts::value<int>()->default_value("500"))
        ;

    auto opt_result = options.parse(argc, argv);
    std::string filename = opt_result["file"].as<std::string>();
    int max_idx = opt_result["maxIdx"].as<int>();

    io::CSVReader<8> in(filename);
    in.read_header(io::ignore_extra_column,
    "idx","ts","wx","wy","wz","ax","ay","az");
    float wx, wy, wz, ax, ay, az;
    int pl;
    double ts;
    int idx = 0;
    
    std::vector<ORB_SLAM2::ImuMeasure> cum_meas;
    cum_meas.reserve(100);
    std::vector<ORB_SLAM2::PreintegratedIMU> final_Imus;

    // Frame 1 Ground truth
    ORB_SLAM2::PreintegratedIMU new_meas;
    new_meas.initialize();
    cv::Mat g = cv::Mat::zeros(3, 1, CV_64F);
    new_meas.setGravity(g);

    while (in.read_row(pl, ts, wx, wy, wz, ax, ay, az) && idx++ < max_idx) {
        ts /= 200;

        ORB_SLAM2::ImuMeasure meas(ax, ay, az, wx, wy, wz, ts);
        cum_meas.push_back(meas);
        // auto acc = meas._a;
        // auto angVel = meas._w;
        // new_meas.integrateMeasurement(acc, angVel, 1. / 200);

        if (cum_meas.size() >= 100) {
            // No Bias (Assumption), No Calib (EuRoC, where IMU is the centre)
            ORB_SLAM2::PreintegratedIMU new_meas;
            cv::Mat g = cv::Mat::zeros(3, 1, CV_64F);
            new_meas.setGravity(g);
            int n = cum_meas.size();

            // If no previous Integrated Measurement
            if (final_Imus.size() == 0) {
                new_meas.initialize();
                // std::cout << "This is only called once" << std::endl;
            } else {
                new_meas.mergePrior(final_Imus.back());
            }

            for (size_t i = 0; i < cum_meas.size(); i++) {
                // double dt;
                cv::Point3f acc, angVel;
                // double last_ts;
                // if (final_Imus.size() == 0) last_ts = cum_meas.at(i)._ts;
                // else last_ts = final_Imus.back().getTS();

                // if (n == 1) {
                //     // There are only 2 IMU Cumulated
                //     acc = (cum_meas.at(i)._a + cum_meas.at(i + 1)._a) / (2.f);
                //     angVel = (cum_meas.at(i)._w + cum_meas.at(i + 1)._w) / (2.f);
                //     // ORB SLAM for the same case uses current frame time stamp and last frame timestamp
                //     dt = cum_meas.at(i + 1)._ts - final_Imus.back().getTS();
                // } else {
                //     acc = (cum_meas.at(i)._a + cum_meas.at(i + 1)._a) / (2.f);
                //     angVel = (cum_meas.at(i)._w + cum_meas.at(i + 1)._w) / (2.f);
                //     if (i == 0) {
                //         // linear interpolate to previous value
                //         dt = cum_meas.at(i + 1)._ts - last_ts;
                //     } else if (i == (n - 1)) {
                //         dt = last_ts - cum_meas.at(i + 1)._ts;
                //     } else { 
                //         // Averaging ?
                //         dt = cum_meas.at(i + 1)._ts - cum_meas.at(i)._ts;
                //     }
                // }
                acc = cum_meas.at(i)._a;
                angVel = cum_meas.at(i)._w;
                new_meas.integrateMeasurement(acc, angVel, 1. / 200);
            }

            new_meas.setKeyFrameTS(cum_meas.back()._ts);
            final_Imus.push_back(new_meas);
            cum_meas.clear();
        }
    }

    std::ofstream result;
    result.open("result.csv");
    result << "ts,x,y,z,w,i,j,k" << std::endl;
    for (auto const & i : final_Imus) {
        result << i << std::endl;
    }

}