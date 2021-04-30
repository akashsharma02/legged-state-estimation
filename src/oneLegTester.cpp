/******************************************************************************
* File:             test.cpp
*
* Author:           Ruoyang Xu
* Created:          04/28/2021
* Description:      Test files for Point Contact Factor
*****************************************************************************/

#include <utils.h>
#include <CLI/CLI.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <gtsam/base/Lie.h>
#include <vector>
#include <string.h>

// Factors
#include "ContactUtils.h"
#include "BetweenContactFactor.h"

// Utils 
#include "fast-cpp-csv-parser/csv.h"
#include "dataloader.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace gtsam;

using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

int main(int argc, char* argv[])
{
    utils::setLogPattern();
    CLI::App app{"Test file for Multi Leg Contact Factor"};

    std::string configFilePath("");
    std::string datasetFilePath("");
    int maxIdx = 10;
    app.add_option("-c, --config", configFilePath, "Configuration input");
    app.add_option("-d, --data", datasetFilePath, "Dataset input");
    app.add_option("-m, --maxIdx", maxIdx, "max index for number of data to be read");
    CLI11_PARSE(app, argc, argv);

    // Load Leg Configs
    std::map<std::string, gtsam::LegConfig> legConfigs = DataLoader::loadLegConfig(configFilePath);

    // Setup IMU Configs
    // 

    // Setup Dataset
    io::CSVReader<23> datafile(datasetFilePath);
    datafile.read_header(io::ignore_extra_column,
        "ts","wx","wy","wz","ax","ay","az","fl0","fl1","fl2",
        "fr0","fr1","fr2","bl0","bl1","bl2","br0","br1","br2",
        "flc","frc","blc","brc"
    );
    double ts,wx,wy,wz,ax,ay,az,fl0,fl1,fl2,fr0,fr1,fr2,bl0,bl1,bl2,br0,br1,br2;
    int flc,frc,blc,brc;
    int idx = 0;
    bool robotReady = false;

    // Setup FactorGraph
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();

    while (datafile.read_row(
        ts,wx,wy,wz,ax,ay,az,fl0,fl1,fl2,fr0,fr1,fr2,bl0,bl1,bl2,br0,br1,br2,flc,frc,blc,brc
        ) && idx++ < maxIdx) {

        // Wait until All Four legs are on the ground, since they were initially floating
        if (flc == 1 && frc == 1 && blc == 1 && brc == 1 && !robotReady) {
            // Mark ready
            robotReady = true;
            
            // Create Prior and Stuff

            // Skip so that factors are not added repeatedly
            continue;
        } else if (!robotReady) {
            // If not ready and also not on the ground, just skip the rest
            continue;
        }





        // if ready
            // If contact sensor no changes
                // accumulate IMU Measurment
            // if contact sensor changes
                // Create New Node in the Factor Graph
                    // IMU Factor
                    // Point Contact Factor
                    // Forward Kinematic Factor
    }
    
}
