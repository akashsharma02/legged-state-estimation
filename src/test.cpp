/******************************************************************************
* File:             test.cpp
*
* Author:           Akash Sharma, Ruoyang Xu
* Created:          04/11/21
* Description:      Test file for gtsam compilation
*****************************************************************************/
#include <utils.h>

#include <CLI/CLI.hpp>

int main(int argc, char* argv[])
{
    utils::setLogPattern();
    CLI::App app{"Test file for GTSAM compilation"};

    std::string datafile("/tmp/");
    app.add_option("-f, --datafile", datafile, "Datafile input");

    CLI11_PARSE(app, argc, argv);
}

