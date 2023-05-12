//
// Created by tioeare on 5/6/23.
//
#include <thread>

#include "get_data.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "data_fusion");

    ros::start();
    fusion::Data data;

    auto *of = new fusion::read_OF();
    // std::thread read_of(&fusion::read_OF::read_serial, of);

    data.start();
    //    read_of.join();
    ros::waitForShutdown();
    ros::shutdown();

    return 0;
}
