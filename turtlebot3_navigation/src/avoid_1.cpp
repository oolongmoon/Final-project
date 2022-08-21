//
// Created by oolongmoon on 05/08/2022.
//
// /kobuki/laser/scan
#include "avoid.h"

int main(int argc, char **argv) {
    std::string num = "1";
    ros::init(argc, argv, "node_1");
    Avoid avoid(num.c_str());
    avoid.moving();
    return 0;
}
