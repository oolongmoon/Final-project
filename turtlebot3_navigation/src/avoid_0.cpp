//
// Created by oolongmoon on 08/08/2022.
//

#include "avoid.h"

int main(int argc, char **argv) {
    std::string num = "0";
    ros::init(argc, argv, "node_0");
    Avoid avoid(num.c_str());
    avoid.moving();
    return 0;
}