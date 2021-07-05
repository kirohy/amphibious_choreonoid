#include "calc_motions.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "calc_motions");
    CalcMotions cm;
    ros::spin();
    return 0;
}
