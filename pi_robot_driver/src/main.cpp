//
// Created by matejko on 7.10.2017.
//
#include <ros/ros.h>
#include "pi_robot_driver/Robot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    PiRobot::Robot robot;
    ros::Rate r(1); // 10 hz
    ros::AsyncSpinner spinner(2);
    spinner.start();
    while (ros::ok())
    {
        robot.update();
    }
    ros::spin();
}