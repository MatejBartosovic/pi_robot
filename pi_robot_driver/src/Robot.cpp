//
// Created by matejko on 7.10.2017.
//

#include "pi_robot_driver/Robot.h"
namespace PiRobot{

    Robot::Robot() : controller_manager::ControllerManager(this){

    }
    void Robot::update(){
        Communication::read();
        ros::Time currentTime = ros::Time::now();
        ros::Duration duration =  currentTime - lastTime;
        controller_manager::ControllerManager::update(currentTime,duration);
        enforceLimits(duration);
        Communication::write();
        lastTime = currentTime;
    }

}