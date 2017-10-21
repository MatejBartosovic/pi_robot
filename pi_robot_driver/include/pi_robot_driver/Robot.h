//
// Created by matejko on 7.10.2017.
//

#ifndef PROJECT_PIROBOT_H
#define PROJECT_PIROBOT_H
#include "pi_robot_driver/Communication.h"
#include <controller_manager/controller_manager.h>


namespace PiRobot{
    class Robot : private Communication, private controller_manager::ControllerManager {
        public:
        Robot();
        void update();

    private:
        ros::Time lastTime;
    };
}
#endif //PROJECT_PIROBOT_H
