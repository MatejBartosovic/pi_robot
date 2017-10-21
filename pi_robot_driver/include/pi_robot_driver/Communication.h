//
// Created by matejko on 7.10.2017.
//

#ifndef PROJECT_PIROBOTCOMUNICATION_H
#define PROJECT_PIROBOTCOMUNICATION_H

#include "pi_robot_driver/Hardware.h"
#include <serial/serial.h>

namespace PiRobot{

    class Communication : public Hardware {
    public:
        Communication();
        void read();
        void write();

    private:
        void syncRead();
        serial::Serial* serial;
        std::vector<double> last_vel_cmd;
        uint8_t expectedAnswer;
    };
}
#endif //PROJECT_PIROBOTCOMUNICATION_H
