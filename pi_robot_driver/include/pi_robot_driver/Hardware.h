//
// Created by matejko on 7.10.2017.
//

#ifndef PROJECT_PIROBOTHARDWARE_H
#define PROJECT_PIROBOTHARDWARE_H

//ros controll lib
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

//joint limit interface lib
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>

//dynamic reconfigure lib
#include <dynamic_reconfigure/server.h>
#include <pi_robot_driver/PiRobotDynamicReconfigureConfig.h>

//std lib
#include <queue>

//project lib
#include <pi_robot_driver/Messages.h>

namespace PiRobot{
    class Hardware : public hardware_interface::RobotHW{
    public:
        Hardware();
        void dynamicReconfigureCallback(pi_robot_driver::PiRobotDynamicReconfigureConfig &config, uint32_t level);

    protected:

        inline void enforceLimits(){
            ros::Time current_time = ros::Time::now();
            jointVelocityLimitInterface.enforceLimits(current_time - last_time);
            last_time = current_time;
        }

        inline void enforceLimits(ros::Duration duration){
            jointVelocityLimitInterface.enforceLimits(duration);
            last_time = ros::Time::now();
        }

        std::vector<double> pos;                                        //joint positions
        std::vector<double> vel;                                        //joint velocity
        std::vector<double> eff;                                        //joint effort
        std::vector<double> vel_cmd;                                    //joint velocity command to robot
        std::queue<std::shared_ptr<OutMessage>> messageQueue;



    private:
        hardware_interface::JointStateInterface jointStatedInterface;   //state interface
        hardware_interface::VelocityJointInterface jointVelInterface;   //camera position interface
        joint_limits_interface::JointLimits jointLimitInterface;        //limit interface

        joint_limits_interface::VelocityJointSaturationInterface jointVelocityLimitInterface; //velocity saturation interface

        dynamic_reconfigure::Server<pi_robot_driver::PiRobotDynamicReconfigureConfig> dynServer; // dynamic reconfigure serve

        ros::Time last_time;
    };
}
#endif //PROJECT_PIROBOTHARDWARE_H
