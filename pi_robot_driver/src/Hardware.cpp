//
// Created by matejko on 7.10.2017.
//

#include "pi_robot_driver/Hardware.h"
#include <ros/ros.h>
#include <urdf/model.h>

namespace PiRobot{
    Hardware::Hardware() : hardware_interface::RobotHW(), pos(2,0), vel(2,0),eff(2,0), vel_cmd(2,0),last_time(ros::Time::now()){
        std::vector<std::string> joints;
        ros::NodeHandle nh("~");
        nh.getParam("joint_names", joints);

        if (joints.size() != 2)
            throw std::length_error("expected 2 joint on parameter server, found " + joints.size());

        //state interface
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[0], &pos[0], &vel[0], &eff[0]));
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[1], &pos[1], &vel[1], &eff[1]));

        //robot velocity interface
        jointVelInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[0]), &vel_cmd[0]));
        jointVelInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[1]), &vel_cmd[1]));


        urdf::Model urdf;
        if (!urdf.initParam("/robot_description"))
            throw std::invalid_argument("robot_description not found on parameter server");


        std::vector<joint_limits_interface::JointLimits> joint_limits(joints.size());
        for (int i = 0; i < joints.size(); i++) {
            urdf::JointConstSharedPtr urdf_joint = urdf.getJoint(joints[i]);

            //get limits from urdf
            if (urdf_joint) {
                getJointLimits(urdf_joint, joint_limits[i]);
            }
            //get limits from parameter server
            joint_limits_interface::getJointLimits(joints[i], ros::NodeHandle("~"), joint_limits[i]);
        }

        for (int i = 0; i < joints.size(); i++) {
            //robot velocity limit
            jointVelocityLimitInterface.registerHandle(joint_limits_interface::VelocityJointSaturationHandle(jointVelInterface.getHandle(joints[i]),joint_limits[i]));
        }
        registerInterface(&jointStatedInterface);
        registerInterface(&jointVelInterface);
        dynServer.setCallback(boost::bind(&Hardware::dynamicReconfigureCallback,this, _1, _2));
    }

    void Hardware::dynamicReconfigureCallback(pi_robot_driver::PiRobotDynamicReconfigureConfig &config, uint32_t level) {
        ROS_INFO("Reconfigure Request");
        /*if(level&Message::SetP)
            messageQueue.push(std::shared_ptr<OutMessage>(new OutMessage(config.left_P,config.right_P,Message::SetP)));
        if(level&Message::SetI)
            messageQueue.push(std::shared_ptr<OutMessage>(new OutMessage(config.left_I,config.right_I,Message::SetI)));
        if(level&Message::SetD)
            messageQueue.push(std::shared_ptr<OutMessage>(new OutMessage(config.left_D,config.right_D,Message::SetD)));
*/        
return;
    }

}
