//
// Created by matejko on 7.10.2017.
//
#include "pi_robot_driver/Communication.h"
#include "pi_robot_driver/Messages.h"
#include <ros/ros.h>


namespace PiRobot{

    Communication::Communication() : last_vel_cmd(2,0), expectedAnswer(0x00){
        ros::NodeHandle nh("~");
        std::string port;
        double baud;
        int timeOut;
        nh.param<std::string>("port", port,"/dev/ttyS0");
        nh.param<double>("baud", baud,460800);
        nh.param<int>("time_out",timeOut, 5);
        ROS_INFO("Starting pi_robot on %s port with baud %lf and timeout %d ms",port.c_str(),baud,timeOut);
        try{
            serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(timeOut),serial::eightbits,serial::parity_none,serial::stopbits_one,serial::flowcontrol_none);

        }catch (std::exception& e){
            ROS_ERROR("pi_robot: port don't open: %s", e.what());
        }
        serial->flushInput();
    }
    void Communication::read(){
        InMessage msg;
        serial->flushInput(); //this have to be here because of stream and bad synchronization on startup (old messages was read out)
        if(serial->read(msg.get(),msg.size())!= msg.size()){
            ROS_ERROR("Timeout");
            vel = std::vector<double>(2,0);
            pos = std::vector<double>(2,0);
            return;
        }
        if(msg.check()){
            if(msg.getFlags() & Message::Odometry){
                memcpy(&vel,msg.getVel(),2* sizeof(float));
                memcpy(&pos,msg.getPos(),2* sizeof(float));
            }
            if(msg.error()){
                ROS_ERROR("Robot responded with error.");
            }
            if(msg.getFlags()&expectedAnswer){
                ROS_INFO("pop queue %x\n",msg.getFlags());
                if(!messageQueue.empty())
                    messageQueue.pop();
            }
        }
        else{
          syncRead();
        }
        return;
    }

    void Communication::write(){
        if(vel_cmd != last_vel_cmd){
            OutMessage msg(vel_cmd[0],vel_cmd[1]);
            ROS_WARN("New set point %lf %lf",vel_cmd[0],vel_cmd[1]);
            serial->write(msg.get(), msg.size());
            last_vel_cmd = vel_cmd;

        }
        else if (!messageQueue.empty()){
            for (int i = 0; i< vel.size(); i++){
                if (vel[i] != 0)
                    return;
            }
            std::shared_ptr<OutMessage> msg = messageQueue.front();
            expectedAnswer = msg->getFlags();
            serial->write(msg->get(), msg->size());
        }
    }

    void Communication::syncRead(){
        ROS_WARN("Syncing read communication");
        InMessage msg;
        uint8_t index = 0;
        uint8_t count = 0;
        while(true){
            if(!serial->read(msg.get()+index,1)){
                ROS_ERROR("Timeout while syncing");
                continue;
            }
            if(msg.checkHeader()){
                if(serial->read(msg.get()+2,msg.size()-2) != msg.size()-2){
                    ROS_ERROR("Timeout while syncing");
                    continue;
                }
                if(!msg.checkCrc()){
                    ROS_ERROR("header %d %d",*msg.get(),*(msg.get()+1));
                    continue;
                }
                memcpy(&vel,msg.getVel(),2* sizeof(float));
                memcpy(&pos,msg.getPos(),2* sizeof(float));
                ROS_INFO("Synced. count = %d",count);
                return;
            }
            count++;
            index = count & 0x01;
        }
    }
}