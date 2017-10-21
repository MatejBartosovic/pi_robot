//
// Created by matejko on 8.10.2017.
//

#include "pi_robot_driver/Messages.h"
#include <pi_robot_driver/Communication.h>
#include <ros/ros.h>

namespace PiRobot{
    Message::Message(BaseMsg &baseMsg) : baseMsg(baseMsg){
    }

    uint8_t Message::computeCRC(uint8_t *buffer, unsigned int num){
        unsigned int ind = 0;
        uint8_t CRC = 0x00;

        while((ind<num)){
            CRC = har_tableCRC[CRC ^ (*buffer++)];
            ind++;
        }

        return CRC;
    }

    bool Message::check(){
        return (checkHeader() && checkCrc());
    }
    bool Message::checkHeader(){
        if((baseMsg.header != Header) ){
            return false;
        }
        return true;
    }
    bool Message::error(){
        return (baseMsg.msgType & Message::Error);
    }

    uint8_t* Message::get(){
        return  (uint8_t*)(&baseMsg);
    }

    uint8_t Message::getFlags(){
        return baseMsg.msgType;
    }

    OutMessage::OutMessage(float data1, float data2, uint8_t msgType) : Message(msg){
        msg.header = Header;
        msg.msgType = msgType;
        msg.data1 = data1;
        msg.data2 =data2;
        msg.crc = computeCRC(get(),this->size()-1);
    }

    bool OutMessage::checkCrc(){
        if(msg.crc != computeCRC(get(),size()-1)){
            return false;
        }
        return true;
    }

    size_t OutMessage::size(){
        return sizeof(Data);
    }

    InMessage::InMessage() : Message(msg){
    }

    uint8_t* InMessage::getVel(){
        return (uint8_t*)(&msg.leftVel);
    }

    uint8_t* InMessage::getPos(){
        return (uint8_t*)(&msg.leftPos);
    }

    size_t InMessage::size(){
        return sizeof(Data);
    }

    bool InMessage::checkCrc(){
        if(msg.crc != computeCRC(get(),size()-1)){
            return false;
        }
        return true;
    }

}