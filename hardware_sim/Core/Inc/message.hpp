#pragma once 
#include <cstdint> //gives fixed width int types 

//declare enumerations:
enum SystemState {
    STATE_CLEAR =0, //dist>=80 cm 
    STATE_WARNING =1, //30-80cm
    STATE_DANGER =2 //dist<=30cm
};

enum AlertCommand {
    CMD_NO_ALERT =0, //CLEAR so silent
    CMD_SLOW_BEEP =1, //WARNING so intermittent buzz
    CMD_RAPID_BEEP =2 //DANGER so continuous alert
};

//declare message structs:
struct SensorMessage{
    uint16_t id; //CAN message ID (Standard CAN uses an 11 bit identifier)
    float distance_cm; //HC-SR04 sensor reading
    uint32_t timestamp; //simulation time (needs be 32 so reset time is not too minimal-any smaller sizes will reset every min or less)
};

struct ControlMessage{
    uint16_t id; 
    float distance_cm;
    uint32_t timestamp;
    SystemState state; //sys state 
    AlertCommand command; //command corresponding to sys state
};