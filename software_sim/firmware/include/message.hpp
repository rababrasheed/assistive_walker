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
    CMD_RAPID_BEEP =2, //DANGER so continuous alert
    //part2
    CMD_DIR_RIGHT   = 3, //go right (1 beep)
    CMD_DIR_LEFT    = 4, //go left (2 beeps)
    CMD_DIR_BACK    = 5, //go back (3 beeps)
    CMD_ALL_BLOCKED = 6  //no clear path exists
};

//part2
// direction the sensor is facing
enum Direction {
    DIR_FRONT = 0,
    DIR_LEFT  = 1,
    DIR_RIGHT = 2,
    DIR_BACK  = 3
};

//declare message structs:
struct SensorMessage{
    uint16_t id; //CAN message ID (Standard CAN uses an 11 bit identifier)
    float distance_cm; //HC-SR04 sensor reading
    uint32_t timestamp; //simulation time (needs be 32 so reset time is not too minimal-any smaller sizes will reset every min or less)
    //part2
    Direction direction;   //which sensor sent this (FRONT/LEFT/RIGHT/BACK)
};

struct ControlMessage{
    uint16_t id; 
    float distance_cm;
    uint32_t timestamp;
    SystemState state; //sys state 
    AlertCommand command; //warning/danger command 
    //part2
    AlertCommand dir_command;  //directional command (which way to go)
};