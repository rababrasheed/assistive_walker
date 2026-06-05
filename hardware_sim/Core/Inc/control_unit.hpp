#pragma once 
#include "message.hpp"
#include "can_bus.hpp"

class ControlUnit{
public:
    //constuctor: needs to connect to BOTH sensor and command busses
    ControlUnit(CANBus<SensorMessage>& sensor_bus_ref, CANBus<ControlMessage>& control_bus_ref);

    //read all sensor messages and send control commands 
    void tick(); 

    //core of the control unit
    //used for unit testing/debuging control unit
    SystemState last_state() const; 
    uint32_t message_count() const;

    //thresholds made public so they can be easily modified in main
    float threshold_clear; 
    float threshold_danger; 

private: 
    CANBus<SensorMessage>& sensor_bus;
    CANBus<ControlMessage>& control_bus;
    SystemState last_state_val;
    uint32_t msg_count;
    //helper functions
    SystemState classify(float distance_cm) const;
    AlertCommand command_for(SystemState state) const;
};