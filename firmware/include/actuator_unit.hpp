#pragma once
#include "message.hpp"
#include "can_bus.hpp"

class ActuatorUnit{
public:
    //constructor: connects to control bus 
    ActuatorUnit(CANBus<ControlMessage>& control_bus_ref);

    //read all control commands
    //core of the actuator unit
    void tick();

    //used for unit testing/debuging control unit
    uint32_t total_alerts() const;
    uint32_t total_logs() const;

private: 
    CANBus<ControlMessage>& control_bus; 
    uint32_t total_alerts_count;
    uint32_t total_logs_count; 
    //helper functions
    void execute_alert(const ControlMessage& msg);
    void log_status(const ControlMessage& msg);
};