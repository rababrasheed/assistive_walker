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

    //part2 - phase timing state
    static constexpr uint32_t PHASE1_DURATION_MS = 2000; //2 sec of warning/danger tone before redirect
    static constexpr uint32_t PHASE2_DURATION_MS = 2000; //directional beep window
    static constexpr uint32_t CYCLE_DURATION_MS  = PHASE1_DURATION_MS + PHASE2_DURATION_MS; //full loop length
    bool     obstacle_active;       //true once we've entered warning/danger
    uint32_t obstacle_start_time;   //timestamp when we first entered warning/danger

    //helper functions
    void execute_alert(const ControlMessage& msg);
    void log_status(const ControlMessage& msg);
};