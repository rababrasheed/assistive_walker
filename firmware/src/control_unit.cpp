#include "control_unit.hpp"

//constructor 
ControlUnit::ControlUnit(CANBus<SensorMessage>& sensor_bus_ref, CANBus<ControlMessage>& control_bus_ref): sensor_bus(sensor_bus_ref), control_bus(control_bus_ref)
{
    threshold_clear=80.0f;
    threshold_danger=30.0f;
    last_state_val= STATE_CLEAR;
    msg_count=0;
}

void ControlUnit::tick()
{
    SensorMessage smsg;
    //read every sensor message on the queue 
    while(!sensor_bus.is_empty())
    {
        if(!sensor_bus.receive(smsg))
        {
            break; //no sensor messages left to process
        }
        
        SystemState state = classify(smsg.distance_cm);
        AlertCommand command = command_for(state);
        last_state_val=state;

        //populate the CAN control message 
        ControlMessage cmsg;
        cmsg.id= 0x200;
        cmsg.distance_cm= smsg.distance_cm;
        cmsg.timestamp= smsg.timestamp;
        cmsg.state= state;
        cmsg.command = command;

        //push control message to bus
        if(control_bus.send(cmsg))
        {
            msg_count++;
        }
    }
}

//determine what state system is in 
SystemState ControlUnit::classify(float distance_cm) const
{
    if(distance_cm>=threshold_clear)
    {
        return STATE_CLEAR;
    }
    else if (distance_cm<=threshold_danger)
    {
        return STATE_DANGER;
    }
    else 
    {
        return STATE_WARNING;
    }
}

//determine what the output should be based on the system state 
AlertCommand ControlUnit::command_for(SystemState state) const
{
    switch (state) {
        case STATE_CLEAR:
            return CMD_NO_ALERT;
        case STATE_DANGER:
            return CMD_RAPID_BEEP;
        case STATE_WARNING:
            return CMD_SLOW_BEEP;
        default:
            return CMD_NO_ALERT;
    }
}

SystemState ControlUnit::last_state() const
{
    return last_state_val;
}

uint32_t ControlUnit::message_count() const
{
    return msg_count;
}