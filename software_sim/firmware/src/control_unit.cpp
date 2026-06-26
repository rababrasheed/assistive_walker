#include "control_unit.hpp"

//constructor 
ControlUnit::ControlUnit(CANBus<SensorMessage>& sensor_bus_ref, CANBus<ControlMessage>& control_bus_ref): sensor_bus(sensor_bus_ref), control_bus(control_bus_ref)
{
    threshold_clear=80.0f;
    threshold_danger=30.0f;
    last_state_val= STATE_CLEAR;
    msg_count=0;

    //part2 - assume clear until we hear otherwise
    for(int i=0; i<4; i++)
    {
        distances[i]=999.0f;
        fresh[i]=false;
    }
}

/*void ControlUnit::tick()
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
*/

void ControlUnit::tick()
{
    SensorMessage smsg;

    //reset freshness flags each cycle so we only act on what we actually got this cycle
    for(int i=0; i<4; i++)
    {
        fresh[i]=false;
    }

    //read every sensor message on the queue and sort into distances[] by direction
    while(!sensor_bus.is_empty())
    {
        if(!sensor_bus.receive(smsg))
        {
            break; //no sensor messages left to process
        }
        distances[smsg.direction]=smsg.distance_cm;
        fresh[smsg.direction]=true;
    }

    //if front didn't update this cycle, nothing meaningful changed so skip
    if(!fresh[DIR_FRONT])
    {
        return;
    }

    SystemState state = classify(distances[DIR_FRONT]);
    AlertCommand command = command_for(state);
    last_state_val=state;

    //only look for an alternate direction if front isn't clear
    AlertCommand dir_command = CMD_NO_ALERT;
    if(state != STATE_CLEAR)
    {
        dir_command = find_clear_direction();
    }

    //populate the CAN control message 
    ControlMessage cmsg;
    cmsg.id= 0x200;
    cmsg.distance_cm= distances[DIR_FRONT];
    cmsg.timestamp= smsg.timestamp;
    cmsg.state= state;
    cmsg.command = command;
    cmsg.dir_command = dir_command;  //what direction to go 

    //push control message to bus
    if(control_bus.send(cmsg))
    {
        msg_count++;
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

//part2 - finds best alternate direction when front is blocked
//priority order: right, then left, then back
AlertCommand ControlUnit::find_clear_direction() const
{
    bool right_clear = distances[DIR_RIGHT] >= threshold_clear;
    bool left_clear   = distances[DIR_LEFT]  >= threshold_clear;
    bool back_clear   = distances[DIR_BACK]  >= threshold_clear;

    if(right_clear)
    {
        return CMD_DIR_RIGHT;
    }
    else if(left_clear)
    {
        return CMD_DIR_LEFT;
    }
    else if(back_clear)
    {
        return CMD_DIR_BACK;
    }
    else
    {
        return CMD_ALL_BLOCKED;
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