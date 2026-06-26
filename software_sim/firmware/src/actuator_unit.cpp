#include "actuator_unit.hpp"
#include <cstdio> // used for printf but swap out when testing hardware

//constructor:
ActuatorUnit::ActuatorUnit(CANBus<ControlMessage>& control_bus_ref) : control_bus(control_bus_ref)
{
    total_alerts_count=0;
    total_logs_count=0;
    obstacle_active=false;     //part2
    obstacle_start_time=0;     //part2
}

void ActuatorUnit::tick()
{
    ControlMessage cmsg;
    //read every control message on the queue 
    while(!control_bus.is_empty())
    {
        if(!control_bus.receive(cmsg))
        {
            break; //no control messages left to process
        }
        execute_alert(cmsg);
        log_status(cmsg);
    }
}

/*void ActuatorUnit::execute_alert(const ControlMessage& msg)
{
    switch (msg.command) 
    {
        case CMD_NO_ALERT:
            break; //no action needed bc in CLEAR state 
        case CMD_SLOW_BEEP:
            printf("SLOW BEEP - OBJ APPROACHING\n");
            total_alerts_count++;
            break;
        case CMD_RAPID_BEEP:
            printf("RAPID BEEP - GONNA HIT OBJ\n");
            total_alerts_count++;
            break;
        default:
            break;
    }
}
*/
void ActuatorUnit::execute_alert(const ControlMessage& msg)
{
    //front is clear, reset phase tracking, stay silent
    if(msg.command == CMD_NO_ALERT)
    {
        obstacle_active=false;
        return; //no action needed bc in CLEAR state
    }

    //we're in warning or danger. mark when this obstacle episode started
    if(!obstacle_active)
    {
        obstacle_active=true;
        obstacle_start_time=msg.timestamp;
    }

    //wrap elapsed time into the cycle so it loops 0..4000 
    uint32_t elapsed = (msg.timestamp - obstacle_start_time) % CYCLE_DURATION_MS;

    //phase 1: 0-2000ms — urgency tone, motor on
    if(elapsed < PHASE1_DURATION_MS)
    {
        switch (msg.command)
        {
            case CMD_SLOW_BEEP:
                printf("SLOW BEEP - OBJ APPROACHING (motor ON)\n");
                total_alerts_count++;
                break;
            case CMD_RAPID_BEEP:
                printf("RAPID BEEP - GONNA HIT OBJ (motor ON)\n");
                total_alerts_count++;
                break;
            default:
                break;
        }
    }
    //phase 2: 2000-4000ms — directional pattern, motor off
    else
    {
        switch (msg.dir_command)
        {
            case CMD_DIR_RIGHT:
                printf("BEEP (1x) - GO RIGHT (motor OFF)\n");
                total_alerts_count++;
                break;
            case CMD_DIR_LEFT:
                printf("BEEP BEEP (2x) - GO LEFT (motor OFF)\n");
                total_alerts_count++;
                break;
            case CMD_DIR_BACK:
                printf("BEEP BEEP BEEP (3x) - GO BACK (motor OFF)\n");
                total_alerts_count++;
                break;
            case CMD_ALL_BLOCKED:
                printf("CONTINUOUS TONE - ALL BLOCKED, STOP (motor OFF)\n");
                total_alerts_count++;
                break;
            default:
                break;
        }
    }
}

void ActuatorUnit::log_status(const ControlMessage& msg)
{
    unsigned int time_ms = static_cast<unsigned int>(msg.timestamp); //cast for safety
    switch (msg.state)
    {
        case STATE_CLEAR:
            printf("LOG: t=%u ms | state=CLEAR   | dist=%.1f cm\n", time_ms, msg.distance_cm);
            break;
        case STATE_WARNING:
            printf("LOG: t=%u ms | state=WARNING   | dist=%.1f cm\n", time_ms, msg.distance_cm);
            break;
        case STATE_DANGER:
            printf("LOG: t=%u ms | state=DANGER   | dist=%.1f cm\n", time_ms, msg.distance_cm);
            break;
        default:
            printf("LOG: t=%u ms | state=UNKNOWN   | dist=%.1f cm\n", time_ms, msg.distance_cm);
            break;
    }
    total_logs_count++;
}

uint32_t ActuatorUnit::total_alerts() const
{
    return total_alerts_count;
}

uint32_t ActuatorUnit::total_logs() const
{
    return total_logs_count;
}


