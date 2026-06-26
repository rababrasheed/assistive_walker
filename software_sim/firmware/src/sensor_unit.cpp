#include "sensor_unit.hpp"
#include <cstdlib> //for rand()

//constructor
SensorUnit::SensorUnit(CANBus<SensorMessage>& bus_ref, Direction direction) : bus(bus_ref), direction(direction)
{
    last_distance_cm=0.0f;
    msg_count=0;
    override_active=false;
    override_distance=0.0f;
    current_sim_distance = 100.0f; //part2 - set each sensor to 100cm initially
}

bool SensorUnit::tick(uint32_t timestamp_ms)
{
    SensorMessage msg; 

    //determine which distance measurement is being used 
    if(override_active)
    {
        msg.distance_cm= override_distance; 
    }
    else
    {
        msg.distance_cm= simulate_reading(); 
    }

    //populate the rest of the CAN sensor message 
    msg.id = 0x100; 
    msg.timestamp=timestamp_ms;
    msg.direction=direction; //part2 (which sensor sent the message)

    //save distance history 
    last_distance_cm= msg.distance_cm;

    //send msg to bus
    if(bus.send(msg))
    {
        msg_count++;
        return true; //message was successfully passed to bus
    }
    return false; //bus is full, message not sent. 
}

void SensorUnit::set_override(float distance_cm)
{
    override_active=true;
    override_distance=distance_cm;
}

void SensorUnit::clear_override()
{
    override_active=false;
    override_distance=0.0f; 
}


float SensorUnit::last_distance() const
{
    return last_distance_cm;
}

uint32_t SensorUnit::message_count() const
{
    return msg_count;
}

//replace with STM32 HAL peripheral calls later 
float SensorUnit::simulate_reading() 
{
    //static float current = 100.0f; //start person at 100cm 

    //pick random step (in range -5cm to 5cm)
    float step = (static_cast<float>(rand())/RAND_MAX) * 10.0f - 5.0f; 
    current_sim_distance+=step; //add step to current pos 

    //keep in range
    if(current_sim_distance<5.0f) //5cm is the min limit of the HCSR04 sensor
    {
        current_sim_distance=5.0f;
    }
    if(current_sim_distance>200.0f) //set 200cm as max limit (sensor max is approx 400cm)
    {
        current_sim_distance=200.0f;
    }
    return current_sim_distance;
}