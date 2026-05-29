#include "sensor_unit.hpp"
#include <cstdlib> //for rand()

//constructor
SensorUnit::SensorUnit(CANBus<SensorMessage>& bus_ref): bus(bus_ref)
{
    last_distance_cm=0.0f;
    msg_count=0;
    override_active=false;
    override_distance=0.0f;
}

bool SensorUnit::tick(uint32_t timestamp_ms)
{
    SensorMessage msg; 

    //determine which distance measurement is being used (test or actual)
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
float SensorUnit::simulate_reading() const
{
    static float current = 100.0f; //start person at 100cm 

    //pick random step (in range -5cm to 5cm)
    float step = (static_cast<float>(rand())/RAND_MAX) * 10.0f - 5.0f; 
    current+=step; //add step to current pos 

    //keep in range
    if(current<5.0f) //5cm is the min limit of the HCSR04 sensor
    {
        current=5.0f;
    }
    if(current>200.0f) //set 200cm as max limit (sensor max is approx 400cm)
    {
        current=200.0f;
    }
    return current;
}