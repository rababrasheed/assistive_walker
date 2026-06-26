#pragma once 
#include "message.hpp"
#include "can_bus.hpp"

class SensorUnit {
public: 
    //constructor: connects to shared CAN bus and its own direction (part2)
    SensorUnit(CANBus<SensorMessage>& bus, Direction direction);

    //called once every 70 ms at approx 14 HZ
    //core of the sensor_unit  
    //returns true if the message was successfully placed on the bus 
    bool tick(uint32_t timestamp_ms);

    //used to test specific distances 
    void set_override(float distance_cm);
    void clear_override();

    //used for unit testing/debuging sensor_unit
    float last_distance() const;
    uint32_t message_count() const;

private:
    float simulate_reading(); //test person walking 
    CANBus<SensorMessage>& bus;
    Direction direction; //part2
    float last_distance_cm; 
    uint32_t msg_count;

    bool override_active;
    float override_distance;
    float current_sim_distance; //part 2 - each sensor has its own position 
};
