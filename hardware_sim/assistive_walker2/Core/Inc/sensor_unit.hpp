#pragma once 
#include "message.hpp"
#include "can_bus.hpp"

#ifdef STM32_BUILD
#include "main.h"
#endif

class SensorUnit {
public: 
#ifdef STM32_BUILD
	//hardware constructor: takes GPIO info for TRIG and ECHO pins
	SensorUnit(CANBus<SensorMessage>& bus, GPIO_TypeDef* trig_port, uint16_t trig_pin, GPIO_TypeDef* echo_port, uint16_t echo_pin);
#else
    //sim constructor: connects to shared CAN bus
    SensorUnit(CANBus<SensorMessage>& bus);
#endif

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
    float read_distance_cm() const; //hardware read or sim depending on build
    CANBus<SensorMessage>& bus;
    float last_distance_cm; 
    uint32_t msg_count;
    bool override_active;
    float override_distance;

#ifdef STM32_BUILD
    GPIO_TypeDef* trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef* echo_port;
    uint16_t echo_pin;
#endif
};
