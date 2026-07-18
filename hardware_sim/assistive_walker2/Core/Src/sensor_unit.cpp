#include "sensor_unit.hpp"
#include <cstdlib> //for rand()

#ifdef STM32_BUILD
// hardware constructor
//connect to echo and trig pins
SensorUnit::SensorUnit(CANBus<SensorMessage>& bus_ref, GPIO_TypeDef* trig_port, uint16_t trig_pin, GPIO_TypeDef* echo_port, uint16_t echo_pin) : bus(bus_ref),
      trig_port(trig_port), trig_pin(trig_pin), echo_port(echo_port), echo_pin(echo_pin)
{
    last_distance_cm = 0.0f;
    msg_count = 0;
    override_active = false;
    override_distance = 0.0f;
}
#else
// sim constructor
SensorUnit::SensorUnit(CANBus<SensorMessage>& bus_ref): bus(bus_ref)
{
    last_distance_cm=0.0f;
    msg_count=0;
    override_active=false;
    override_distance=0.0f;
}
#endif

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
        msg.distance_cm= read_distance_cm(); //simulate_reading fn modified to read_distance
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

#ifdef STM32_BUILD //only for STM version
float SensorUnit::read_distance_cm() const
{
	  // bring in the shared ISR variables
	//don't create these here, they are defined elsewhere; don't cache to a register they can change anytime
	    extern volatile uint32_t echo_rise_time;
	    extern volatile uint32_t echo_fall_time;
	    extern volatile bool echo_ready;
	    extern TIM_HandleTypeDef htim2;

	    echo_ready = false;  // clear flag for next reading

	    // fire the trigger pulse
	    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
	    for(volatile int i = 0; i < 180; i++);   // 1us delay
	    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
	    for(volatile int i = 0; i < 1800; i++);  // 10us delay
	    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);

	    /*
	    // wait for ISR to complete the reading (with timeout)
	    uint32_t timeout = HAL_GetTick();
	    while(!echo_ready)
	    {
	        if(HAL_GetTick() - timeout > 30)
	        {
	        	return 999.0f; //return 999 if nothing in range
	        }

	    }
	    */
	    uint32_t start_count = __HAL_TIM_GET_COUNTER(&htim2);
	       while(!echo_ready)
	       {
	           // timeout measured off TIM2's own hardware counter -- no HAL_GetTick() involved
	           if((__HAL_TIM_GET_COUNTER(&htim2) - start_count) > 30000)  // 30000 us = 30ms
	           {
	               return 999.0f;
	           }
	       }

	    // reading is ready so calculate distance from ISR timestamps
	    uint32_t elapsed_us = echo_fall_time - echo_rise_time; //how long echo pulse was
	    //return (float)(elapsed_ms * 17); //17 bc sound travels 17cm/ms for a roundtrip
	    return (float)elapsed_us / 58.0f;  // standard HC-SR04 formula: cm = us / 58
}
#else
float SensorUnit::read_distance_cm() const //sim version
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
#endif

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    extern volatile uint32_t echo_rise_time;
    extern volatile uint32_t echo_fall_time;
    extern volatile bool echo_ready;

    if(htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t captured = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        if(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
        {
            echo_rise_time = captured;  // this capture happened on the rising edge
        }
        else
        {
            echo_fall_time = captured;  // this capture happened on the falling edge
            echo_ready = true;
        }
    }
}
