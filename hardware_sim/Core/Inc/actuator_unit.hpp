#pragma once
#include "message.hpp"
#include "can_bus.hpp"

#ifdef STM32_BUILD
#include "main.h"
#endif

class ActuatorUnit{
public:
#ifdef STM32_BUILD
    // hardware constructor: takes motor GPIO and buzzer timer
    ActuatorUnit(CANBus<ControlMessage>& control_bus_ref, GPIO_TypeDef* motor_port, uint16_t motor_pin, TIM_HandleTypeDef* buzzer_tim);
#else
    // sim constructor
    ActuatorUnit(CANBus<ControlMessage>& control_bus_ref);
#endif
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

#ifdef STM32_BUILD
    GPIO_TypeDef* motor_port;
    uint16_t motor_pin;
    TIM_HandleTypeDef* buzzer_tim;
    uint32_t last_toggle;
    bool buzzer_state;
#endif
};
