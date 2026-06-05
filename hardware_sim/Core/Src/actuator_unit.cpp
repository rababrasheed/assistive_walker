#include "actuator_unit.hpp"
#include <cstdio> // used for printf but swap out when testing hardware

#ifdef STM32_BUILD
ActuatorUnit::ActuatorUnit(CANBus<ControlMessage>& control_bus_ref, GPIO_TypeDef* motor_port, uint16_t motor_pin,TIM_HandleTypeDef* buzzer_tim) : control_bus(control_bus_ref),
      motor_port(motor_port), motor_pin(motor_pin), buzzer_tim(buzzer_tim)
{
    total_alerts_count = 0;
    total_logs_count = 0;
    last_toggle = 0;
    buzzer_state = false;
}
#else
//sim constructor:
ActuatorUnit::ActuatorUnit(CANBus<ControlMessage>& control_bus_ref) : control_bus(control_bus_ref)
{
    total_alerts_count=0;
    total_logs_count=0;
}
#endif

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

void ActuatorUnit::execute_alert(const ControlMessage& msg)
{
    switch (msg.command) 
    {
        case CMD_NO_ALERT:
		#ifdef STM32_BUILD
            // motor OFF, buzzer OFF
            HAL_GPIO_WritePin(motor_port, motor_pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(buzzer_tim, TIM_CHANNEL_1, 0);
		#endif
            break; //no action needed bc in CLEAR state 
        case CMD_SLOW_BEEP:
		#ifdef STM32_BUILD
            // motor ON constant, buzzer intermittent every 500ms
            HAL_GPIO_WritePin(motor_port, motor_pin, GPIO_PIN_SET);
            if(HAL_GetTick() - last_toggle >= 500)
            {
                buzzer_state = !buzzer_state;
                __HAL_TIM_SET_COMPARE(buzzer_tim, TIM_CHANNEL_1, buzzer_state ? 7 : 0);
                last_toggle = HAL_GetTick();
            }
		#else
            printf("SLOW BEEP - OBJ APPROACHING\n");
		#endif
            total_alerts_count++;
            break;
        case CMD_RAPID_BEEP:
		#ifdef STM32_BUILD
            // motor ON constant, buzzer solid tone
            HAL_GPIO_WritePin(motor_port, motor_pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(buzzer_tim, TIM_CHANNEL_1, 40);
		#else
            printf("RAPID BEEP - GONNA HIT OBJ\n");
		#endif
            total_alerts_count++;
            break;
        default:
            break;
    }
}

void ActuatorUnit::log_status(const ControlMessage& msg)
{
    unsigned int time_ms = static_cast<unsigned int>(msg.timestamp); //cast for safety
    switch (msg.state)
    {
        case STATE_CLEAR:
            printf("LOG: t=%u ms | state=CLEAR   | dist=%u cm\r\n", time_ms, (unsigned int)msg.distance_cm);
            break;
        case STATE_WARNING:
            printf("LOG: t=%u ms | state=WARNING   | dist=%u cm\r\n", time_ms, (unsigned int)msg.distance_cm);
            break;
        case STATE_DANGER:
            printf("LOG: t=%u ms | state=DANGER   | dist=%u cm\r\n", time_ms, (unsigned int)msg.distance_cm);
            break;
        default:
            printf("LOG: t=%u ms | state=UNKNOWN   | dist=%u cm\r\n", time_ms, (unsigned int)msg.distance_cm);
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


