#include "message.hpp"
#include "can_bus.hpp"
#include "sensor_unit.hpp"
#include "control_unit.hpp"
#include "actuator_unit.hpp"

#include <cstdio> //used for printf 
#include <cstdlib> //used for srand
#include <ctime> //used for time 

//read only, belongs to this class only, and global (for when additional fns are added in hardware)
static constexpr uint32_t TICK_MS =70; //HC-SR04 polls at approv 14 Hz which is 70ms
static constexpr uint32_t SIM_DURATION= 60000; //total ms (60s)
static constexpr uint32_t TOTAL_TICKS= SIM_DURATION/TICK_MS; //857

//run tests
void run_tests()
{
    printf("RUNNING TESTS\n\n");

    CANBus<SensorMessage>  sensor_bus;
    CANBus<ControlMessage> control_bus;
    SensorUnit   sensor(sensor_bus);
    ControlUnit  control(sensor_bus, control_bus);
    ActuatorUnit actuator(control_bus);

    // Test 1: CLEAR (dist > 80cm)
    printf("Test 1 - CLEAR (100cm): \n");
    sensor.set_override(100.0f);
    sensor.tick(0);
    control.tick();
    actuator.tick();

    // Test 2: WARNING (30-80cm)
    printf("Test 2 - WARNING (50cm): \n");
    sensor.set_override(50.0f);
    sensor.tick(70);
    control.tick();
    actuator.tick();

    // Test 3: DANGER (dist < 30cm)
    printf("Test 3 - DANGER (15cm): \n");
    sensor.set_override(15.0f);
    sensor.tick(140);
    control.tick();
    actuator.tick();

    // Test 4: boundary — exactly 80cm (should be CLEAR not WARNING)
    printf("Test 4 - boundary 80cm (expect CLEAR): \n");
    sensor.set_override(80.0f);
    sensor.tick(210);
    control.tick();
    actuator.tick();

    // Test 5: boundary — exactly 30cm (should be DANGER not WARNING)
    printf("Test 5 - boundary 30cm (expect DANGER): \n");
    sensor.set_override(30.0f);
    sensor.tick(280);
    control.tick();
    actuator.tick();

    // Test 6: flood bus — confirm no crash at capacity
    printf("Test 6 - flood bus (expect size 1000): \n");
    sensor.clear_override();
    for(uint32_t i = 0; i < 1200; i++)
    {
        sensor.tick(i * 70);
    }
    printf("Bus size after flood: %u (should be 1000)\n\n", sensor_bus.size());

    printf("TESTS DONE\n\n"); 
}

int main()
{
    srand(static_cast<unsigned>(time(nullptr))); //seed to generate rand val 

    run_tests();

    //buses
    CANBus<SensorMessage> sensor_bus; // set to size bus cap (1000)
    CANBus<ControlMessage> control_bus; //set to size bus cap (1000)

    //units
    SensorUnit sensor(sensor_bus);
    ControlUnit control(sensor_bus, control_bus);
    ActuatorUnit actuator(control_bus);

    printf("Assistive Walker Simulation — %u ticks x %u ms ===\n\n", TOTAL_TICKS, TICK_MS);

    //main sim loop
    //run inside a 14 Hz timer ISR or RTOS task 
    for(uint32_t tick=0; tick<TOTAL_TICKS; ++tick) 
    {
        uint32_t timestamp_ms=tick*TICK_MS; //every 70 ms
        sensor.tick(timestamp_ms); //sensor generates a reading and puts it on the sensor bus
        control.tick(); //control reads sensor bus, classifies distance, and sends control command on control bus
        actuator.tick(); // actuator reads control bus, alerts user, logs
    }

    printf("Simulation complete!\n");
    printf("Sensor messages sent : %u\n", sensor.message_count());
    printf("Control messages sent: %u\n", control.message_count());
    printf("Total alerts fired   : %u\n", actuator.total_alerts());
    printf("Total log entries    : %u\n", actuator.total_logs());

    return 0;
}
