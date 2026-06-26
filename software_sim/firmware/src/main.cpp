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
    //SensorUnit   sensor(sensor_bus);

    //part2 - 4 sensors, one per direction, all sharing the same sensor_bus
    SensorUnit front(sensor_bus, DIR_FRONT);
    SensorUnit left(sensor_bus, DIR_LEFT);
    SensorUnit right(sensor_bus, DIR_RIGHT);
    SensorUnit back(sensor_bus, DIR_BACK);

    ControlUnit  control(sensor_bus, control_bus);
    ActuatorUnit actuator(control_bus);

    // Test 1: CLEAR (dist > 80cm) ; front 100cm, sides/back also clear
    printf("Test 1 - ALL CLEAR (100cm): \n");
    //sensor.set_override(100.0f);
    //sensor.tick(0);
    front.set_override(100.0f);
    left.set_override(100.0f);
    right.set_override(100.0f);
    back.set_override(100.0f);
    front.tick(0);
    left.tick(0); 
    right.tick(0); 
    back.tick(0);
    control.tick();
    actuator.tick();

    // Test 2: WARNING (30-80cm) ; front WARNING, right is clear (expect redirect right after phase 1)
    //printf("Test 2 - WARNING (50cm): \n");
    //sensor.set_override(50.0f);
    //sensor.tick(70);
    printf("\nTest 2 - front WARNING (50cm), right clear: \n");
    front.set_override(50.0f);
    right.set_override(100.0f);
    left.set_override(20.0f);  //blocked
    back.set_override(20.0f);  //blocked
    front.tick(70); 
    left.tick(70); 
    right.tick(70); 
    back.tick(70);
    control.tick();
    actuator.tick();

    // Test 3: DANGER (dist < 30cm) ; front DANGER, only left is clear (expect redirect left)
    //printf("Test 3 - DANGER (15cm): \n");
    //sensor.set_override(15.0f);
    //sensor.tick(140);
    printf("\nTest 3 - front DANGER (15cm), left clear: \n");
    front.set_override(15.0f);
    left.set_override(100.0f);
    right.set_override(20.0f); //blocked
    back.set_override(20.0f);  //blocked
    front.tick(140); 
    left.tick(140); 
    right.tick(140); 
    back.tick(140);
    control.tick();
    actuator.tick();

    // Test 4: boundary — exactly 80cm (should be CLEAR not WARNING) ; front DANGER, only back is clear (expect redirect back)
    //printf("Test 4 - boundary 80cm (expect CLEAR): \n");
    //sensor.set_override(80.0f);
    //sensor.tick(210);
    printf("\nTest 4 - front DANGER (15cm), only back clear: \n");
    front.set_override(15.0f);
    left.set_override(20.0f);  //blocked
    right.set_override(20.0f); //blocked
    back.set_override(100.0f);
    front.tick(210); 
    left.tick(210); 
    right.tick(210); 
    back.tick(210);
    control.tick();
    actuator.tick();

    // Test 5: boundary — exactly 30cm (should be DANGER not WARNING) ; all blocked (expect CMD_ALL_BLOCKED)
    //printf("Test 5 - boundary 30cm (expect DANGER): \n");
    //sensor.set_override(30.0f);
    //sensor.tick(280);
    printf("\nTest 5 - all directions blocked: \n");
    front.set_override(15.0f);
    left.set_override(15.0f);
    right.set_override(15.0f);
    back.set_override(15.0f);
    front.tick(280); 
    left.tick(280); 
    right.tick(280);
    back.tick(280);
    control.tick();
    actuator.tick();

    // Test 6: boundary — exactly 80cm front (should be CLEAR not WARNING)
    printf("\nTest 6 - boundary 80cm front (expect CLEAR): \n");
    front.set_override(80.0f);
    front.tick(350); 
    left.tick(350); 
    right.tick(350); 
    back.tick(350);
    control.tick();
    actuator.tick();

    // Test 7: boundary — exactly 30cm front (should be DANGER not WARNING)
    printf("\nTest 7 - boundary 30cm front (expect DANGER): \n");
    front.set_override(30.0f);
    front.tick(420); 
    left.tick(420); 
    right.tick(420); 
    back.tick(420);
    control.tick();
    actuator.tick();

    // Test 8: phase cycling — front stays DANGER for 4+ seconds straight
    // verifies phase 1 (urgency tone) -> phase 2 (directional beep) -> loops back to phase 1
    printf("\nTest 8 - phase cycling over time (front DANGER, right clear, held for 5s): \n");
    front.set_override(15.0f);
    right.set_override(100.0f);
    left.set_override(20.0f);
    back.set_override(20.0f);
    uint32_t phase_test_start = 490;
    for(uint32_t t = 0; t < 5000; t += TICK_MS)
    {
        uint32_t ts = phase_test_start + t;
        front.tick(ts); 
        left.tick(ts); 
        right.tick(ts); 
        back.tick(ts);
        control.tick();
        actuator.tick();
    }

    // Test 9: flood bus — confirm no crash at capacity
    printf("Test 9 - flood bus (expect size 1000): \n");
    //sensor.clear_override();
    front.clear_override();
    left.clear_override();
    right.clear_override();
    back.clear_override();
    for(uint32_t i = 0; i < 1200; i++)
    {
        //sensor.tick(i * 70);
        front.tick(i*70);
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
    //SensorUnit sensor(sensor_bus);

    //part2 - 4 sensors, one per direction
    SensorUnit front(sensor_bus, DIR_FRONT);
    SensorUnit left(sensor_bus, DIR_LEFT);
    SensorUnit right(sensor_bus, DIR_RIGHT);
    SensorUnit back(sensor_bus, DIR_BACK);
    ControlUnit control(sensor_bus, control_bus);
    ActuatorUnit actuator(control_bus);

    printf("Assistive Walker Simulation — %u ticks x %u ms ===\n\n", TOTAL_TICKS, TICK_MS);

    //main sim loop
    //run inside a 14 Hz timer ISR or RTOS task 
    for(uint32_t tick=0; tick<TOTAL_TICKS; ++tick) 
    {
        uint32_t timestamp_ms=tick*TICK_MS; //every 70 ms
        //sensor.tick(timestamp_ms); //sensor generates a reading and puts it on the sensor bus
        //part2 - all 4 sensors read every cycle
        front.tick(timestamp_ms);
        left.tick(timestamp_ms);
        right.tick(timestamp_ms);
        back.tick(timestamp_ms);
        control.tick(); //control reads sensor bus, classifies distance, and sends control command on control bus
        actuator.tick(); // actuator reads control bus, alerts user, logs
    }

    printf("Simulation complete!\n");
    //printf("Sensor messages sent : %u\n", sensor.message_count());
    printf("Front sensor messages: %u\n", front.message_count());
    printf("Control messages sent: %u\n", control.message_count());
    printf("Total alerts fired   : %u\n", actuator.total_alerts());
    printf("Total log entries    : %u\n", actuator.total_logs());

    return 0;
}
