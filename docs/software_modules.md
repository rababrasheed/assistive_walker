# Software Modules

## Sensor Unit Module
Responsibilities:
- generate simulated distance values (cm)
- package distance readings into sensor messages
- send messages to Control Unit

Possible files:
- sensor_unit.cpp
- sensor_unit.hpp

---

## Control Unit Module
Responsibilities:
- receive distance sensor data
- determine system state (CLEAR / WARNING / DANGER)
- send control commands to Actuator Unit

Possible files:
- control_unit.cpp
- control_unit.hpp

---

## Actuator Unit Module
Responsibilities:
- receive control commands
- execute alert behavior (beep / vibrate / silent)
- log system state and distance

Possible files:
- actuator_unit.cpp
- actuator_unit.hpp

---

## CAN Communication Module
Responsibilities:
- simulate CAN-style message passing between units
- define message structure (ID + distance value)
- handle communication between Sensor, Control, and Actuator

Possible files:
- can_bus.cpp
- can_bus.hpp

---

## Main System Module
Responsibilities:
- initialize all units
- start simulation loop
- coordinate overall system execution

Possible files:
- main.cpp