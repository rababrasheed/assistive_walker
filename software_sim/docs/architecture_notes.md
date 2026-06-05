# Architecture Notes

## Sensor Unit
Responsible for:
- collecting simulated distance sensor data
- packaging sensor messages
- sending messages to Control Unit

Sensors:
- distance (cm) — primary obstacle detection sensor
  - simulates HC-SR04 ultrasonic sensor readings
  - measures distance from walker to nearest object in front

---

## Control Unit
Responsible for:
- receiving sensor messages
- determining system state based on distance thresholds
- sending control commands to Actuator Unit

Possible system states:
- CLEAR    → distance > 80 cm, no obstacle
- WARNING  → distance 30–80 cm, obstacle approaching
- DANGER   → distance < 30 cm, immediate obstacle

---

## Actuator Unit
Responsible for:
- receiving control commands
- executing alert behavior
- logging system activity

Possible actions:
- no alert       (CLEAR state)
- slow beep      (WARNING state — intermittent buzz)
- rapid beep     (DANGER state — continuous alert)
- log status     (all states — record state + distance)

---

## Communication Model

Units communicate through a simulated CAN-style message system.

Messages contain:
- message ID
- sensor value (distance in cm)
- timestamp (later)