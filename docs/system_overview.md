ASSISTIVE WALKER SYSTEM OVERVIEW (v1)

I am building a simplified embedded assistive walking device control system.

The system has 3 units:

1. Sensor Unit
- Generates simulated distance sensor data (cm)
  - simulates an HC-SR04 ultrasonic sensor
  - measures distance from walker to nearest obstacle in front

2. Control Unit
- Reads distance data
- Makes decisions based on distance thresholds
- Outputs system state (CLEAR / WARNING / DANGER)
  - CLEAR:   distance > 80 cm
  - WARNING: distance 30–80 cm
  - DANGER:  distance < 30 cm

3. Actuator Unit
- Receives control commands
- Executes alert outputs
- Logs system behavior
  - CLEAR:   no alert, log status
  - WARNING: intermittent beep / vibration
  - DANGER:  continuous rapid beep / vibration

* The Sensor Unit reads (simulated) distance and sends it to the Control Unit,
  which uses the value to determine the system state. This is passed to the
  Actuator Unit which executes the appropriate alert (beep, vibrate, or silent)
  and logs the event.

Communication:
- Units communicate using a simulated CAN-style message system
