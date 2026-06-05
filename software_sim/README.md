# Assistive Walker Obstacle Detection System

## Overview
A C++ embedded systems project simulating a proximity based obstacle detection 
system for an assistive walker. The system uses an HC-SR04 ultrasonic sensor to 
detect nearby obstacles and alerts the user through buzzer and haptic (vibration) 
feedback. Units communicate through a simulated CAN style message bus architecture 
inspired by automotive ECU design patterns.

---

## Hardware
- STM32 Nucleo-F446RE
- HC-SR04 Ultrasonic Sensor
- Passive Buzzer Module
- Coin Vibration Motor Module

---

## System Architecture
Three independent units communicate over two typed CAN-style message queues:

**SensorUnit → sensor_bus → ControlUnit → control_bus → ActuatorUnit**

### Sensor Unit
- Polls HC-SR04 at ~14 Hz (70ms tick)
- Packages distance reading into a SensorMessage and pushes to sensor bus
- Polling frequency chosen to prevent echo wave aliasing and satisfy HC-SR04 
  minimum 60ms cycle requirement
- Supports distance override for unit testing

### Control Unit
- Drains sensor bus each tick
- Classifies distance into system state:
  - CLEAR   — distance ≥ 80 cm
  - WARNING — distance 30–80 cm  
  - DANGER  — distance ≤ 30 cm
- Maps state to alert command and pushes ControlMessage to control bus

### Actuator Unit
- Drains control bus each tick
- Executes alert behavior:
  - CLEAR   → no alert
  - WARNING → slow beep + vibration
  - DANGER  → rapid beep + vibration
- Logs all state transitions with timestamp

---

## Communication
Units communicate via a templated fixed-capacity circular FIFO queue (CANBus<T>).
- Capacity: 1000 messages
- Fixed array (no dynamic memory allocation)
- Works for both SensorMessage and ControlMessage types
- Returns false on send if full, preventing overflow

---

## Build Configuration
The codebase supports both PC simulation and STM32 hardware builds from the 
same source files using a compile-time flag:

- `STM32_BUILD` defined → compiles HAL hardware code (SensorUnit reads real 
  ECHO pin, ActuatorUnit drives real GPIO/PWM)
- `STM32_BUILD` not defined → compiles simulation code (SensorUnit generates 
  random walk distance, ActuatorUnit prints to stdout)

---

## Testing
The simulation runs 5 boundary tests before the main loop:
- CLEAR state (100cm)
- WARNING state (50cm)
- DANGER state (15cm)
- Boundary at 80cm (expect CLEAR)
- Boundary at 30cm (expect DANGER)
- Bus flood test (1200 messages → confirm cap at 1000)

---

## Planned Next Steps
- **Hardware watchdog timer** — fault tolerance, auto-reset if main loop stalls
- **FreeRTOS migration** — replace bare metal superloop with dedicated tasks 
  for SensorUnit, ControlUnit, and ActuatorUnit running at proper priorities
- **Multi-directional detection** — 3x HC-SR04 (left, center, right)
- **Directional haptic feedback** — two vibration motors, one per handle, 
  guide user away from obstacle based on which sensor triggers
