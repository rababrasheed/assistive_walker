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

## Testing
The simulation runs 5 boundary tests before the main loop:
- CLEAR state (100cm)
- WARNING state (50cm)
- DANGER state (15cm)
- Boundary at 80cm (expect CLEAR)
- Boundary at 30cm (expect DANGER)
- Bus flood test (1200 messages → confirm cap at 1000)

---

## Planned Hardware Integration
- Replace simulate_reading() with STM32 HAL TIM2 input capture for real 
  HC-SR04 Echo pulse measurement
- Replace printf alerts with HAL PWM (TIM3/PC6) for buzzer 
  and GPIO (PA10) for vibration motor
- Timer interrupt driven tick at 14 Hz replacing polling loop
- FreeRTOS task separation for sensor, control, and actuator units
- Hardware watchdog timer for fault tolerance

## Planned Feature Expansion
- Multi-directional obstacle detection (3x HC-SR04: left, center, right)
- Directional haptic feedback (two vibration motors, one per handle)
- Navigation guidance: direct user away from obstacle based on which 
  sensor triggers
