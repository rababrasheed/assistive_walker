# Assistive Walker Obstacle Detection System

## Overview

A C++ embedded systems project simulating a proximity-based obstacle detection
system for an assistive walker. Four HC-SR04 ultrasonic sensors cover front,
left, right, and back directions. When an obstacle is detected, the system
alerts the user through a two-phase buzzer and haptic feedback cycle, then
guides them toward the clearest available path. Units communicate through a
simulated CAN-style message bus architecture inspired by automotive ECU design
patterns.

---

## Hardware

- STM32 Nucleo-F446RE
- 4x HC-SR04 Ultrasonic Sensors (front, left, right, back)
- Passive Buzzer Module
- Coin Vibration Motor Module

---

## System Architecture

Three independent units communicate over two typed CAN-style message queues:

**SensorUnit → sensor_bus → ControlUnit → control_bus → ActuatorUnit**

### Sensor Unit

- Four independent SensorUnit instances, one per direction (FRONT, LEFT, RIGHT, BACK)
- Each polls its HC-SR04 at ~14 Hz (70ms tick)
- Packages distance reading and direction into a SensorMessage and pushes to sensor bus
- Polling frequency chosen to prevent echo wave aliasing and satisfy HC-SR04
  minimum 60ms cycle requirement
- Supports distance override for unit testing

### Control Unit

- Drains sensor bus each tick, sorting readings into a direction-indexed array
- Classifies front sensor distance into system state:
  - CLEAR   — distance ≥ 80 cm
  - WARNING — distance 30–80 cm
  - DANGER  — distance ≤ 30 cm
- When front is blocked, finds best alternate direction using priority order: right → left → back
- Packs state, alert command, and directional command into a ControlMessage and pushes to control bus

### Actuator Unit

- Drains control bus each tick
- Executes a two-phase alert cycle when obstacle is detected:
  - **Phase 1 (0–2s):** urgency tone + vibration motor ON (slow beep for WARNING, rapid beep for DANGER)
  - **Phase 2 (2–4s):** directional beep pattern, motor OFF (1 beep = go right, 2 = go left, 3 = go back, continuous = all blocked)
  - Cycle repeats every 4 seconds until obstacle clears
- Resets to silent when front sensor returns CLEAR
- Logs all state transitions with timestamp

---

## Communication

Units communicate via a templated fixed-capacity circular FIFO queue (`CANBus<T>`).

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

The simulation runs 9 unit tests before the main loop:

- Test 1: All directions CLEAR (100cm)
- Test 2: Front WARNING (50cm), right clear → expect redirect right
- Test 3: Front DANGER (15cm), left clear → expect redirect left
- Test 4: Front DANGER (15cm), only back clear → expect redirect back
- Test 5: All directions blocked → expect CMD_ALL_BLOCKED
- Test 6: Boundary at exactly 80cm → expect CLEAR
- Test 7: Boundary at exactly 30cm → expect DANGER
- Test 8: Phase cycling — front DANGER held 5s, right clear → verifies phase 1 → phase 2 → phase 1 loop
- Test 9: Bus flood (1200 messages → confirm cap at 1000)

---

## Next Steps

- **Hardware integration** — port multi-directional sensor logic to STM32 HAL,
  wire 4x HC-SR04 with dedicated trigger/echo pins, replace printf with GPIO/PWM calls
- **FreeRTOS migration** — replace bare metal superloop with dedicated tasks
  for SensorUnit, ControlUnit, and ActuatorUnit running at proper priorities with `vTaskDelayUntil`
- **IWDG watchdog timer** — fault tolerance, auto-reset if main loop stalls