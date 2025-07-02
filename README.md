# RMT-Based Stepper Motor Smooth Controller

## MCPWM-Based PWM Controller for EDM Power Supply

**Current branch:** `stepper-working`

## Goals

- Use only bare minimum ESP-IDF
- Use only C (might change in the future)
- Have the ESP32 control the stepper motor for advance and control the "spark gap"
- Have the ESP32 control voltage/current generation and provide a feedback loop

### What is working so far

- Stepper motor is working:
  - Starts to cut following the start button.
  - Jogging works, but could be improved.
- ADC works, but I want to provide the option to use an external ADC, as the ESP32's ADC is not ideal.
- The control of the constant current generator works on the oscilloscope, but not yet tested with real IGBTs and does not have active control yet.
  - It is just sending signals for a half-bridge IGBT with a gap between signals.
- Hardware direction is not fully decided yet. For now, I am considering a kind of buck-boost configuration with energy recovery and "spark gap" clamping control.
- As far as I understand, there is not a single type of power source in an industrial machine, but rather a combination of several sources depending on the operation.
- The target is to have flexibility to control the same hardware to do as much as possible. EDM machines, due to the spark (plasma) not being fully understood and the lack of a mathematical model that fits the way we try to simplify the equations, are still esoteric in nature and there is still a lot of empirical approach. Combined with the variety of materials and electrodes, this creates a complex situation.
- There is a lot of documentation and publications out there, but there is still a gap between implementation and theoretical approach.

One RMT TX channel can use different encoders in sequence, which is useful to generate waveforms that have multiple stages.

This project shows how to drive a stepper motor with a **STEP/DIR** interface controller (e.g. [DRV8825](https://www.ti.com/lit/ds/symlink/drv8825.pdf)) in a [smooth](https://en.wikipedia.org/wiki/Smoothstep) way. To smoothly drive a stepper motor, there are three phases: **Acceleration**, **Uniform**, and **Deceleration**. The code implements two encoders so that the RMT channel can generate waveforms with different characteristics:

* `curve_encoder` encodes the **Acceleration** and **Deceleration** phases
* `uniform_encoder` encodes the **Uniform** phase

## How to Use

- Added `LIMIT_SWITCH_GPIO` and `START_CUT_GPIO` as new input pins.
- Both pins are now configured as inputs.
- Jogging is only allowed if the limit switch is ON (not triggered).
- EDM control only runs if not jogging, the limit switch is ON, and the start button is ON.
- If the limit switch is OFF, all movement is inhibited (including jogging and EDM control).

The loop now fully supports the new safety and control logic.

## Connection

```
+---------------------------+             +--------------------+      +--------------+
|          ESP Board        |             |       DRV8825      |      |    4-wire    |
|                       GND +-------------+ GND                |      |     Step     |
|                           |             |                    |      |     Motor    |
|                       3V3 +-------------+ VDD             A+ +------+ A+           |
|                           |             |                    |      |              |
|       STEP_MOTOR_GPIO_DIR +------------>+ DIR             A- +------+ A-           |
|                           |             |                    |      |              |
|      STEP_MOTOR_GPIO_STEP +------------>+ STEP            B- +------+ B-           |
|                           |             |                    |      |              |
|                           |      3V3----+ nSLEEP          B+ +------+ B+           |
|                           |             |                    |      +--------------+
|                           |      3V3----+ nRST            VM +-------------------+
|                           |             |                    |                   |
|                           |  3V3|GND----+ M2             GND +----------+        |
|                           |             |                    |          |        |
|                           |  3V3|GND----+ M1                 |          |        |
|                           |             |                    |          |        |
|                           |  3V3|GND----+ M0                 |      +---+--------+-----+
|                           |             |                    |      |  GND     +12V    |
|        STEP_MOTOR_GPIO_EN +------------>+ nEN                |      |   POWER SUPPLY   |
+---------------------------+             +--------------------+      +------------------+
```

The GPIO numbers used can be changed according to your board, by the macros `STEP_MOTOR_GPIO_EN`, `STEP_MOTOR_GPIO_DIR`, and `STEP_MOTOR_GPIO_STEP` defined in the [source file](main/main.c).

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash, and monitor the project.

(To exit the serial monitor, type `Ctrl-]`.)

## Project Structure and Code Overview

The project is organized as a minimal ESP-IDF C application for real-time stepper motor and EDM control. The main components are:

- **main/main.c**: Core application logic. Handles:
  - Stepper motor control using the ESP-IDF RMT peripheral and custom jog/cut logic (acceleration, deceleration, and jog phases)
  - GPIO configuration for stepper, jog buttons, limit switch, and start button
  - Main control loop for jogging, EDM cutting, and safety logic
  - Integration with ADC and PWM tasks
- **ADC.c** (referenced, not shown): Handles ADC sampling and gap voltage capture for EDM feedback
- **MCPWM task** (referenced, not shown): Controls the half-bridge IGBT PWM for power generation
- **stepper_motor_encoder.h**: Provides encoder configuration structures for RMT-based stepper control

### What the Code Does So Far

- Initializes all required GPIOs for stepper, jog, limit switch, and start button
- Configures RMT channel and encoders for smooth stepper motion (acceleration, deceleration, and jog)
- Implements a main loop that:
  - Allows jogging (manual movement) only if the limit switch is not triggered
  - Starts EDM cutting when the start button is pressed and not jogging
  - Uses ADC feedback to advance or retract the electrode based on gap voltage
  - Inhibits all movement if the limit switch is triggered (safety)
- Integrates with ADC and PWM tasks for feedback and power control
- Designed for flexibility and real-time safety, with clear separation of jog and cut logic

This structure allows for robust, real-time control of a stepper-driven EDM axis, with safety interlocks and feedback-based electrode positioning. The code is modular and ready for further expansion, such as improved motion profiles, external ADC support, or more advanced EDM power control.

---

## Notes on Code Evolution

This project started from an ESP-IDF stepper example, but has been heavily refactored:
- All remaining references to the original IDF stepper demo have been removed or replaced.
- The control logic is now fully custom, focused on EDM and jog/cut safety.
- The code is streamlined for clarity, maintainability, and real-world EDM use.

If you find any leftover comments or code fragments from the original ESP-IDF example, they can be safely removed or replaced with project-specific logic.

## Project Photo

Below is a photo of the current hardware setup:

![Project Hardware Setup](17c2c7c6de31dd8232fc0ded1c6dbf4.jpg)
