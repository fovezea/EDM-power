

# RMT Based Stepper Motor Smooth Controller
# MCPWM based pwm controller for EDM power supply


One RMT TX channel can use different encoders in sequence, which is useful to generate waveforms that have obvious multiple stages.

This example shows how to drive a stepper motor with a **STEP/DIR** interfaced controller (e.g. [DRV8825](https://www.ti.com/lit/ds/symlink/drv8825.pdf)) in a [smooth](https://en.wikipedia.org/wiki/Smoothstep) way. To smoothly drive a stepper motor, there're three phases: **Acceleration**, **Uniform** and **Deceleration**. Accordingly, this example implements two encoders so that RMT channel can generate the waveforms with different characteristics:

* `curve_encoder` is to encode the **Acceleration** and **Deceleration** phase
* `uniform_encoder` is to encode the ***Uniform** phase

## How to Use Example

````

````


Made changes.

The requested changes have been applied:

- Added `LIMIT_SWITCH_GPIO` and `START_CUT_GPIO` as new input pins.
- Both pins are now configured as inputs.
- Jogging is only allowed if the limit switch is ON (not triggered).
- EDM control only runs if not jogging, limit switch is ON, and start button is ON.
- If the limit switch is OFF, all movement is inhibited (including jogging and EDM control).

Loop now fully supports the new safety and control logic. 


Connection :

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

The GPIO number used in this example can be changed according to your board, by the macro `STEP_MOTOR_GPIO_EN`, `STEP_MOTOR_GPIO_DIR` and `STEP_MOTOR_GPIO_STEP` defined in the [source file](main/main.c).

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)
