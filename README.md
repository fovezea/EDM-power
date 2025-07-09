# EDM Power Control System

A comprehensive ESP32-based EDM (Electrical Discharge Machining) power supply and control system with real-time web interface, PID control, and persistent settings management.

## 🎯 **Project Goals**

- **Real-time EDM Control**: Precise electrode positioning with PID feedback
- **Web Interface**: Modern responsive web UI for monitoring and control
- **Persistent Settings**: NVS-based settings storage with web configuration
- **Safety Features**: Limit switch protection and emergency stops
- **Modular Design**: Clean separation of concerns for maintainability

### **EDM Philosophy & Approach**

EDM machines, due to the spark (plasma) not being fully understood and the lack of a mathematical model that fits the way we try to simplify the equations, are still esoteric in nature and there is still a lot of empirical approach. Combined with the variety of materials and electrodes, this creates a complex situation.

As far as I understand, there is not a single type of power source in an industrial machine, but rather a combination of several sources depending on the operation. The target is to have flexibility to control the same hardware to do as much as possible.

There is a lot of documentation and publications out there, but there is still a gap between implementation and theoretical approach.

## ✨ **Current Features**

### 🔧 **Core EDM Functionality**
- **Stepper Motor Control**: Smooth acceleration/deceleration with RMT peripheral
- **PID Control Loop**: Configurable proportional, integral, and derivative gains
- **Gap Voltage Monitoring**: Real-time ADC sampling with configurable blanking delay
- **PWM Power Generation**: Half-bridge IGBT control with dead time protection
- **Safety Interlocks**: Limit switch protection and emergency stop functionality

### 🌐 **Web Interface & Connectivity**
- **WiFi Station Mode**: Automatic connection with retry logic
- **Real-time Web UI**: Modern responsive interface accessible via browser
- **WebSocket Communication**: Live status updates and command control
- **Cross-platform**: Works on desktop, tablet, and mobile devices

### ⚙️ **Settings Management**
- **Persistent Storage**: NVS-based settings that survive power cycles
- **Web Configuration**: Real-time parameter adjustment via web interface
- **Default Values**: Automatic fallback to sensible defaults
- **Settings Reset**: One-click restore to factory defaults

### 📊 **Configurable Parameters**
- **PWM Duty Cycle**: 1-99% with real-time adjustment
- **ADC Blanking Delay**: 10-200 ticks for noise filtering
- **PID Parameters**: 
  - Kp (Proportional): 0.000-1.000
  - Ki (Integral): 0.000-1.000  
  - Kd (Derivative): 0.000-1.000
  - Setpoint: 0-4095 ADC value
- **Speed Settings**: Jog and cut speeds in mm/s

## 🚀 **Getting Started**

### **Prerequisites**
- ESP-IDF v5.4.1 or later
- ESP32 development board
- DRV8825 stepper motor driver
- 4-wire stepper motor
- Half-bridge IGBT module
- Limit switch and control buttons

### **Hardware Connections**

```
+---------------------------+             +--------------------+      +--------------+
|          ESP32 Board      |             |       DRV8825      |      |    4-wire    |
|                       GND +-------------+ GND                |      |     Step     |
|                           |             |                    |      |     Motor    |
|                       3V3 +-------------+ VDD             A+ +------+ A+           |
|                           |             |                    |      |              |
|       GPIO 2 (DIR)        +------------>+ DIR             A- +------+ A-           |
|                           |             |                    |      |              |
|       GPIO 4 (STEP)       +------------>+ STEP            B- +------+ B-           |
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
|       GPIO 0 (ENABLE)     +------------>+ nEN                |      |   POWER SUPPLY   |
+---------------------------+             +--------------------+      +------------------+

PWM Outputs:
- GPIO 16 (PWM0A) -> IGBT High Side
- GPIO 17 (PWM0B) -> IGBT Low Side
- GPIO 18 (CAP)   -> External Signal Capture

Control Inputs:
- GPIO 12 (JOG_UP)     -> Jog Up Button
- GPIO 13 (JOG_DOWN)   -> Jog Down Button  
- GPIO 14 (LIMIT)      -> Limit Switch
- GPIO 15 (START_CUT)  -> Start/Stop Button

ADC Input:
- GPIO 6 (ADC_CH6)     -> Gap Voltage Sensing
```

### **Build and Flash**

1. **Configure WiFi** (optional):
   ```bash
   idf.py menuconfig
   # Navigate to: Component config -> WiFi Configuration
   # Set your WiFi credentials
   ```

2. **Build and flash**:
   ```bash
   idf.py build
   idf.py -p [PORT] flash monitor
   ```

3. **Access Web Interface**:
   - Connect to the ESP32's WiFi network (if in AP mode)
   - Or connect to your WiFi and note the IP address from serial output
   - Open browser and navigate to: `http://[ESP32_IP_ADDRESS]`

## 📁 **Project Structure**

```
main/
├── main.c                 # Core application logic and task management
├── MCPWM_task.c          # PWM generation and PID control loop
├── ADC.c                 # ADC sampling and gap voltage capture
├── stepper_motor_encoder.c # RMT-based stepper motor control
├── wifi_connect.c        # WiFi station mode with auto-reconnect
├── web_server.c          # HTTP server and WebSocket communication
├── settings.c            # NVS-based persistent settings management
├── web_server.h          # Web interface definitions
├── wifi_connect.h        # WiFi connection interface
└── settings.h            # Settings management interface
```

## 🎛️ **Web Interface Features**

### **Real-time Monitoring**
- **Gap Voltage**: Live ADC readings with voltage conversion
- **PWM Duty Cycle**: Current duty cycle percentage
- **System Status**: Cutting state, limit switch, WiFi connection
- **Motor Position**: Current step position and speeds
- **PID Parameters**: Current Kp, Ki, Kd, and setpoint values

### **Control Panel**
- **EDM Control**: Start/Stop cutting operations
- **Manual Jogging**: Up/Down movement with safety limits
- **Home Position**: Return to reference position
- **Settings Reset**: Restore factory defaults

### **Parameter Configuration**
- **Duty Cycle Slider**: 1-99% PWM control
- **ADC Blanking Delay**: 10-200 ticks for noise filtering
- **PID Tuning**: Real-time adjustment of control parameters
- **Setpoint Control**: Target ADC value for gap control

## 🔧 **Technical Details**

### **RMT-Based Stepper Control**
One RMT TX channel can use different encoders in sequence, which is useful to generate waveforms that have multiple stages.

To smoothly drive a stepper motor, there are three phases: **Acceleration**, **Uniform**, and **Deceleration**. The code implements two encoders so that the RMT channel can generate waveforms with different characteristics:

* `curve_encoder` encodes the **Acceleration** and **Deceleration** phases
* `uniform_encoder` encodes the **Uniform** phase

### **PID Control Loop**
The system uses a PID controller to maintain optimal gap voltage:
- **Proportional (Kp)**: Responds to current error
- **Integral (Ki)**: Eliminates steady-state error
- **Derivative (Kd)**: Reduces overshoot and oscillations
- **Setpoint**: Target ADC value for gap control

### **Control Logic & Safety Features**

The main control loop implements comprehensive safety and operational logic:

- **Jogging Control**: Manual movement only allowed if limit switch is ON (not triggered)
- **EDM Cutting**: Only runs if not jogging, limit switch is ON, and start button is ON
- **Safety Inhibition**: If limit switch is OFF, all movement is inhibited (including jogging and EDM control)
- **Range Validation**: All parameters validated before application
- **Fault Logging**: Comprehensive error reporting

The loop now fully supports the new safety and control logic with clear separation of jog and cut operations.

### **Safety Features**
- **Limit Switch Protection**: Inhibits all movement when triggered
- **Emergency Stop**: Immediate halt of all operations
- **Range Validation**: All parameters validated before application
- **Fault Logging**: Comprehensive error reporting

### **Performance Characteristics**
- **Update Rate**: 50Hz PID control loop
- **ADC Resolution**: 12-bit (0-4095)
- **PWM Frequency**: 20kHz with configurable duty cycle
- **Stepper Resolution**: 200 steps/revolution
- **WebSocket Latency**: <100ms for real-time control

## 🛠️ **Configuration**

### **WiFi Setup**
Edit `main/wifi_connect.c` to configure your WiFi credentials:
```c
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASS "your_wifi_password"
```

### **GPIO Configuration**
Modify GPIO assignments in `main/main.c`:
```c
#define STEP_MOTOR_GPIO_EN       0
#define STEP_MOTOR_GPIO_DIR      2
#define STEP_MOTOR_GPIO_STEP     4
#define JOG_UP_GPIO   12
#define JOG_DOWN_GPIO 13
#define LIMIT_SWITCH_GPIO 14
#define START_CUT_GPIO    15
```

### **Default Settings**
Adjust default values in `main/settings.h`:
```c
#define DEFAULT_DUTY_PERCENT 40
#define DEFAULT_ADC_BLANKING_DELAY 1
#define DEFAULT_PID_KP 0.05f
#define DEFAULT_PID_KI 0.01f
#define DEFAULT_PID_KD 0.0f
#define DEFAULT_PID_SETPOINT 1500
```

## 📈 **Future Enhancements**

- **External ADC Support**: Higher precision voltage measurement
- **Multi-axis Control**: Support for X, Y, Z positioning
- **G-code Interpreter**: CNC-style programming interface
- **Data Logging**: Historical performance tracking
- **Advanced PID Tuning**: Auto-tuning algorithms
- **Material Database**: Pre-configured settings for different materials

## 🤝 **Contributing**

This project is actively developed for EDM applications. Contributions are welcome for:
- Hardware improvements
- Control algorithm enhancements
- Web interface features
- Documentation updates

## 📄 **License**

This project is open source. See LICENSE file for details.

## 📸 **Project Hardware Setup**

Below is a photo of the current hardware setup:

![Project Hardware Setup](17c2c7c6de31dd8232fc0ded1c6dbf4.jpg)

---

**Note**: This hardware schematic is writen in Kicad 9 but so far is just  power stage control without the reset of the generator. It will be added soon
**Note**: This system is designed for educational and experimental use. Always follow proper safety procedures when working with high-voltage EDM systems.
