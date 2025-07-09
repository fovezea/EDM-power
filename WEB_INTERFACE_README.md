# EDM Power Control - Web Interface

## 🚀 Overview

Your EDM system now includes a complete web-based control interface that allows you to monitor and control your EDM machine remotely via WiFi.

## 📋 Features

### Real-time Monitoring
- **PWM Duty Cycle** - Live display and control (1-99%)
- **Gap Voltage** - Real-time ADC readings with voltage conversion
- **System Status** - Cutting status, limit switch, WiFi connection
- **Speed Parameters** - Jog speed, cut speed, stepper position
- **Uptime** - System running time

### Remote Control
- **Start/Stop EDM Cutting** - Remote control of cutting process
- **Jog Controls** - Up/down movement commands
- **Home Position** - Return to reference position
- **Duty Cycle Adjustment** - Real-time PWM control via slider

### Live Charting
- **Gap Voltage Chart** - Real-time plotting of gap voltage over time
- **Auto-scaling** - Adapts to voltage range
- **Data History** - Last 50 data points displayed

## 🔧 Setup Instructions

### 1. Configure WiFi
Edit `main/wifi_connect.h` and set your WiFi credentials:
```c
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
```

### 2. Build and Flash
```bash
idf.py build
idf.py flash monitor
```

### 3. Access the Web Interface
1. Connect your device to the same WiFi network as your ESP32
2. Look for the ESP32's IP address in the serial monitor output
3. Open your web browser and navigate to: `http://[ESP32_IP_ADDRESS]`

## 🎮 How to Use

### Web Interface Controls

#### **Status Cards**
- **PWM Duty Cycle**: Shows current duty cycle and provides a slider for adjustment
- **Gap Voltage**: Displays real-time gap voltage in millivolts and raw ADC value
- **System Status**: Shows cutting status, limit switch state, and WiFi connection
- **Speeds**: Displays jog speed, cut speed, and current stepper position

#### **Control Panel**
- **Start EDM Cut**: Begin the EDM cutting process
- **Stop EDM Cut**: Stop the EDM cutting process
- **Jog Up**: Move electrode up
- **Jog Down**: Move electrode down
- **Home Position**: Return to home position

#### **Real-time Chart**
- **Gap Voltage Chart**: Live plotting of gap voltage over time
- **Auto-scaling**: Automatically adjusts to show the full voltage range
- **Data History**: Maintains the last 50 data points

### Connection Status
- **Green**: Connected to ESP32 via WebSocket
- **Red**: Disconnected (will auto-reconnect)

## 🔌 Technical Details

### WebSocket Communication
- **Real-time Updates**: 1-second refresh rate
- **Bidirectional**: Commands from web to ESP32, status from ESP32 to web
- **Multiple Clients**: Support for up to 4 simultaneous connections
- **Auto-reconnection**: Client automatically reconnects if connection lost

### Integration with EDM System
- **Command Queue**: Thread-safe communication between web and main tasks
- **Status Synchronization**: Real-time updates from existing EDM code
- **Error Handling**: Graceful fallback if WiFi/web server fails
- **ESP-IDF v5.4.1 Compatible**: Uses latest APIs

## 📱 Mobile Support
The web interface is fully responsive and works on:
- Desktop computers
- Tablets
- Smartphones

## 🛠️ Troubleshooting

### WiFi Connection Issues
1. Check WiFi credentials in `wifi_connect.h`
2. Ensure ESP32 is within WiFi range
3. Check serial monitor for connection status

### Web Interface Not Loading
1. Verify ESP32 IP address in serial monitor
2. Check if ESP32 and device are on same network
3. Try refreshing the browser page

### Commands Not Working
1. Check WebSocket connection status (should be green)
2. Verify ESP32 is not in a critical state (limit switch triggered)
3. Check serial monitor for error messages

## 🔒 Security Notes
- The web interface is designed for local network use
- No authentication is implemented (add if needed for production)
- Consider using HTTPS for production environments

## 📊 Performance
- **Update Rate**: 1 second for status updates
- **Chart Data**: 50 data points maintained
- **Memory Usage**: ~8KB for web server task
- **CPU Usage**: Minimal impact on EDM performance

## 🎯 Next Steps
Consider adding these features:
- **User Authentication**: Login system for security
- **Data Logging**: Save cutting data to SD card
- **Email Alerts**: Notifications for system events
- **Advanced Charts**: Multiple parameter plotting
- **Recipe Management**: Save and load cutting parameters 