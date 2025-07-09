/*
 * Web Server with WebSocket for EDM Power Project
 * ESP-IDF v5.4.1 compatible
 */

#include "web_server.h"
#include "wifi_connect.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "settings.h"
#include <string.h>
#include "lwip/sockets.h"

// Command structure for web interface
typedef struct {
    char command[32];
    int value;
} web_command_t;

// External declaration for command queue from main.c
extern QueueHandle_t command_queue;

static const char *TAG = "web_server";

// Global variables
static httpd_handle_t server = NULL;
static client_t clients[MAX_CLIENTS];
static int client_count = 0;
static edm_status_t current_status = {0};
static SemaphoreHandle_t status_mutex = NULL;
static uint32_t system_start_time = 0;

// Simple HTML page for EDM control interface
static const char *html_page = 
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"    <title>EDM Power Control</title>\n"
"    <meta charset=\"utf-8\">\n"
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
"    <style>\n"
"        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }\n"
"        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }\n"
"        .status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin: 20px 0; }\n"
"        .status-card { background: #f8f9fa; padding: 15px; border-radius: 8px; border-left: 4px solid #007bff; }\n"
"        .control-panel { background: #e9ecef; padding: 20px; border-radius: 8px; margin: 20px 0; }\n"
"        .button { background: #007bff; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; margin: 5px; }\n"
"        .button:hover { background: #0056b3; }\n"
"        .button.danger { background: #dc3545; }\n"
"        .button.danger:hover { background: #c82333; }\n"
"        .button.success { background: #28a745; }\n"
"        .button.success:hover { background: #218838; }\n"
"        .slider { width: 100%; margin: 10px 0; }\n"
"        .value-display { font-weight: bold; color: #007bff; }\n"
"        .connection-status { padding: 10px; border-radius: 5px; margin: 10px 0; }\n"
"        .connected { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }\n"
"        .disconnected { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }\n"
"    </style>\n"
"</head>\n"
"<body>\n"
"    <div class=\"container\">\n"
"        <h1>EDM Power Control System</h1>\n"
"        \n"
"        <div id=\"connection-status\" class=\"connection-status disconnected\">\n"
"            <strong>Status:</strong> <span id=\"status-text\">Disconnected</span>\n"
"        </div>\n"
"\n"
"        <div class=\"status-grid\">\n"
"            <div class=\"status-card\">\n"
"                <h3>PWM Duty Cycle</h3>\n"
"                <div class=\"value-display\" id=\"duty-percent\">0%</div>\n"
"                <input type=\"range\" min=\"1\" max=\"99\" value=\"40\" class=\"slider\" id=\"duty-slider\">\n"
"                <button class=\"button\" onclick=\"setDuty()\">Set Duty</button>\n"
"                <button class=\"button\" id=\"pid-toggle-btn\" onclick=\"togglePID()\">Enable PID</button>\n"
"            </div>\n"
"            \n"
"            <div class=\"status-card\">\n"
"                <h3>Gap Voltage</h3>\n"
"                <div class=\"value-display\" id=\"gap-voltage\">0 mV</div>\n"
"                <div>ADC: <span id=\"adc-value\">0</span></div>\n"
"            </div>\n"
"            \n"
"            <div class=\"status-card\">\n"
"                <h3>ADC Blanking Delay</h3>\n"
"                <div class=\"value-display\" id=\"blanking-delay\">10 ticks</div>\n"
"                <input type=\"range\" min=\"10\" max=\"200\" value=\"10\" class=\"slider\" id=\"blanking-delay-slider\">\n"
"                <button class=\"button\" onclick=\"setBlankingDelay()\">Set Delay</button>\n"
"            </div>\n"
"            \n"
"            <div class=\"status-card\">\n"
"                <h3>PID Control</h3>\n"
"                <div>Kp: <span id=\"pid-kp\" class=\"value-display\">0.050</span></div>\n"
"                <input type=\"range\" min=\"0\" max=\"1\" step=\"0.001\" value=\"0.050\" class=\"slider\" id=\"pid-kp-slider\">\n"
"                <div>Ki: <span id=\"pid-ki\" class=\"value-display\">0.010</span></div>\n"
"                <input type=\"range\" min=\"0\" max=\"1\" step=\"0.001\" value=\"0.010\" class=\"slider\" id=\"pid-ki-slider\">\n"
"                <div>Kd: <span id=\"pid-kd\" class=\"value-display\">0.000</span></div>\n"
"                <input type=\"range\" min=\"0\" max=\"1\" step=\"0.001\" value=\"0.000\" class=\"slider\" id=\"pid-kd-slider\">\n"
"                <div>Setpoint: <span id=\"pid-setpoint\" class=\"value-display\">1500</span></div>\n"
"                <input type=\"range\" min=\"0\" max=\"4095\" step=\"1\" value=\"1500\" class=\"slider\" id=\"pid-setpoint-slider\">\n"
"                <button class=\"button\" onclick=\"setPID()\">Set PID</button>\n"
"            </div>\n"
"            \n"
"            <div class=\"status-card\">\n"
"                <h3>System Status</h3>\n"
"                <div>Cutting: <span id=\"cutting-status\" class=\"value-display\">Stopped</span></div>\n"
"                <div>Limit Switch: <span id=\"limit-status\" class=\"value-display\">OK</span></div>\n"
"                <div>WiFi: <span id=\"wifi-status\" class=\"value-display\">Connected</span></div>\n"
"            </div>\n"
"            \n"
"            <div class=\"status-card\">\n"
"                <h3>Speeds</h3>\n"
"                <div>Jog Speed: <span id=\"jog-speed\" class=\"value-display\">1.0 mm/s</span></div>\n"
"                <div>Cut Speed: <span id=\"cut-speed\" class=\"value-display\">0.1 mm/s</span></div>\n"
"                <div>Position: <span id=\"step-position\" class=\"value-display\">0 steps</span></div>\n"
"            </div>\n"
"        </div>\n"
"\n"
"        <div class=\"control-panel\">\n"
"            <h3>Control Panel</h3>\n"
"            <div style=\"margin-bottom: 15px; padding: 10px; background: #fff3cd; border: 1px solid #ffeaa7; border-radius: 5px;\">\n"
"                <strong>Control Mode:</strong> <span id=\"control-mode\" class=\"value-display\">Physical Buttons</span>\n"
"                <button class=\"button\" id=\"control-toggle-btn\" onclick=\"toggleWebControl()\">Take Web Control</button>\n"
"            </div>\n"
"            <button class=\"button success\" onclick=\"startCut()\">Start EDM Cut</button>\n"
"            <button class=\"button danger\" onclick=\"stopCut()\">Stop EDM Cut</button>\n"
"            <button class=\"button\" onclick=\"jogUp()\">Jog Up</button>\n"
"            <button class=\"button\" onclick=\"jogDown()\">Jog Down</button>\n"
"            <button class=\"button\" onclick=\"homePosition()\">Home Position</button>\n"
"            <button class=\"button danger\" onclick=\"resetSettings()\">Reset Settings</button>\n"
"            <button class=\"button\" onclick=\"checkPIDState()\">Check PID State</button>\n"
"            <button class=\"button\" onclick=\"debugSetDuty50()\">Set Duty to 50%</button>\n"
"            <button class=\"button\" onclick=\"debugSetAdc2000()\">Set ADC to 2000</button>\n"
"            <button class=\"button\" onclick=\"debugDisableAdcOverride()\">Disable ADC Override</button>\n"
"            <button class=\"button\" onclick=\"debugAllValues()\">Debug All Values</button>\n"
"        </div>\n"
"    </div>\n"
"\n"
"    <script>\n"
"        console.log('JavaScript starting...');\n"
"        var updateTimer = null;\n"
"        var isConnected = false;\n"
"        var sliderActive = {\n"
"            duty: false,\n"
"            blanking: false,\n"
"            kp: false,\n"
"            ki: false,\n"
"            kd: false,\n"
"            setpoint: false\n"
"        };\n"
"        var lastUserValues = {\n"
"            duty: null,\n"
"            blanking: null,\n"
"            kp: null,\n"
"            ki: null,\n"
"            kd: null,\n"
"            setpoint: null\n"
"        };\n"
"\n"
"        function startPolling() {\n"
"            console.log('Starting polling updates');\n"
"            updateStatus();\n"
"            updateTimer = setInterval(updateStatus, 1000); // Update every second\n"
"        }\n"
"\n"
"        function stopPolling() {\n"
"            if (updateTimer) {\n"
"                clearInterval(updateTimer);\n"
"                updateTimer = null;\n"
"            }\n"
"            isConnected = false;\n"
"            document.getElementById('status-text').textContent = 'Disconnected';\n"
"            document.getElementById('connection-status').className = 'connection-status disconnected';\n"
"        }\n"
"\n"
"        function updateStatus() {\n"
"            fetch('/status')\n"
"                .then(response => response.json())\n"
"                .then(data => {\n"
"                    console.log('Received status update:', data);\n"
"                    updateDisplay(data);\n"
"                    if (!isConnected) {\n"
"                        isConnected = true;\n"
"                        document.getElementById('status-text').textContent = 'Connected (Polling)';\n"
"                        document.getElementById('connection-status').className = 'connection-status connected';\n"
"                    }\n"
"                })\n"
"                .catch(error => {\n"
"                    console.error('Status update failed:', error);\n"
"                    stopPolling();\n"
"                    // Retry after 2 seconds\n"
"                    setTimeout(startPolling, 2000);\n"
"                });\n"
"        }\n"
"\n"
"        function updateDisplay(data) {\n"
"            console.log('Status update received:', data);\n"
"            \n"
"            // Only update duty cycle display if slider is not being actively used and user hasn't set a value\n"
"            if (!sliderActive.duty && lastUserValues.duty === null) {\n"
"                document.getElementById('duty-percent').textContent = data.duty_percent.toFixed(1) + '%';\n"
"                document.getElementById('duty-slider').value = data.duty_percent;\n"
"            } else if (lastUserValues.duty !== null) {\n"
"                // User has set a value, keep it\n"
"                document.getElementById('duty-percent').textContent = lastUserValues.duty.toFixed(1) + '%';\n"
"                document.getElementById('duty-slider').value = lastUserValues.duty;\n"
"            }\n"
"            \n"
"            document.getElementById('gap-voltage').textContent = data.gap_voltage.toFixed(1) + ' mV';\n"
"            document.getElementById('adc-value').textContent = data.adc_value;\n"
"            document.getElementById('cutting-status').textContent = data.is_cutting ? 'Running' : 'Stopped';\n"
"            document.getElementById('limit-status').textContent = data.limit_switch_triggered ? 'TRIGGERED' : 'OK';\n"
"            document.getElementById('wifi-status').textContent = data.wifi_connected ? 'Connected' : 'Disconnected';\n"
"            document.getElementById('jog-speed').textContent = data.jog_speed.toFixed(1) + ' mm/s';\n"
"            document.getElementById('cut-speed').textContent = data.cut_speed.toFixed(2) + ' mm/s';\n"
"            document.getElementById('step-position').textContent = data.step_position + ' steps';\n"
"            \n"
"            // Only update blanking delay display if slider is not being actively used and user hasn't set a value\n"
"            if (!sliderActive.blanking && lastUserValues.blanking === null) {\n"
"                document.getElementById('blanking-delay').textContent = data.adc_blanking_delay + ' ticks';\n"
"                document.getElementById('blanking-delay-slider').value = data.adc_blanking_delay;\n"
"            } else if (lastUserValues.blanking !== null) {\n"
"                // User has set a value, keep it\n"
"                document.getElementById('blanking-delay').textContent = lastUserValues.blanking + ' ticks';\n"
"                document.getElementById('blanking-delay-slider').value = lastUserValues.blanking;\n"
"            }\n"
"            \n"
"            // Only update PID displays if sliders are not being actively used and user hasn't set a value\n"
"            if (!sliderActive.kp && lastUserValues.kp === null) {\n"
"                document.getElementById('pid-kp').textContent = data.pid_kp.toFixed(3);\n"
"                document.getElementById('pid-kp-slider').value = data.pid_kp;\n"
"            } else if (lastUserValues.kp !== null) {\n"
"                // User has set a value, keep it\n"
"                document.getElementById('pid-kp').textContent = lastUserValues.kp.toFixed(3);\n"
"                document.getElementById('pid-kp-slider').value = lastUserValues.kp;\n"
"            }\n"
"            \n"
"            if (!sliderActive.ki && lastUserValues.ki === null) {\n"
"                document.getElementById('pid-ki').textContent = data.pid_ki.toFixed(3);\n"
"                document.getElementById('pid-ki-slider').value = data.pid_ki;\n"
"            } else if (lastUserValues.ki !== null) {\n"
"                // User has set a value, keep it\n"
"                document.getElementById('pid-ki').textContent = lastUserValues.ki.toFixed(3);\n"
"                document.getElementById('pid-ki-slider').value = lastUserValues.ki;\n"
"            }\n"
"            \n"
"            if (!sliderActive.kd && lastUserValues.kd === null) {\n"
"                document.getElementById('pid-kd').textContent = data.pid_kd.toFixed(3);\n"
"                document.getElementById('pid-kd-slider').value = data.pid_kd;\n"
"            } else if (lastUserValues.kd !== null) {\n"
"                // User has set a value, keep it\n"
"                document.getElementById('pid-kd').textContent = lastUserValues.kd.toFixed(3);\n"
"                document.getElementById('pid-kd-slider').value = lastUserValues.kd;\n"
"            }\n"
"            \n"
"            if (!sliderActive.setpoint && lastUserValues.setpoint === null) {\n"
"                document.getElementById('pid-setpoint').textContent = data.pid_setpoint;\n"
"                document.getElementById('pid-setpoint-slider').value = data.pid_setpoint;\n"
"            } else if (lastUserValues.setpoint !== null) {\n"
"                // User has set a value, keep it\n"
"                document.getElementById('pid-setpoint').textContent = lastUserValues.setpoint;\n"
"                document.getElementById('pid-setpoint-slider').value = lastUserValues.setpoint;\n"
"            }\n"
"            \n"
"            // Update PID toggle button text\n"
"            var pidBtn = document.getElementById('pid-toggle-btn');\n"
"            if (data.pid_control_enabled) {\n"
"                pidBtn.textContent = 'Disable PID';\n"
"                pidBtn.className = 'button danger';\n"
"            } else {\n"
"                pidBtn.textContent = 'Enable PID';\n"
"                pidBtn.className = 'button success';\n"
"            }\n"
"            \n"
"            // Update control mode display\n"
"            var controlMode = document.getElementById('control-mode');\n"
"            var controlBtn = document.getElementById('control-toggle-btn');\n"
"            if (data.web_control_active) {\n"
"                controlMode.textContent = 'Web Interface';\n"
"                controlBtn.textContent = 'Release Control';\n"
"                controlBtn.className = 'button danger';\n"
"            } else {\n"
"                controlMode.textContent = 'Physical Buttons';\n"
"                controlBtn.textContent = 'Take Web Control';\n"
"                controlBtn.className = 'button success';\n"
"            }\n"
"        }\n"
"\n"
"        function sendCommand(command, value) {\n"
"            var cmd = { command: command };\n"
"            if (value !== null) cmd.value = value;\n"
"            \n"
"            console.log('Sending command:', JSON.stringify(cmd));\n"
"            \n"
"            fetch('/command', {\n"
"                method: 'POST',\n"
"                headers: {\n"
"                    'Content-Type': 'application/json',\n"
"                },\n"
"                body: JSON.stringify(cmd)\n"
"            })\n"
"            .then(response => response.json())\n"
"            .then(data => {\n"
"                console.log('Command sent successfully:', command);\n"
"            })\n"
"            .catch(error => {\n"
"                console.error('Command failed:', error);\n"
"            });\n"
"        }\n"
"\n"
"        function setDuty() {\n"
"            var duty = document.getElementById('duty-slider').value;\n"
"            sendCommand('set_duty', parseInt(duty));\n"
"            lastUserValues.duty = null; // Allow polling to take over\n"
"        }\n"
"\n"
"        function setBlankingDelay() {\n"
"            var delay = document.getElementById('blanking-delay-slider').value;\n"
"            sendCommand('set_blanking_delay', parseInt(delay));\n"
"            lastUserValues.blanking = null; // Allow polling to take over\n"
"        }\n"
"\n"
"        function setPID() {\n"
"            var kp = document.getElementById('pid-kp-slider').value;\n"
"            var ki = document.getElementById('pid-ki-slider').value;\n"
"            var kd = document.getElementById('pid-kd-slider').value;\n"
"            var setpoint = document.getElementById('pid-setpoint-slider').value;\n"
"            \n"
"            console.log('Setting PID values: Kp=' + kp + ', Ki=' + ki + ', Kd=' + kd + ', Setpoint=' + setpoint);\n"
"            \n"
"            // Send commands sequentially with delays\n"
"            sendCommand('set_pid_kp', parseFloat(kp));\n"
"            setTimeout(function() {\n"
"                sendCommand('set_pid_ki', parseFloat(ki));\n"
"                setTimeout(function() {\n"
"                    sendCommand('set_pid_kd', parseFloat(kd));\n"
"                    setTimeout(function() {\n"
"                        sendCommand('set_pid_setpoint', parseInt(setpoint));\n"
"                        setTimeout(function() {\n"
"                            sendCommand('save_pid_settings');\n"
"                        }, 100);\n"
"                    }, 100);\n"
"                }, 100);\n"
"            }, 100);\n"
"            \n"
"            // Allow polling to take over for all PID values\n"
"            console.log('Resetting lastUserValues for PID sliders');\n"
"            lastUserValues.kp = null;\n"
"            lastUserValues.ki = null;\n"
"            lastUserValues.kd = null;\n"
"            lastUserValues.setpoint = null;\n"
"        }\n"
"\n"
"        function startCut() {\n"
"            sendCommand('start_cut');\n"
"        }\n"
"\n"
"        function stopCut() {\n"
"            sendCommand('stop_cut');\n"
"        }\n"
"\n"
"        function jogUp() {\n"
"            sendCommand('jog_up');\n"
"        }\n"
"\n"
"        function jogDown() {\n"
"            sendCommand('jog_down');\n"
"        }\n"
"\n"
"        function homePosition() {\n"
"            sendCommand('home_position');\n"
"        }\n"
"\n"
"        function resetSettings() {\n"
"            if (confirm('Are you sure you want to reset all settings to defaults?')) {\n"
"                sendCommand('reset_settings');\n"
"            }\n"
"        }\n"
"\n"
"        function disablePID() {\n"
"            sendCommand('disable_pid');\n"
"        }\n"
"\n"
"        function enablePID() {\n"
"            sendCommand('enable_pid');\n"
"        }\n"
"\n"
"        function togglePID() {\n"
"            var btn = document.getElementById('pid-toggle-btn');\n"
"            if (btn.textContent.includes('Enable')) {\n"
"                sendCommand('enable_pid');\n"
"            } else {\n"
"                sendCommand('disable_pid');\n"
"            }\n"
"        }\n"
"\n"
"        function checkPIDState() {\n"
"            sendCommand('get_pid_state');\n"
"        }\n"
"\n"
"        function debugSetDuty50() {\n"
"            sendCommand('debug_set_duty', 50);\n"
"        }\n"
"\n"
"        function debugSetAdc2000() {\n"
"            sendCommand('debug_set_adc', 2000);\n"
"        }\n"
"\n"
"        function debugDisableAdcOverride() {\n"
"            sendCommand('debug_disable_adc_override');\n"
"        }\n"
"\n"
"        function debugAllValues() {\n"
"            sendCommand('debug_all_values');\n"
"        }\n"
"\n"
"        function toggleWebControl() {\n"
"            var isWebControl = document.getElementById('control-mode').textContent === 'Web Interface';\n"
"            sendCommand('web_control', isWebControl ? 0 : 1);\n"
"        }\n"
"\n"
"        console.log('Page loaded, starting polling updates...');\n"
"        startPolling();\n"
"        \n"
"        console.log('Setting up event listeners...');\n"
"        \n"
"        // Duty cycle slider\n"
"        var dutySlider = document.getElementById('duty-slider');\n"
"        dutySlider.addEventListener('mousedown', function() { sliderActive.duty = true; });\n"
"        dutySlider.addEventListener('mouseup', function() { \n"
"            sliderActive.duty = false; \n"
"            lastUserValues.duty = parseInt(this.value);\n"
"        });\n"
"        dutySlider.addEventListener('input', function() {\n"
"            document.getElementById('duty-percent').textContent = this.value + '%';\n"
"        });\n"
"        \n"
"        // Blanking delay slider\n"
"        var blankingSlider = document.getElementById('blanking-delay-slider');\n"
"        blankingSlider.addEventListener('mousedown', function() { sliderActive.blanking = true; });\n"
"        blankingSlider.addEventListener('mouseup', function() { \n"
"            sliderActive.blanking = false; \n"
"            lastUserValues.blanking = parseInt(this.value);\n"
"        });\n"
"        blankingSlider.addEventListener('input', function() {\n"
"            document.getElementById('blanking-delay').textContent = this.value + ' ticks';\n"
"        });\n"
"        \n"
"        // PID Kp slider\n"
"        var kpSlider = document.getElementById('pid-kp-slider');\n"
"        kpSlider.addEventListener('mousedown', function() { sliderActive.kp = true; });\n"
"        kpSlider.addEventListener('mouseup', function() { \n"
"            sliderActive.kp = false; \n"
"            lastUserValues.kp = parseFloat(this.value);\n"
"        });\n"
"        kpSlider.addEventListener('input', function() {\n"
"            document.getElementById('pid-kp').textContent = parseFloat(this.value).toFixed(3);\n"
"        });\n"
"        \n"
"        // PID Ki slider\n"
"        var kiSlider = document.getElementById('pid-ki-slider');\n"
"        kiSlider.addEventListener('mousedown', function() { sliderActive.ki = true; });\n"
"        kiSlider.addEventListener('mouseup', function() { \n"
"            sliderActive.ki = false; \n"
"            lastUserValues.ki = parseFloat(this.value);\n"
"        });\n"
"        kiSlider.addEventListener('input', function() {\n"
"            document.getElementById('pid-ki').textContent = parseFloat(this.value).toFixed(3);\n"
"        });\n"
"        \n"
"        // PID Kd slider\n"
"        var kdSlider = document.getElementById('pid-kd-slider');\n"
"        kdSlider.addEventListener('mousedown', function() { sliderActive.kd = true; });\n"
"        kdSlider.addEventListener('mouseup', function() { \n"
"            sliderActive.kd = false; \n"
"            lastUserValues.kd = parseFloat(this.value);\n"
"        });\n"
"        kdSlider.addEventListener('input', function() {\n"
"            document.getElementById('pid-kd').textContent = parseFloat(this.value).toFixed(3);\n"
"        });\n"
"        \n"
"        // PID Setpoint slider\n"
"        var setpointSlider = document.getElementById('pid-setpoint-slider');\n"
"        setpointSlider.addEventListener('mousedown', function() { sliderActive.setpoint = true; });\n"
"        setpointSlider.addEventListener('mouseup', function() { \n"
"            sliderActive.setpoint = false; \n"
"            lastUserValues.setpoint = parseInt(this.value);\n"
"        });\n"
"        setpointSlider.addEventListener('input', function() {\n"
"            document.getElementById('pid-setpoint').textContent = this.value;\n"
"        });\n"
"    </script>\n"
"</body>\n"
"</html>";



// Create WebSocket frame
static esp_err_t create_websocket_frame(const char *payload, size_t payload_len, char *frame, size_t *frame_len) {
    frame[0] = 0x81; // FIN + text frame
    frame[1] = payload_len & 0x7F;
    
    size_t header_len = 2;
    if (payload_len > 125) {
        frame[1] = 126;
        frame[2] = (payload_len >> 8) & 0xFF;
        frame[3] = payload_len & 0xFF;
        header_len = 4;
    }
    
    memcpy(frame + header_len, payload, payload_len);
    *frame_len = header_len + payload_len;
    
    return ESP_OK;
}

// HTTP GET handler for main page
static esp_err_t root_get_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Serving main page");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

// Test handler for WebSocket debugging
static esp_err_t test_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Test endpoint called");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "WebSocket test endpoint working", 30);
    return ESP_OK;
}

// Catch-all handler for unknown requests
// WebSocket upgrade handler
// Status endpoint for polling-based updates
static esp_err_t status_handler(httpd_req_t *req) {
    
    // Create JSON response with current status
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "duty_percent", duty_percent);
    cJSON_AddNumberToObject(json, "adc_value", adc_value_on_capture);
    cJSON_AddNumberToObject(json, "gap_voltage", (float)adc_value_on_capture * 3.3f / 4095.0f * 1000.0f);
    cJSON_AddBoolToObject(json, "is_cutting", get_edm_cutting_status());
    cJSON_AddBoolToObject(json, "limit_switch_triggered", false); // TODO: get from main task
    cJSON_AddNumberToObject(json, "jog_speed", 1.0f); // TODO: get from main task
    cJSON_AddNumberToObject(json, "cut_speed", 0.1f); // TODO: get from main task
    cJSON_AddNumberToObject(json, "step_position", 0); // TODO: get from main task
    cJSON_AddBoolToObject(json, "wifi_connected", wifi_is_connected());
    cJSON_AddNumberToObject(json, "uptime_seconds", (esp_timer_get_time() / 1000000) - system_start_time);
    cJSON_AddNumberToObject(json, "adc_blanking_delay", adc_blanking_delay_ticks);
    cJSON_AddNumberToObject(json, "pid_kp", pid_kp);
    cJSON_AddNumberToObject(json, "pid_ki", pid_ki);
    cJSON_AddNumberToObject(json, "pid_kd", pid_kd);
    cJSON_AddNumberToObject(json, "pid_setpoint", pid_setpoint);
    cJSON_AddBoolToObject(json, "pid_control_enabled", pid_control_enabled);
    cJSON_AddBoolToObject(json, "web_control_active", get_web_control_active());
    
    char *json_str = cJSON_Print(json);
    if (json_str) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
        httpd_resp_send(req, json_str, strlen(json_str));
        free(json_str);
    }
    
    cJSON_Delete(json);
    return ESP_OK;
}

// Command processing - now sends commands to main task via queue
__attribute__((used)) static void process_command(const char *command, const char *value) {
    ESP_LOGI(TAG, "Processing command: %s, value: %s", command, value ? value : "null");
    
    static edm_settings_t current_settings;
    
    // For now, we'll handle duty cycle directly here since it's a simple variable
    if (strcmp(command, "set_duty") == 0 && value) {
        int duty = atoi(value);
        if (duty >= 1 && duty <= 99) {
            int old_duty = duty_percent;
            duty_percent = duty;
            ESP_LOGI(TAG, "Duty cycle set to %d%% (was %d%%)", duty, old_duty);
            
            // Save to settings
            if (settings_load(&current_settings) == ESP_OK) {
                current_settings.duty_percent = duty;
                settings_save(&current_settings);
                ESP_LOGI(TAG, "Duty cycle saved to settings");
            }
        }
    }
    // Handle ADC blanking delay setting
    else if (strcmp(command, "set_blanking_delay") == 0 && value) {
        int delay = atoi(value);
        if (delay >= 1 && delay <= 200) { // Limit to reasonable range
            adc_blanking_delay_ticks = delay;
            ESP_LOGI(TAG, "ADC blanking delay set to %d ticks", delay);
            
            // Save to settings
            if (settings_load(&current_settings) == ESP_OK) {
                current_settings.adc_blanking_delay = delay;
                settings_save(&current_settings);
            }
        }
    }
    // Handle PID parameter settings
    else if (strcmp(command, "set_pid_kp") == 0) {
        if (value && strlen(value) > 0) {
            float kp = atof(value);
            if (kp >= 0.0f && kp <= 10.0f) {
                float old_kp = pid_kp;
                pid_kp = kp;
                ESP_LOGI(TAG, "PID Kp set to %.3f (was %.3f)", kp, old_kp);
            } else {
                ESP_LOGW(TAG, "Invalid Kp value: %s", value);
            }
        } else {
            ESP_LOGW(TAG, "Kp value is null or empty");
        }
    }
    else if (strcmp(command, "set_pid_ki") == 0) {
        if (value && strlen(value) > 0) {
            float ki = atof(value);
            if (ki >= 0.0f && ki <= 10.0f) {
                pid_ki = ki;
                ESP_LOGI(TAG, "PID Ki set to %.3f", ki);
            } else {
                ESP_LOGW(TAG, "Invalid Ki value: %s", value);
            }
        } else {
            ESP_LOGW(TAG, "Ki value is null or empty");
        }
    }
    else if (strcmp(command, "set_pid_kd") == 0) {
        if (value && strlen(value) > 0) {
            float kd = atof(value);
            if (kd >= 0.0f && kd <= 10.0f) {
                pid_kd = kd;
                ESP_LOGI(TAG, "PID Kd set to %.3f", kd);
            } else {
                ESP_LOGW(TAG, "Invalid Kd value: %s", value);
            }
        } else {
            ESP_LOGW(TAG, "Kd value is null or empty");
        }
    }
    else if (strcmp(command, "set_pid_setpoint") == 0) {
        if (value && strlen(value) > 0) {
            int setpoint = atoi(value);
            if (setpoint >= 0 && setpoint <= 4095) {
                pid_setpoint = setpoint;
                ESP_LOGI(TAG, "PID setpoint set to %d", setpoint);
            } else {
                ESP_LOGW(TAG, "Invalid setpoint value: %s", value);
            }
        } else {
            ESP_LOGW(TAG, "Setpoint value is null or empty");
        }
    }
    // Handle PID settings save
    else if (strcmp(command, "save_pid_settings") == 0) {
        ESP_LOGI(TAG, "Saving PID settings to NVS");
        ESP_LOGI(TAG, "Current memory values: Kp=%.3f, Ki=%.3f, Kd=%.3f, Setpoint=%d",
                 pid_kp, pid_ki, pid_kd, pid_setpoint);
        if (settings_load(&current_settings) == ESP_OK) {
            ESP_LOGI(TAG, "Loaded from NVS: Kp=%.3f, Ki=%.3f, Kd=%.3f, Setpoint=%d",
                     current_settings.pid_kp, current_settings.pid_ki, current_settings.pid_kd, current_settings.pid_setpoint);
            current_settings.pid_kp = pid_kp;
            current_settings.pid_ki = pid_ki;
            current_settings.pid_kd = pid_kd;
            current_settings.pid_setpoint = pid_setpoint;
            settings_save(&current_settings);
            ESP_LOGI(TAG, "PID settings saved: Kp=%.3f, Ki=%.3f, Kd=%.3f, Setpoint=%d",
                     pid_kp, pid_ki, pid_kd, pid_setpoint);
        }
    }
    // Handle settings reset
    else if (strcmp(command, "reset_settings") == 0) {
        ESP_LOGI(TAG, "Resetting settings to defaults");
        if (settings_reset_to_defaults() == ESP_OK) {
            // Reload settings and update variables
            if (settings_load(&current_settings) == ESP_OK) {
                duty_percent = current_settings.duty_percent;
                adc_blanking_delay_ticks = current_settings.adc_blanking_delay;
                pid_kp = current_settings.pid_kp;
                pid_ki = current_settings.pid_ki;
                pid_kd = current_settings.pid_kd;
                pid_setpoint = current_settings.pid_setpoint;
                pid_control_enabled = current_settings.pid_control_enabled;
                ESP_LOGI(TAG, "Settings reset and reloaded");
            }
        }
    }
    // Handle ping command for connection testing
    else if (strcmp(command, "ping") == 0) {
        ESP_LOGI(TAG, "Received ping command, connection is working");
    }
    // Handle PID state query
    else if (strcmp(command, "get_pid_state") == 0) {
        ESP_LOGI(TAG, "Current PID state: enabled=%s, duty_percent=%d", 
                 pid_control_enabled ? "true" : "false", duty_percent);
    }
    // Handle debug all values query
    else if (strcmp(command, "debug_all_values") == 0) {
        ESP_LOGI(TAG, "DEBUG ALL VALUES:");
        ESP_LOGI(TAG, "  duty_percent = %d", duty_percent);
        ESP_LOGI(TAG, "  adc_blanking_delay_ticks = %d", adc_blanking_delay_ticks);
        ESP_LOGI(TAG, "  pid_kp = %.3f", pid_kp);
        ESP_LOGI(TAG, "  pid_ki = %.3f", pid_ki);
        ESP_LOGI(TAG, "  pid_kd = %.3f", pid_kd);
        ESP_LOGI(TAG, "  pid_setpoint = %d", pid_setpoint);
        ESP_LOGI(TAG, "  pid_control_enabled = %s", pid_control_enabled ? "true" : "false");
    }
    // Handle debug duty cycle set
    else if (strcmp(command, "debug_set_duty") == 0) {
        if (value && strlen(value) > 0) {
            int duty = atoi(value);
            if (duty >= 1 && duty <= 99) {
                int old_duty = duty_percent;
                duty_percent = duty;
                ESP_LOGI(TAG, "DEBUG: Duty cycle manually set to %d%% (was %d%%)", duty, old_duty);
            } else {
                ESP_LOGW(TAG, "Invalid debug duty value: %s", value);
            }
        } else {
            ESP_LOGW(TAG, "Debug duty value is null or empty");
        }
    }
    // Handle debug ADC value set
    else if (strcmp(command, "debug_set_adc") == 0) {
        if (value && strlen(value) > 0) {
            int adc_val = atoi(value);
            if (adc_val >= 0 && adc_val <= 4095) {
                extern volatile int adc_value_on_capture;
                extern volatile bool adc_manual_override;
                int old_adc = adc_value_on_capture;
                adc_value_on_capture = adc_val;
                adc_manual_override = true; // Prevent ADC task from overwriting
                ESP_LOGI(TAG, "DEBUG: ADC value manually set to %d (was %d), manual override enabled", adc_val, old_adc);
            } else {
                ESP_LOGW(TAG, "Invalid debug ADC value: %s (must be 0-4095)", value);
            }
        } else {
            ESP_LOGW(TAG, "Debug ADC value is null or empty");
        }
    }
    // Handle debug ADC manual override disable
    else if (strcmp(command, "debug_disable_adc_override") == 0) {
        extern volatile bool adc_manual_override;
        adc_manual_override = false; // Allow ADC task to resume normal operation
        ESP_LOGI(TAG, "DEBUG: ADC manual override disabled, returning to normal ADC operation");
    }
    else if (strcmp(command, "debug_set_adc") == 0) {
        extern volatile int adc_value_on_capture;
        extern volatile bool adc_manual_override;
        if (value) {
            int adc_val = atoi(value);
            adc_value_on_capture = adc_val;
            adc_manual_override = true; // Prevent ADC task from overwriting
            ESP_LOGI(TAG, "DEBUG: ADC value manually set to %d", adc_val);
        }
    }
    // Handle PID control enable/disable
    else if (strcmp(command, "enable_pid") == 0) {
        pid_control_enabled = true;
        ESP_LOGI(TAG, "PID control enabled");
    }
    else if (strcmp(command, "disable_pid") == 0) {
        pid_control_enabled = false;
        ESP_LOGI(TAG, "PID control disabled");
    }
    // Handle EDM cutting commands
    else if (strcmp(command, "start_cut") == 0) {
        ESP_LOGI(TAG, "EDM cutting started");
        // Send command to main task queue
        extern QueueHandle_t command_queue;
        if (command_queue) {
            web_command_t cmd;
            strncpy(cmd.command, "start_cut", sizeof(cmd.command) - 1);
            cmd.command[sizeof(cmd.command) - 1] = '\0';
            cmd.value = 0;
            if (xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send start_cut command to queue");
            }
        }
    }
    else if (strcmp(command, "stop_cut") == 0) {
        ESP_LOGI(TAG, "EDM cutting stopped");
        // Send command to main task queue
        extern QueueHandle_t command_queue;
        if (command_queue) {
            web_command_t cmd;
            strncpy(cmd.command, "stop_cut", sizeof(cmd.command) - 1);
            cmd.command[sizeof(cmd.command) - 1] = '\0';
            cmd.value = 0;
            if (xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send stop_cut command to queue");
            }
        }
    }
    // Handle jog commands
    else if (strcmp(command, "jog_up") == 0) {
        ESP_LOGI(TAG, "Jog up requested");
        // Send command to main task queue
        extern QueueHandle_t command_queue;
        if (command_queue) {
            web_command_t cmd;
            strncpy(cmd.command, "jog_up", sizeof(cmd.command) - 1);
            cmd.command[sizeof(cmd.command) - 1] = '\0';
            cmd.value = 0;
            if (xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send jog_up command to queue");
            }
        }
    }
    else if (strcmp(command, "jog_down") == 0) {
        ESP_LOGI(TAG, "Jog down requested");
        // Send command to main task queue
        extern QueueHandle_t command_queue;
        if (command_queue) {
            web_command_t cmd;
            strncpy(cmd.command, "jog_down", sizeof(cmd.command) - 1);
            cmd.command[sizeof(cmd.command) - 1] = '\0';
            cmd.value = 0;
            if (xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send jog_down command to queue");
            }
        }
    }
    else if (strcmp(command, "home_position") == 0) {
        ESP_LOGI(TAG, "Home position requested");
        // Send command to main task queue
        extern QueueHandle_t command_queue;
        if (command_queue) {
            web_command_t cmd;
            strncpy(cmd.command, "home_position", sizeof(cmd.command) - 1);
            cmd.command[sizeof(cmd.command) - 1] = '\0';
            cmd.value = 0;
            if (xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send home_position command to queue");
            }
        }
    }
    // Handle web control toggle
    else if (strcmp(command, "web_control") == 0) {
        int control_value = value ? atoi(value) : 0;
        ESP_LOGI(TAG, "Web control toggle requested: %s", control_value ? "enable" : "disable");
        // Send command to main task queue
        extern QueueHandle_t command_queue;
        if (command_queue) {
            web_command_t cmd;
            strncpy(cmd.command, "web_control", sizeof(cmd.command) - 1);
            cmd.command[sizeof(cmd.command) - 1] = '\0';
            cmd.value = control_value;
            if (xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send web_control command to queue");
            }
        }
    }
    // Catch unrecognized commands
    else {
        ESP_LOGW(TAG, "Unrecognized command: %s", command);
    }
    // Other commands will be handled by the main task through the queue system
}

// Command endpoint for receiving commands
static esp_err_t command_handler(httpd_req_t *req) {
    
    if (req->method == HTTP_POST) {
        char content[256];
        int recv_len = httpd_req_recv(req, content, sizeof(content) - 1);
        if (recv_len > 0) {
            content[recv_len] = '\0';
            
            // Parse JSON command
            cJSON *json = cJSON_Parse(content);
            if (json) {
                cJSON *command = cJSON_GetObjectItem(json, "command");
                cJSON *value = cJSON_GetObjectItem(json, "value");
                
                ESP_LOGI(TAG, "JSON parsed - command: %s, value type: %d", 
                         command ? command->valuestring : "null",
                         value ? value->type : -1);
                
                if (command && command->valuestring) {
                    const char *value_str = NULL;
                    if (value) {
                        if (value->type == cJSON_String) {
                            value_str = value->valuestring;
                        } else if (value->type == cJSON_Number) {
                            // Convert number to string
                            static char num_str[32];
                            if (value->valuedouble == (double)(int)value->valuedouble) {
                                snprintf(num_str, sizeof(num_str), "%d", (int)value->valuedouble);
                            } else {
                                snprintf(num_str, sizeof(num_str), "%.3f", value->valuedouble);
                            }
                            value_str = num_str;
                        }
                    }
                    process_command(command->valuestring, value_str);
                }
                
                cJSON_Delete(json);
            }
        }
    }
    
    // Send response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, "{\"status\":\"ok\"}", 15);
    
    return ESP_OK;
}

// WebSocket handler (simplified for now)
static esp_err_t websocket_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "WebSocket handler called - sending fallback response");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "WebSocket not implemented yet - use polling endpoints", 47);
    return ESP_OK;
}

static esp_err_t catch_all_handler(httpd_req_t *req) {
    // Log the request for debugging
    ESP_LOGW(TAG, "Unknown request: %s %s", 
             req->method == HTTP_GET ? "GET" : 
             req->method == HTTP_POST ? "POST" : 
             req->method == HTTP_OPTIONS ? "OPTIONS" : "UNKNOWN",
             req->uri);
    
    // Check if this is a WebSocket request that should be handled
    if (strcmp(req->uri, "/ws") == 0) {
        ESP_LOGW(TAG, "WebSocket request caught by catch-all handler - routing to WebSocket handler");
        // Route to WebSocket handler
        return websocket_handler(req);
    }
    
    // Return 404 for unknown requests
    httpd_resp_send_404(req);
    return ESP_OK;
}



// Send status to all WebSocket clients
esp_err_t websocket_send_status(edm_status_t *status) {
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "duty_percent", status->duty_percent);
    cJSON_AddNumberToObject(json, "adc_value", status->adc_value);
    cJSON_AddNumberToObject(json, "gap_voltage", status->gap_voltage);
    cJSON_AddBoolToObject(json, "is_cutting", status->is_cutting);
    cJSON_AddBoolToObject(json, "limit_switch_triggered", status->limit_switch_triggered);
    cJSON_AddNumberToObject(json, "jog_speed", status->jog_speed);
    cJSON_AddNumberToObject(json, "cut_speed", status->cut_speed);
    cJSON_AddNumberToObject(json, "step_position", status->step_position);
    cJSON_AddBoolToObject(json, "wifi_connected", status->wifi_connected);
    cJSON_AddNumberToObject(json, "uptime_seconds", status->uptime_seconds);
    cJSON_AddNumberToObject(json, "adc_blanking_delay", status->adc_blanking_delay);
    cJSON_AddNumberToObject(json, "pid_kp", status->pid_kp);
    cJSON_AddNumberToObject(json, "pid_ki", status->pid_ki);
    cJSON_AddNumberToObject(json, "pid_kd", status->pid_kd);
    cJSON_AddNumberToObject(json, "pid_setpoint", status->pid_setpoint);
    
    char *json_str = cJSON_Print(json);
    if (json_str) {
        char frame[WEBSOCKET_BUFFER_SIZE];
        size_t frame_len;
        
        if (create_websocket_frame(json_str, strlen(json_str), frame, &frame_len) == ESP_OK) {
            for (int i = 0; i < client_count; i++) {
                if (clients[i].is_websocket) {
                    send(clients[i].fd, frame, frame_len, 0);
                }
            }
        }
        
        free(json_str);
    }
    
    cJSON_Delete(json);
    return ESP_OK;
}

// Initialize web server
esp_err_t web_server_init(void) {
    system_start_time = esp_timer_get_time() / 1000000; // Convert to seconds
    status_mutex = xSemaphoreCreateMutex();
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEB_SERVER_PORT;
    config.max_uri_handlers = 16;
    config.max_resp_headers = 8;
    config.recv_wait_timeout = 30;
    config.send_wait_timeout = 30;
    
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Error starting server");
        return ESP_FAIL;
    }
    
    // Register URI handlers in order of specificity (most specific first)
    httpd_uri_t test_uri = {
        .uri = "/test",
        .method = HTTP_GET,
        .handler = test_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &test_uri);
    
    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &status_uri);
    
    httpd_uri_t command_uri = {
        .uri = "/command",
        .method = HTTP_POST,
        .handler = command_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &command_uri);
    
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &ws_uri);
    
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root_uri);
    
    // Register catch-all handler for unknown requests (least specific last)
    httpd_uri_t catch_all_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = catch_all_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &catch_all_uri);
    
    ESP_LOGI(TAG, "Web server started on port %d", WEB_SERVER_PORT);
    return ESP_OK;
}

// Update status from main application
void web_server_update_status(edm_status_t *status) {
    if (xSemaphoreTake(status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&current_status, status, sizeof(edm_status_t));
        xSemaphoreGive(status_mutex);
        
        // Send to all WebSocket clients
        websocket_send_status(status);
    }
}

// Web server task
void web_server_task(void *pvParameters) {
    ESP_LOGI(TAG, "Web server task started");
    
    while (1) {
        // Update status periodically
        edm_status_t status = {0};
        
        if (xSemaphoreTake(status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            status = current_status;
            xSemaphoreGive(status_mutex);
        }
        
        // Update real-time values
        status.duty_percent = duty_percent;
        status.adc_value = adc_value_on_capture;
        status.gap_voltage = (float)adc_value_on_capture * 3.3f / 4095.0f * 1000.0f; // Convert to mV
        status.wifi_connected = wifi_is_connected();
        status.uptime_seconds = (esp_timer_get_time() / 1000000) - system_start_time;
        status.adc_blanking_delay = adc_blanking_delay_ticks;
        status.pid_kp = pid_kp;
        status.pid_ki = pid_ki;
        status.pid_kd = pid_kd;
        status.pid_setpoint = pid_setpoint;
        
        web_server_update_status(&status);
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
    }
}

// Start web server
esp_err_t web_server_start(void) {
    return web_server_init();
}

// Stop web server
esp_err_t web_server_stop(void) {
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
    return ESP_OK;
}

// Deinitialize web server
void web_server_deinit(void) {
    web_server_stop();
    if (status_mutex) {
        vSemaphoreDelete(status_mutex);
        status_mutex = NULL;
    }
} 