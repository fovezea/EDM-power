/*
 * Web Server with WebSocket for EDM Power Project
 * ESP-IDF v5.4.1 compatible
 */

#include "web_server.h"
#include "wifi_connect.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include <string.h>
#include "lwip/sockets.h"

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
"                <div class=\"value-display\" id=\"blanking-delay\">1 ticks</div>\n"
"                <input type=\"range\" min=\"1\" max=\"200\" value=\"1\" class=\"slider\" id=\"blanking-delay-slider\">\n"
"                <button class=\"button\" onclick=\"setBlankingDelay()\">Set Delay</button>\n"
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
"            <button class=\"button success\" onclick=\"startCut()\">Start EDM Cut</button>\n"
"            <button class=\"button danger\" onclick=\"stopCut()\">Stop EDM Cut</button>\n"
"            <button class=\"button\" onclick=\"jogUp()\">Jog Up</button>\n"
"            <button class=\"button\" onclick=\"jogDown()\">Jog Down</button>\n"
"            <button class=\"button\" onclick=\"homePosition()\">Home Position</button>\n"
"        </div>\n"
"    </div>\n"
"\n"
"    <script>\n"
"        var ws = null;\n"
"        var reconnectTimer = null;\n"
"\n"
"        function connectWebSocket() {\n"
"            var protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';\n"
"            var wsUrl = protocol + '//' + window.location.host + '/ws';\n"
"            \n"
"            ws = new WebSocket(wsUrl);\n"
"            \n"
"            ws.onopen = function() {\n"
"                document.getElementById('status-text').textContent = 'Connected';\n"
"                document.getElementById('connection-status').className = 'connection-status connected';\n"
"            };\n"
"            \n"
"            ws.onmessage = function(event) {\n"
"                var data = JSON.parse(event.data);\n"
"                updateDisplay(data);\n"
"            };\n"
"            \n"
"            ws.onclose = function() {\n"
"                document.getElementById('status-text').textContent = 'Disconnected';\n"
"                document.getElementById('connection-status').className = 'connection-status disconnected';\n"
"                if (reconnectTimer) clearTimeout(reconnectTimer);\n"
"                reconnectTimer = setTimeout(connectWebSocket, 2000);\n"
"            };\n"
"        }\n"
"\n"
"        function updateDisplay(data) {\n"
"            document.getElementById('duty-percent').textContent = data.duty_percent.toFixed(1) + '%';\n"
"            document.getElementById('gap-voltage').textContent = data.gap_voltage.toFixed(1) + ' mV';\n"
"            document.getElementById('adc-value').textContent = data.adc_value;\n"
"            document.getElementById('cutting-status').textContent = data.is_cutting ? 'Running' : 'Stopped';\n"
"            document.getElementById('limit-status').textContent = data.limit_switch_triggered ? 'TRIGGERED' : 'OK';\n"
"            document.getElementById('wifi-status').textContent = data.wifi_connected ? 'Connected' : 'Disconnected';\n"
"            document.getElementById('jog-speed').textContent = data.jog_speed.toFixed(1) + ' mm/s';\n"
"            document.getElementById('cut-speed').textContent = data.cut_speed.toFixed(2) + ' mm/s';\n"
"            document.getElementById('step-position').textContent = data.step_position + ' steps';\n"
"            document.getElementById('blanking-delay').textContent = data.adc_blanking_delay + ' ticks';\n"
"        }\n"
"\n"
"        function sendCommand(command, value) {\n"
"            if (ws && ws.readyState === WebSocket.OPEN) {\n"
"                var cmd = { command: command };\n"
"                if (value !== null) cmd.value = value;\n"
"                ws.send(JSON.stringify(cmd));\n"
"            }\n"
"        }\n"
"\n"
"        function setDuty() {\n"
"            var duty = document.getElementById('duty-slider').value;\n"
"            sendCommand('set_duty', parseInt(duty));\n"
"        }\n"
"\n"
"        function setBlankingDelay() {\n"
"            var delay = document.getElementById('blanking-delay-slider').value;\n"
"            sendCommand('set_blanking_delay', parseInt(delay));\n"
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
"        connectWebSocket();\n"
"        \n"
"        document.getElementById('duty-slider').addEventListener('input', function() {\n"
"            document.getElementById('duty-percent').textContent = this.value + '%';\n"
"        });\n"
"        \n"
"        document.getElementById('blanking-delay-slider').addEventListener('input', function() {\n"
"            document.getElementById('blanking-delay').textContent = this.value + ' ticks';\n"
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
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

// WebSocket upgrade handler
static esp_err_t websocket_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        // Handle WebSocket upgrade
        char upgrade_header[64];
        char connection_header[64];
        char key_header[64];
        
        esp_err_t ret1 = httpd_req_get_hdr_value_str(req, "Upgrade", upgrade_header, sizeof(upgrade_header));
        esp_err_t ret2 = httpd_req_get_hdr_value_str(req, "Connection", connection_header, sizeof(connection_header));
        esp_err_t ret3 = httpd_req_get_hdr_value_str(req, "Sec-WebSocket-Key", key_header, sizeof(key_header));
        
        if (ret1 != ESP_OK || ret2 != ESP_OK || ret3 != ESP_OK ||
            strcmp(upgrade_header, "websocket") != 0 ||
            strstr(connection_header, "Upgrade") == NULL) {
            return ESP_FAIL;
        }
        
        // Send WebSocket upgrade response
        char response[256];
        snprintf(response, sizeof(response),
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                "Sec-WebSocket-Accept: %s\r\n\r\n",
                key_header); // In production, you should properly hash the key
        
        httpd_resp_send(req, response, strlen(response));
        
        // Add client to list
        if (client_count < MAX_CLIENTS) {
            clients[client_count].fd = httpd_req_to_sockfd(req);
            clients[client_count].is_websocket = true;
            clients[client_count].buffer_len = 0;
            client_count++;
            ESP_LOGI(TAG, "WebSocket client connected, total: %d", client_count);
        }
        
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

// Command processing - now sends commands to main task via queue
static void process_command(const char *command, const char *value) {
    ESP_LOGI(TAG, "Received command: %s, value: %s", command, value ? value : "null");
    
    // For now, we'll handle duty cycle directly here since it's a simple variable
    if (strcmp(command, "set_duty") == 0 && value) {
        int duty = atoi(value);
        if (duty >= 1 && duty <= 99) {
            duty_percent = duty;
            ESP_LOGI(TAG, "Duty cycle set to %d%%", duty);
        }
    }
    // Handle ADC blanking delay setting
    else if (strcmp(command, "set_blanking_delay") == 0 && value) {
        int delay = atoi(value);
        if (delay >= 1 && delay <= 200) { // Limit to reasonable range
            adc_blanking_delay_ticks = delay;
            ESP_LOGI(TAG, "ADC blanking delay set to %d ticks", delay);
        }
    }
    // Other commands will be handled by the main task through the queue system
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
    
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Error starting server");
        return ESP_FAIL;
    }
    
    // Register URI handlers
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root_uri);
    
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &ws_uri);
    
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