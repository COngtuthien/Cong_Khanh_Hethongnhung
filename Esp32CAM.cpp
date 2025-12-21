#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BallBoxBC_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include "soc/rtc_cntl_reg.h"  // Disable brownout

// WiFi credentials
const char* ssid = "Huster 37";
const char* password = "khongnhomatkhau";

// Camera pins
#define CAMERA_MODEL_AI_THINKER
#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#else
#error "Camera model not selected"
#endif

// EI constants
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

// Pins
#define SERVO_PIN 12
#define SDA_PIN   14
#define SCL_PIN   15

// Global camera config (fields in correct order for ESP32 core 3.x)
static camera_config_t camera_config = {
    .pin_pwdn       = PWDN_GPIO_NUM,
    .pin_reset      = RESET_GPIO_NUM,
    .pin_xclk       = XCLK_GPIO_NUM,
    .pin_sccb_sda   = SIOD_GPIO_NUM,
    .pin_sccb_scl   = SIOC_GPIO_NUM,

    .pin_d7         = Y9_GPIO_NUM,
    .pin_d6         = Y8_GPIO_NUM,
    .pin_d5         = Y7_GPIO_NUM,
    .pin_d4         = Y6_GPIO_NUM,
    .pin_d3         = Y5_GPIO_NUM,
    .pin_d2         = Y4_GPIO_NUM,
    .pin_d1         = Y3_GPIO_NUM,
    .pin_d0         = Y2_GPIO_NUM,
    .pin_vsync      = VSYNC_GPIO_NUM,
    .pin_href       = HREF_GPIO_NUM,
    .pin_pclk       = PCLK_GPIO_NUM,

    .xclk_freq_hz   = 10000000,
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,

    .pixel_format   = PIXFORMAT_JPEG,
    .frame_size     = FRAMESIZE_QVGA,
    .jpeg_quality   = 12,
    .fb_count       = 2,
    .fb_location    = CAMERA_FB_IN_PSRAM,
    .grab_mode      = CAMERA_GRAB_LATEST,
};

// Globals
AsyncWebServer server(80);
Servo myservo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf = nullptr;

bool detection_running = true;
String current_detection = "No Object";
float current_confidence = 0.0f;
int current_servo_angle = 90;

// Customizable settings
int servo_ball_angle = 45;
int servo_box_angle = 0;
float confidence_threshold = 0.5f;
int detection_delay = 1000;

// HTML GUI - Đẹp hơn, đóng khung, chữ tiếng Việt rõ ràng, nút servo cập nhật theo setting, form input cập nhật giá trị hiện tại
// ... (toàn bộ phần trước giống code cũ)

// Chỉ thay phần index_html này (phần JS đã sửa)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html lang="vi">
<head>
  <title>ESP32-CAM Phân Loại Ball/Box</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    /* Giữ nguyên style đẹp như cũ */
    body { font-family: 'Segoe UI', Arial, sans-serif; background: linear-gradient(to bottom, #e0f7fa, #ffffff); margin: 0; padding: 20px; color: #333; }
    .container { max-width: 600px; margin: 0 auto; background: white; border-radius: 15px; box-shadow: 0 10px 30px rgba(0,0,0,0.2); overflow: hidden; }
    header { background: #00796b; color: white; padding: 20px; text-align: center; }
    h1 { margin: 0; font-size: 1.8em; }
    .status { padding: 20px; background: #f1f8e9; border-bottom: 1px solid #ccc; text-align: center; font-size: 1.3em; }
    .controls { padding: 20px; }
    .btn-group { display: flex; justify-content: center; gap: 15px; margin: 20px 0; flex-wrap: wrap; }
    button { padding: 12px 25px; font-size: 1.1em; border: none; border-radius: 8px; cursor: pointer; transition: 0.3s; flex: 1 1 150px; max-width: 200px; }
    .btn-servo { background: #4caf50; color: white; }
    .btn-servo:hover { background: #388e3c; }
    .btn-toggle { background: #ff9800; color: white; }
    .btn-toggle:hover { background: #f57c00; }
    .settings { padding: 20px; background: #e8f5e9; }
    .settings h2 { text-align: center; color: #00796b; }
    form { display: grid; gap: 15px; }
    label { font-weight: bold; }
    input[type=number] { width: 100%; padding: 10px; border: 1px solid #ccc; border-radius: 5px; font-size: 1em; }
    .btn-submit { background: #00796b; color: white; padding: 15px; font-size: 1.2em; border-radius: 8px; }
    .btn-submit:hover { background: #004d40; }
    footer { text-align: center; padding: 15px; background: #00796b; color: white; font-size: 0.9em; }
  </style>
  <script>
    function updateStatus() {
      fetch('/data').then(r => r.json()).then(data => {
        document.getElementById('detected').innerText = data.detection;
        document.getElementById('confidence').innerText = (data.confidence).toFixed(2) + '%';
        document.getElementById('servo').innerText = data.servo + '°';
        document.getElementById('status').innerText = data.running ? 'Đang chạy' : 'Dừng';
        
        // Chỉ cập nhật nút servo (không đụng đến input)
        document.getElementById('btn-box').innerText = 'Box → ' + data.box_angle + '°';
        document.getElementById('btn-ball').innerText = 'Ball → ' + data.ball_angle + '°';
        document.getElementById('btn-box').setAttribute('onclick', 'setServo(' + data.box_angle + ')');
        document.getElementById('btn-ball').setAttribute('onclick', 'setServo(' + data.ball_angle + ')');
      });
    }
    
    // Chỉ cập nhật status mỗi giây, KHÔNG cập nhật input
    setInterval(updateStatus, 1000);
    
    // Khi trang load lần đầu, lấy giá trị hiện tại để điền vào form (chỉ chạy 1 lần)
    window.onload = function() {
      fetch('/data').then(r => r.json()).then(data => {
        document.getElementById('input-ball').value = data.ball_angle;
        document.getElementById('input-box').value = data.box_angle;
        document.getElementById('input-threshold').value = data.threshold;
        document.getElementById('input-delay').value = data.delay;
        updateStatus(); // Cập nhật status và nút lần đầu
      });
    };
    
    function setServo(angle) { fetch('/servo?angle=' + angle); }
    function toggleDetect() { fetch('/toggle'); }
  </script>
</head>
<body>
  <div class="container">
    <header>
      <h1>ESP32-CAM Phân Loại Ball / Box</h1>
    </header>
    
    <div class="status">
      <strong>Phát hiện:</strong> <span id="detected">No Object</span><br>
      <strong>Độ tin cậy:</strong> <span id="confidence">0.00%</span><br>
      <strong>Góc Servo:</strong> <span id="servo">90°</span><br>
      <strong>Trạng thái:</strong> <span id="status">Đang chạy</span>
    </div>
    
    <div class="controls">
      <div class="btn-group">
        <button class="btn-servo" id="btn-box">Box → 0°</button>
        <button class="btn-servo" id="btn-ball">Ball → 45°</button>
        <button class="btn-servo" onclick="setServo(90)">Trung lập → 90°</button>
      </div>
      <div style="text-align:center;">
        <button class="btn-toggle" onclick="toggleDetect()">Bật / Tắt Phát Hiện</button>
      </div>
    </div>
    
    <div class="settings">
      <h2>Cài Đặt Hệ Thống</h2>
      <form action="/setup" method="post">
        <label>Góc Servo khi phát hiện Ball (0-180°):</label>
        <input type="number" id="input-ball" name="ball_angle" min="0" max="180">
        
        <label>Góc Servo khi phát hiện Box (0-180°):</label>
        <input type="number" id="input-box" name="box_angle" min="0" max="180">
        
        <label>Ngưỡng độ tin cậy (0.0 - 1.0):</label>
        <input type="number" id="input-threshold" name="threshold" step="0.01" min="0" max="1">
        
        <label>Thời gian giữa các lần phát hiện (ms):</label>
        <input type="number" id="input-delay" name="delay" min="500" max="5000">
        
        <button type="submit" class="btn-submit">Áp Dụng Cài Đặt</button>
      </form>
    </div>
    
    <footer>
      Dự án ESP32-CAM + Edge Impulse | Điều khiển từ xa qua WiFi
    </footer>
  </div>
</body>
</html>
)rawliteral";

// Camera functions
bool ei_camera_init(void) {
    if (is_initialised) return true;
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, -1);
    }
    s->set_framesize(s, FRAMESIZE_QVGA);
    is_initialised = true;
    return true;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) return false;
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
    if (!converted) return false;

    if (img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS || img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {
        ei::image::processing::crop_and_interpolate_rgb888(
            snapshot_buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf, img_width, img_height);
    } else {
        memcpy(out_buf, snapshot_buf, img_width * img_height * 3);
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t out_ptr_ix = 0;
    while (length--) {
        out_ptr[out_ptr_ix++] = (snapshot_buf[pixel_ix + 2] << 16) | (snapshot_buf[pixel_ix + 1] << 8) | snapshot_buf[pixel_ix];
        pixel_ix += 3;
    }
    return 0;
}

void setup() {
    Serial.begin(115200);
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Wire.begin(SDA_PIN, SCL_PIN);
    lcd.init(); lcd.backlight(); lcd.clear();
    lcd.print("Khoi dong...");

    myservo.attach(SERVO_PIN, 500, 2400);
    myservo.write(90);

    if (!ei_camera_init()) {
        lcd.clear(); lcd.print("Camera loi!");
        while(1) delay(1000);
    }

    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi ket noi: " + WiFi.localIP().toString());

    // Web routes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){ req->send_P(200, "text/html", index_html); });

    // Trả về data JSON (đã fix lỗi +)
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *req){
        String json = "{";
        json += "\"detection\":\"" + current_detection + "\",";
        json += "\"confidence\":" + String(current_confidence * 100, 2) + ",";
        json += "\"servo\":" + String(current_servo_angle) + ",";
        json += "\"running\":" + String(detection_running ? "true" : "false") + ",";
        json += "\"ball_angle\":" + String(servo_ball_angle) + ",";
        json += "\"box_angle\":" + String(servo_box_angle) + ",";
        json += "\"threshold\":" + String(confidence_threshold, 2) + ",";
        json += "\"delay\":" + String(detection_delay);
        json += "}";
        req->send(200, "application/json", json);
    });

    server.on("/servo", HTTP_GET, [](AsyncWebServerRequest *req){
        if (req->hasParam("angle")) {
            int angle = req->getParam("angle")->value().toInt();
            if (angle >= 0 && angle <= 180) {
                myservo.write(angle);
                current_servo_angle = angle;
            }
        }
        req->send(200, "text/plain", "OK");
    });

    server.on("/toggle", HTTP_GET, [](AsyncWebServerRequest *req){
        detection_running = !detection_running;
        if (!detection_running) {
            current_detection = "Stopped";
            current_confidence = 0.0f;
            myservo.write(90);
            current_servo_angle = 90;
        }
        req->send(200, "text/plain", "OK");
    });

    server.on("/setup", HTTP_POST, [](AsyncWebServerRequest *req){
        if (req->hasParam("ball_angle", true)) servo_ball_angle = req->getParam("ball_angle", true)->value().toInt();
        if (req->hasParam("box_angle", true)) servo_box_angle = req->getParam("box_angle", true)->value().toInt();
        if (req->hasParam("threshold", true)) confidence_threshold = req->getParam("threshold", true)->value().toFloat();
        if (req->hasParam("delay", true)) detection_delay = req->getParam("delay", true)->value().toInt();
        req->redirect("/");
    });

    server.begin();
    lcd.clear(); lcd.print("IP:"); lcd.setCursor(0,1); lcd.print(WiFi.localIP());
}

void loop() {
    if (!detection_running) { delay(1000); return; }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if (!snapshot_buf) return;

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
        free(snapshot_buf); return;
    }

    ei_impulse_result_t result = {0};
    if (run_classifier(&signal, &result, debug_nn) != EI_IMPULSE_OK) {
        free(snapshot_buf); return;
    }

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    float max_value = 0.0f;
    bool detected = false;
    ei_impulse_result_bounding_box_t best_bb;

    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        auto bb = result.bounding_boxes[i];
        if (bb.value > max_value && bb.value > confidence_threshold && (strcmp(bb.label, "ball")==0 || strcmp(bb.label, "box")==0)) {
            max_value = bb.value;
            best_bb = bb;
            detected = true;
        }
    }

    lcd.clear(); lcd.setCursor(0,0);
    if (detected) {
        current_detection = best_bb.label;
        current_confidence = max_value;
        if (strcmp(best_bb.label, "ball") == 0) {
            myservo.write(servo_ball_angle);
            current_servo_angle = servo_ball_angle;
            lcd.print("Detected: Ball");
        } else {
            myservo.write(servo_box_angle);
            current_servo_angle = servo_box_angle;
            lcd.print("Detected: Box");
        }
        lcd.setCursor(0,1); lcd.print("Conf: "); lcd.print(max_value*100, 1); lcd.print("%");
    } else {
        current_detection = "No Object";
        current_confidence = 0.0f;
        myservo.write(90);
        current_servo_angle = 90;
        lcd.print("No Object");
    }
#endif

    free(snapshot_buf);
    delay(detection_delay);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif