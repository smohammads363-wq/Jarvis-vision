/* Jarvis Vision - Edge Impulse Inferencing with SH1106 OLED */

#include <MdSubhan-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// OLED aur I2C ke liye libraries
#include <Wire.h>
#include <U8x8lib.h>

// ESP32 ko hang/crash hone se bachane ke liye (Brownout Detector Disable)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Camera Model - AI THINKER select kiya hai
#define CAMERA_MODEL_AI_THINKER 

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

// OLED Display Setup (SCL = 14, SDA = 15)
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE, /* clock=*/ 14, /* data=*/ 15);

/* Constant defines */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables */
static bool debug_nn = false; 
static bool is_initialised = false;
uint8_t *snapshot_buf; 

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG, 
    .frame_size = FRAMESIZE_QVGA,   
    .jpeg_quality = 12, 
    .fb_count = 1,       
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);

void setup() {
    // 1. ESP32 ko crash hone se bachane ka hack (Brownout disable)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    
    // 2. OLED Display chalu karna
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.drawString(0, 0, "Jarvis Vision");
    u8x8.drawString(0, 2, "Booting Camera..");

    if (ei_camera_init() == false) {
        Serial.println("Failed to initialize Camera!");
        u8x8.drawString(0, 4, "Camera Error!");
        return;
    }

    u8x8.clearDisplay();
    u8x8.drawString(0, 0, "Jarvis Vision");
    u8x8.drawString(0, 2, "System Ready!");
    ei_sleep(2000);
}

void loop() {
    if (ei_sleep(5) != EI_IMPULSE_OK) return;

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    if(snapshot_buf == nullptr) {
        Serial.println("ERR: Failed to allocate memory!");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        Serial.println("Failed to capture image");
        free(snapshot_buf);
        return;
    }

    // AI Dimaag kaam kar raha hai (Inference)
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        Serial.printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        return;
    }

    // OLED par Percentages dikhane ka logic
    u8x8.clearDisplay();
    u8x8.drawString(0, 0, "Vision Scan:");

    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        // Decimal value (jaise 0.95) ko 100 se multiply karke percentage banaya
        int percentage = (int)(result.classification[i].value * 100);
        
        // Alag-alag line par print karna (Line 2 aur Line 4)
        u8x8.setCursor(0, 2 + (i * 2)); 
        u8x8.print(ei_classifier_inferencing_categories[i]);
        u8x8.print(": ");
        u8x8.print(percentage);
        u8x8.print("%");
        
        // Serial monitor par bhi print karna
        Serial.printf("%s: %d%%\r\n", ei_classifier_inferencing_categories[i], percentage);
    }

    free(snapshot_buf);
}

// ---- Niche ki system files waisi hi rahengi ----

bool ei_camera_init(void) {
    if (is_initialised) return true;
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) return false;
    
    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); 
      s->set_brightness(s, 1); 
      s->set_saturation(s, 0); 
    }
    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();
    if (err == ESP_OK) is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;
    if (!is_initialised) return false;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if(!converted) return false;

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    return 0;
}
