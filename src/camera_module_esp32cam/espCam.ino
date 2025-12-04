#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ===============================================================
// ===                 NETWORK CONFIGURATION                   ===
// ===============================================================

// Your WiFi credentials (already filled in)
const char* ssid = "AndroidAP";
const char* password = "oqro54177";

//
// =================================================================
// ===   ACTION REQUIRED: CHANGE THIS IP ADDRESS!                ===
// =================================================================
//
// Replace with the IP address of your LAPTOP
// after you connect it to the "AndroidAP" hotspot.
// Get this using the 'ipconfig' command.
//
const char* nodeRedServer = "http://192.168.226.2:1880/upload/image";
//
// =================================================================
//


// ===============================================================
// ===         CAMERA MODEL: AI-THINKER - DO NOT CHANGE        ===
// ===============================================================
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
// ===============================================================


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector

  Serial.begin(115200);
  Serial.println("\n--- ESP-CAM to Node-RED Sender ---");

  // Camera configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  config.frame_size = FRAMESIZE_XGA; // 1024x768 resolution
  config.jpeg_quality = 12; // Lower number is higher quality
  config.fb_count = 1;

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("FATAL: Camera init failed with error 0x%x", err);
    return;
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP-CAM IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  Serial.println("-------------------------");
  Serial.println("Attempting to take and send picture...");
  
  camera_fb_t * fb = NULL;
  
  // Take a picture
  fb = esp_camera_fb_get();  
  
  if (!fb) {
    Serial.println("ERROR: Camera capture failed.");
    delay(2000); // Wait before retrying
    return;
  }

  Serial.printf("Picture taken! Size: %zu bytes\n", fb->len);

  // Send the picture to Node-RED
  HTTPClient http;
  http.begin(nodeRedServer);
  http.addHeader("Content-Type", "image/jpeg"); 

  int httpResponseCode = http.POST(fb->buf, fb->len);

  if (httpResponseCode > 0) {
    Serial.printf("SUCCESS! HTTP Response code: %d\n", httpResponseCode);
    //String response = http.getString();
    //Serial.println(response);
  } else {
    Serial.printf("ERROR on sending POST: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  http.end();
  
  // Return the frame buffer to be reused
  esp_camera_fb_return(fb);

  Serial.println("Waiting 10 seconds before next picture...");
  delay(10000); 
}