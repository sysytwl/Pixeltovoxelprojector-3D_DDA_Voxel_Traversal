#include <Arduino.h>
#include "RayCasting.h"
#include "img_process.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// AI Thinker pin definition
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

#define WIFI_SSID "test"
#define WIFI_PASS "12345678"
#define UDP_PORT 12345

WiFiUDP udp;
IPAddress remoteIp(192,168,137,1); // <-- Set this to your PC's IP on the WiFi network
RayCasting rm;
ImageProcessor Imgp;
uint8_t* edge_img_out;
uint32_t packed_size;
uint16_t result_w, result_h;
int img_timecost;
void setup(){
    // Connect to WiFi as STA
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    printf("WiFi connected, IP address: %s\n", WiFi.localIP().toString().c_str());

    camera_config_t config;{
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
        config.pixel_format = PIXFORMAT_GRAYSCALE;
        config.frame_size = FRAMESIZE_HD;
        //config.jpeg_quality = 12;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_LATEST;
    }
    if (esp_camera_init(&config) != ESP_OK) {
        printf("Camera init failed\n");
        while(1);
    }

    Imgp.init(1280, 720, 3); // specify block_width
    // Prepare output buffer for packed edge image
    result_w = Imgp.getResultWidth();
    result_h = Imgp.getResultHeight();
    packed_size = Imgp.getResultSize();
    edge_img_out = (uint8_t*)ps_malloc(packed_size);
    if (!edge_img_out) {
        printf("Failed to allocate edge_img_out\n");
        return;
    }
    //rm.init(img_width, img_heigh,60);
    // Imgp.dissolve(fake_img);
    //printf("PSRAM used: %u bytes, free: %u bytes\n", ESP.getPsramSize() - ESP.getFreePsram(), ESP.getFreePsram());
    //printf("RAM used: %u bytes, free: %u bytes\n", ESP.getHeapSize() - ESP.getFreeHeap(), ESP.getFreeHeap());
    //rm.RayMatch();
};



void loop(){
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        printf("Camera capture failed\n");
        while(1);
    }
    uint8_t* fake_img = fb->buf;

    img_timecost = micros();
    Imgp.GrayToBinary(fake_img,127);
    Imgp.edge_dection(fake_img, edge_img_out);
    img_timecost = micros() - img_timecost;
    printf("time cost for img process: %d\n", img_timecost);

    //--- Pack binary image (1 bit per pixel) into bytes ---
    size_t packed_size_to_send = (packed_size + 7) / 8;
    uint8_t* packed_img = (uint8_t*)ps_malloc(packed_size_to_send);
    if (packed_img) {
        memset(packed_img, 0, packed_size_to_send);
        for (size_t i = 0; i < packed_size; ++i) {
            if (edge_img_out[i]) {
                packed_img[i / 8] |= (1 << (7 - (i % 8)));
            }
        }

        // Send header
        char header[64];
        int header_len = snprintf(header, sizeof(header), "START OF IMG, %d, %d\n", result_w, result_h);
        udp.beginPacket("255.255.255.255", UDP_PORT); // broadcast
        udp.write((uint8_t*)header, header_len);
        udp.endPacket();

        // Send packed image in chunks
        const size_t CHUNK_SIZE = 1400; // UDP safe size
        for (size_t sent = 0; sent < packed_size_to_send; sent += CHUNK_SIZE) {
            size_t to_send = (packed_size_to_send - sent > CHUNK_SIZE) ? CHUNK_SIZE : (packed_size_to_send - sent);
            udp.beginPacket("255.255.255.255", UDP_PORT);
            udp.write(packed_img + sent, to_send);
            udp.endPacket();
            delay(2); // small delay to avoid packet loss
        }

        // Send footer
        const char* footer = "END OF IMG\n";
        udp.beginPacket("255.255.255.255", UDP_PORT);
        udp.write((const uint8_t*)footer, strlen(footer));
        udp.endPacket();
    }
    free(packed_img);

    esp_camera_fb_return(fb);
}