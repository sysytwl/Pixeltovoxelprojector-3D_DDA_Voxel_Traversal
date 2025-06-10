#include <Arduino.h>
#include "RayCasting.h"
#include "img_process.h"

RayCasting rm;
ImageProcessor Imgp;

void setup(){
    // Check if PSRAM is available
    if (psramFound()) {
        printf("PSRAM found. Size: %u bytes\n", ESP.getPsramSize());
    } else {
        printf("PSRAM not found.\n");
        while(1);
    }

    int timecost = micros();
    rm.init(800,600,60);
    timecost = micros() - timecost;
    printf("time cost for init cam ray lut: %d\n", timecost);



    uint8_t* fake_img = (uint8_t*)ps_malloc(1280 * 720);
    if (!fake_img) {
        printf("Failed to allocate fake image in PSRAM.\n");
        while(1);
    }
    // Fill image with dummy data
    for (int i = 0; i < 1280 * 720; ++i) {
        fake_img[i] = i % 256;
    }
    Imgp.init(800,600);

    int img_timecost = micros();
    Imgp.YUV422ToGray(fake_img);
    img_timecost = micros() - img_timecost;
    printf("time cost for gray trans: %d\n", img_timecost);

    img_timecost = micros();
    Imgp.GrayToBinary(fake_img);
    img_timecost = micros() - img_timecost;
    printf("time cost for binary: %d\n", img_timecost);

    img_timecost = micros();
    Imgp.dissolve(fake_img);
    img_timecost = micros() - img_timecost;
    printf("time cost for dissolve: %d\n", img_timecost);

    img_timecost = micros();
    Imgp.edge_dection(fake_img);
    img_timecost = micros() - img_timecost;
    printf("time cost for edge dect: %d\n", img_timecost);

    printf("PSRAM used: %u bytes, free: %u bytes\n", ESP.getPsramSize() - ESP.getFreePsram(), ESP.getFreePsram());
    printf("RAM used: %u bytes, free: %u bytes\n", ESP.getHeapSize() - ESP.getFreeHeap(), ESP.getFreeHeap());

    //rm.RayMatch();
};

void loop(){

};