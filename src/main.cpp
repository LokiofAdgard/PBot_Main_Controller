#include <Arduino.h>

#include "params.h"

MController mController;

void updateTask(void* pvParameters) {
    while (true) {
        mController.update();
        vTaskDelay(40);
    }
}

void setup() {
    neopixelWrite(LED_RGB, 0x00, 0x00, 0x00);
    mController.init();

    xTaskCreatePinnedToCore(
        updateTask,           // Task function
        "MControllerUpdate",  // Task name
        4096,                 // Stack size
        nullptr,              // Parameters
        1,                    // Priority
        nullptr,              // Task handle
        1                     // Core 1
    );
    neopixelWrite(LED_RGB, 0x00, 0x04, 0x00);
}

void loop() {
    mros_spin();

    if (per_ms10_flag) {
        per_ms10_flag = false;

        mController.mrosGetUpdate();
    }

    if (per_ms100_flag) {
        per_ms100_flag = false;

        can_tx_cmdvel(mController.cmd_vel);

        mros_publish_imu(&mController);
        mros_publish_tof(&mController);
    }

    if (per_sec_flag) {
        per_sec_flag = false;

        if (mros_fail()) esp_restart();

        mros_publish_pc(&mController.powerc);
        mros_publish_mc(&mController.motorc);

        can_req(PC_DATA_REQ, GET_ALL);
        can_req(MC_DATA_REQ, GET_ALL);
    }

    if (per_sec10_flag) {
        per_sec10_flag = false;
    }

    if (can_available) {
        can_available = false;
    }
}
