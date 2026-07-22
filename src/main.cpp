#include <Arduino.h>

#include "params.h"

MController mController;

void setup() {
    neopixelWrite(48, 0x00, 0x04, 0x00);
    init_can();
    Serial.begin(115200);
    mController.init();
}

void loop() {
    // mros_spin();

    if (per_ms10_flag) {
    }

    if (per_ms100_flag) {
    }
    
    if (per_sec_flag) {
        per_sec_flag = false;
        mController.update();
        // if (mros_fail()) esp_restart();
        // char buffer[32];
        // sprintf(buffer, "%.2f", mController.cmd_vel.x);
        // mros_debug(buffer);
        // mros_publish_pc(&mController.powerc);

        can_req(PC_DATA_REQ, GET_ALL);
    }

    if (per_sec10_flag) {
        per_sec10_flag = false;
    }

    if (can_available) {
        can_available = false;
        can_update(&mController);

        Serial.printf("V: %02f\n", (float)(mController.powerc.solar.voltage * 1.25e-3f));
        Serial.printf("I: %02f\n", (float)(mController.powerc.solar.current * 0.4f));
        Serial.printf("P: %02f\n", (float)(mController.powerc.solar.power * 0.0025f * 0.4f));
        Serial.printf("T: %02f\n", (float)(mController.powerc.temp * 0.0625f));
        Serial.println("");
    }
}