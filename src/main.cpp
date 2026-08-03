#include <Arduino.h>

#include "params.h"

MController mController;

void setup() {
    neopixelWrite(LED_RGB, 0x00, 0x00, 0x00);
    // Serial.begin(115200);
    mController.init();
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
        mController.update();
        mros_publish_imu(&mController);
        mros_publish_tof(&mController);
    }

    if (per_sec_flag) {
        per_sec_flag = false;

        if (mros_fail()) esp_restart();
        char buffer[32];
        sprintf(buffer, "%.2f", mController.cmd_vel.x);
        mros_debug(buffer);
        mros_publish_pc(&mController.powerc);
        mros_publish_mc(&mController.motorc);

        // can_req(PC_DATA_REQ, GET_STAT);
        can_req(PC_DATA_REQ, GET_ALL);
        can_req(MC_DATA_REQ, GET_ALL);
    }

    if (per_sec10_flag) {
        per_sec10_flag = false;
    }

    if (can_available) {
        can_available = false;
        // can_update(&mController);

        // Serial.printf("V: %02f\n", (float)(mController.powerc.solar.voltage * 1.25e-3f));
        // Serial.printf("I: %02f\n", (float)(mController.powerc.solar.current * 0.4f));
        // Serial.printf("P: %02f\n", (float)(mController.powerc.solar.power * 0.0025f * 0.4f));
        // Serial.printf("T: %02f\n", (float)(mController.powerc.temp * 0.0625f));
        // Serial.println("");
    }
}