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
        mController.update();
        can_update(&mController);
    }

    if (per_ms100_flag) {
    }

    if (per_sec_flag) {
        per_sec_flag = false;
        // if (mros_fail()) esp_restart();
        // char buffer[32];
        // sprintf(buffer, "%.2f", mController.cmd_vel.x);
        // mros_debug(buffer);
        // mros_publish_pc(&mController.powerc);

        Serial.printf("V: %02f\n", mController.powerc.solar.voltage);
        Serial.printf("I: %02f\n", mController.powerc.solar.current);
        Serial.printf("P: %02f\n", mController.powerc.solar.power);
        Serial.println("");
    }

    if (per_sec10_flag) {
        per_sec10_flag = false;
    }
}