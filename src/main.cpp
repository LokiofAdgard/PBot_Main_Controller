#include <Arduino.h>

#include "params.h"

MController mController;

void setup() {
    // neopixelWrite(48, 0, 0, 0);
    mController.init();
}

void loop() {
    mros_spin();

    if (per_ms10_flag) {
        mController.update();
    }

    if (per_ms100_flag) {
    }

    if (per_sec_flag) {
        per_sec_flag = false;

        mController.powerc.mppt.voltage++;

        char buffer[32];
        sprintf(buffer, "%.2f", mController.cmd_vel.x);
        mros_debug(buffer);
        mros_publish_pc(&mController.powerc);
    }

    if (per_sec10_flag) {
    }
}