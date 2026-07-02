#include <Arduino.h>

#include "params.h"

MController mController;

void setup() {
    // neopixelWrite(48, 0, 0, 0);
    mros_init(Serial);
}

void loop() {
    static uint32_t last = 0;

    mros_spin();
    mController.update();

    if (millis() - last >= 2000) {
        last = millis();

        mController.powerc.mppt.voltage++;

        char buffer[32];
        sprintf(buffer, "%.2f", mController.cmd_vel.x);
        mros_publish_string(buffer);
        mros_publish_pc(&mController.powerc);
    }
}