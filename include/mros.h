#ifndef MROS_H
#define MROS_H

#include <Arduino.h>

#include "esp_system.h"
#include "structs.h"

void mros_init(HardwareSerial& serial);
void mros_spin();

void mros_debug(const char* text);
void mros_publish_pc(const PC_t* pc);
void mros_publish_mc(const MC_t* mc);
void mros_publish_imu(const MController* mc);

int16_t mros_get_motor_l();
int16_t mros_get_motor_r();

bool mros_fail();

#endif
