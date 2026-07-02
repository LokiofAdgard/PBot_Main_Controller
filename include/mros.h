#ifndef MROS_H
#define MROS_H

#include <Arduino.h>

#include "esp_system.h"
#include "structs.h"

void mros_init(HardwareSerial& serial);
void mros_spin();

void mros_debug(const char* text);
void mros_publish_pc(const PC_t* pc);

float mros_get_cmd_vel_lin_x();
float mros_get_cmd_vel_lin_y();
float mros_get_cmd_vel_ang_z();

bool mros_fail();

#endif
