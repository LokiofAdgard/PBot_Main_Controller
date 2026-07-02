#ifndef MROS_H
#define MROS_H

#include <Arduino.h>

void mros_init(HardwareSerial& serial);
void mros_spin();

void mros_publish_string(const char* text);

float mros_get_cmd_vel_lin_x();
float mros_get_cmd_vel_lin_y();
float mros_get_cmd_vel_ang_z();

#endif
