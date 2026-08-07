#ifndef PARAMS_H
#define PARAMS_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "VL53L0X.h"
#include "canBus.h"
#include "mros.h"
#include "pins.h"
#include "structs.h"

#define MTOF_ADDR  41U
#define TOF_F_ADDR 0x30
#define TOF_B_ADDR 0x31
#define TOF_L_ADDR 0x32

#define CMD_VEL_TIMEOUT 100  //(x10 ms)
#define PC_TIMEOUT      30   //(x update())
#define MC_TIMEOUT      30   //(x update())

extern bool volatile per_ms10_flag;
extern bool volatile per_ms100_flag;
extern bool volatile per_sec_flag;
extern bool volatile per_sec10_flag;

void per_sec_init(void);
void init_gpio(void);

#endif