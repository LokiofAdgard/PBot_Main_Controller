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

#define MTOF_ADDR 41U

extern bool volatile per_ms10_flag;
extern bool volatile per_ms100_flag;
extern bool volatile per_sec_flag;
extern bool volatile per_sec10_flag;

void per_sec_init(void);
void init_gpio(void);

#endif