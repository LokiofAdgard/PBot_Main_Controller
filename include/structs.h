#ifndef STRUCTS_H
#define STRUCTS_H

#include <Arduino.h>

typedef enum {
    STATE_INIT   = 0x00,
    STATE_ACTIVE = 0x01
} MCState_t;

typedef enum {
    STATUS_OK = 0x00
} MCStatus_t;

typedef enum {
    ERR_BAT = 0x00
} MCErr_off_t;

typedef struct {
    float voltage;
    float current;
    float power;
} PBus_t;

typedef struct {
    PBus_t solar;
    PBus_t mppt;
    PBus_t bat;
    PBus_t v5;
    PBus_t v12a;
    PBus_t v12b;

    uint8_t temp;
} PC_t;

typedef struct {
    uint8_t temp;

    int32_t enc_m1;
    int32_t enc_m2;
    int32_t enc_m3;
    int32_t enc_m4;
} MC_t;

typedef struct {
    float x;
    float y;
    float theta;
} Cmd_vel_t;

#endif