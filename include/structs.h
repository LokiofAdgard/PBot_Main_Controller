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
    ERR_MROS = 0x00,
    ERR_NRF  = 0x01,
    ERR_CAN  = 0x02,
    ERR_IMU  = 0x03,
    ERR_CLS  = 0xFF
} MCErr_off_t;

typedef struct {
    float voltage;
    float current;
    float power;
} PBus_t;

typedef struct {
    union {
        struct {
            uint16_t mode : 2;

            uint16_t sol : 1;
            uint16_t solOut : 1;
            uint16_t mpptIn : 1;
            uint16_t bat : 1;

            uint16_t en_v5 : 1;
            uint16_t en_v12a : 1;
            uint16_t en_v12b : 1;

            uint16_t en_fan : 1;

            uint16_t req : 4;
        } bits;
        uint16_t raw;
    } state;

    PBus_t solar;
    PBus_t mppt;
    PBus_t bat;
    PBus_t v5;
    PBus_t v12a;
    PBus_t v12b;

    uint16_t temp;
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

class MController {
    private:
    MCState_t state;
    uint8_t   err_reg;

    void set_err(MCErr_off_t err, bool cls = false);

    public:
    uint8_t   vbat;
    Cmd_vel_t cmd_vel;
    PC_t      powerc;
    MC_t      motorc;

    MCStatus_t init(void);
    MCStatus_t update(void);

    MCStatus_t mrosGetUpdate(void);

    MCStatus_t pcGetBus(void);
    MCStatus_t pcGetTemp(void);
    MCStatus_t pcSetMode(void);

    MCStatus_t mcGetTemp(void);
    MCStatus_t mcGetEnc(void);
    MCStatus_t mcSetVel(void);
};

#endif