#ifndef STRUCTS_H
#define STRUCTS_H

#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>

typedef enum {
    STATE_INIT   = 0x00,
    STATE_ACTIVE = 0x01
} MCState_t;

typedef enum {
    STATUS_OK = 0x00,
    STATUS_TO = 0x01
} MCStatus_t;

typedef enum {
    ERR_MROS = 0x00,
    ERR_NRF  = 0x01,
    ERR_CAN  = 0x02,
    ERR_IMU  = 0x03,
    ERR_CLS  = 0xFF
} MCErr_off_t;

typedef struct {
    int16_t voltage;
    int16_t current;
    int16_t power;

    uint8_t timeout_c;
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
        int16_t raw;
    } state;

    PBus_t solar;
    PBus_t mppt;
    PBus_t bat;
    PBus_t v5;
    PBus_t v12a;
    PBus_t v12b;

    uint16_t temp;
    uint8_t timeout_c;
} PC_t;

typedef struct {
    union {
        struct {
            uint16_t mode : 2;

            uint16_t req_stat : 1;
            uint16_t req_enc : 1;
            uint16_t req_all : 1;
            uint16_t req : 2;
        } bits;
        uint16_t raw;
    } state;

    uint16_t temp;

    int32_t enc_m1;
    int32_t enc_m2;
    int32_t enc_m3;
    int32_t enc_m4;

    uint8_t timeout_c;
} MC_t;

typedef struct {
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
} quat_t;

typedef struct {
    float x;
    float y;
    float theta;

    int16_t m_left;
    int16_t m_right;

    uint8_t timeout_c;
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

    Adafruit_BNO055* bno055;
    imu::Quaternion  imu_quat;
    imu::Vector<3>   imu_accel;
    imu::Vector<3>   imu_gyro;
    uint8_t          mag_cal;

    uint16_t             tof_data[3];
    VL53L5CX_ResultsData measurementData;

    MCStatus_t init(void);
    MCStatus_t update(void);
    MCStatus_t updateBNO(void);

    MCStatus_t mrosGetUpdate(void);

    MCStatus_t pcGetBus(void);
    MCStatus_t pcGetTemp(void);
    MCStatus_t pcSetMode(void);

    MCStatus_t mcGetTemp(void);
    MCStatus_t mcGetEnc(void);
    MCStatus_t mcSetVel(void);
};

#endif