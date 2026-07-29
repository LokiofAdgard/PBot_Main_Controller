#include "params.h"

hw_timer_t*      per_sec_timer  = NULL;
volatile uint8_t ms10_couter    = 0;
volatile uint8_t ms100_couter   = 0;
volatile uint8_t sec_couter     = 0;
volatile bool    per_ms10_flag  = false;
volatile bool    per_ms100_flag = false;
volatile bool    per_sec_flag   = false;
volatile bool    per_sec10_flag = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

adafruit_bno055_offsets_t savedOffsets = {
    .accel_offset_x = 1,
    .accel_offset_y = -2,
    .accel_offset_z = 20,

    .mag_offset_x = -908,
    .mag_offset_y = 1492,
    .mag_offset_z = 322,

    .gyro_offset_x = -3,
    .gyro_offset_y = -3,
    .gyro_offset_z = 0,

    .accel_radius = 1000,
    .mag_radius   = 756,
};

void IRAM_ATTR on_per_ms10_timer() {
    per_ms10_flag = true;
    if (ms10_couter++ >= 10) {
        ms10_couter = 0;

        per_ms100_flag = true;
        if (ms100_couter++ >= 10) {
            ms100_couter = 0;

            per_sec_flag   = true;
            per_ms100_flag = true;
            if (sec_couter++ >= 10) {
                sec_couter = 0;

                per_sec10_flag = true;
            }
        }
    }
}

void per_sec_init(void) {
    per_sec_timer = timerBegin(0, 80, true);  // 80 prescaler = 1us per tick
    timerAttachInterrupt(per_sec_timer, &on_per_ms10_timer, true);
    timerAlarmWrite(per_sec_timer, 10000, true);  // autoreload = true
    timerAlarmEnable(per_sec_timer);
}

void init_gpio() {
    pinMode(CE, OUTPUT);
    // pinMode(LED_RGB, OUTPUT);
    // pinMode(LED_STR, OUTPUT);
}

void MController::set_err(MCErr_off_t err, bool cls) {
    if (err == ERR_CLS) {
        err_reg = 0x00;
        return;
    }
    if (cls)
        this->err_reg &= ~(1 << err);
    else
        this->err_reg |= (1 << err);
}

MCStatus_t MController::updateBNO(void) {
    bno055->getCalibration(nullptr, nullptr, nullptr, &mag_cal);

    // Read IMU values
    imu_quat  = bno055->getQuat();
    imu_accel = bno055->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu_gyro  = bno055->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    return STATUS_OK;
}

MCStatus_t MController::init(void) {
    per_sec_init();
    mros_init(Serial);
    init_gpio();
    init_can();
    Wire.begin(SDA2, SCL2);
    bno.begin();
    bno.setMode(OPERATION_MODE_IMUPLUS);
    bno.setExtCrystalUse(true);
    bno.setSensorOffsets(savedOffsets);
    this->bno055 = &bno;
    return STATUS_OK;
}

MCStatus_t MController::update(void) {
    updateBNO();
    return STATUS_OK;
}

MCStatus_t MController::mrosGetUpdate(void) {
    // this->cmd_vel.x     = mros_get_cmd_vel_lin_x();
    // this->cmd_vel.y     = mros_get_cmd_vel_lin_y();
    // this->cmd_vel.theta = mros_get_cmd_vel_ang_z();

    this->cmd_vel.m_left  = mros_get_motor_l();
    this->cmd_vel.m_right = mros_get_motor_r();

    return STATUS_OK;
}