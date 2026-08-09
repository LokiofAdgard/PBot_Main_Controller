#include "params.h"

hw_timer_t*      per_sec_timer  = NULL;
volatile uint8_t ms10_couter    = 0;
volatile uint8_t ms100_couter   = 0;
volatile uint8_t sec_couter     = 0;
volatile bool    per_ms10_flag  = false;
volatile bool    per_ms100_flag = false;
volatile bool    per_sec_flag   = false;
volatile bool    per_sec10_flag = false;

TwoWire i2c1 = TwoWire(0);
TwoWire i2c2 = TwoWire(1);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &i2c2);

SparkFun_VL53L5CX mTof;

VL53L0X tof_f;
VL53L0X tof_b;
VL53L0X tof_l;

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
    if (++ms10_couter >= 10) {
        ms10_couter = 0;

        per_ms100_flag = true;
        if (++ms100_couter >= 10) {
            ms100_couter = 0;

            per_sec_flag = true;
            if (++sec_couter >= 10) {
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

void inaTimeout(PBus_t* bus) {
    if (bus->timeout_c > 40) return;
    if (bus->timeout_c++ >= 30) {
        bus->voltage = 0;
        bus->current = 0;
        bus->power   = 0;
    }
}

void pc_timeout(PC_t* pc) {
    inaTimeout(&pc->solar);
    inaTimeout(&pc->mppt);
    inaTimeout(&pc->bat);
    inaTimeout(&pc->v5);
    inaTimeout(&pc->v12a);
    inaTimeout(&pc->v12b);

    if (pc->timeout_c > PC_TIMEOUT + 10) return;
    if (pc->timeout_c++ >= PC_TIMEOUT) {
        pc->state.raw = 0;
        pc->temp      = 0;
    }
}

void mc_timeout(MC_t* mc) {
    if (mc->timeout_c > MC_TIMEOUT) return;
    if (mc->timeout_c++ >= MC_TIMEOUT) {
        mc->state.raw = 0;
        mc->temp      = 0;
        // mc->enc_m1 = 0;
        // mc->enc_m2 = 0;
        // mc->enc_m3 = 0;
        // mc->enc_m4 = 0;
    }
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
#ifdef WIFI_MODE
    Serial.begin(115200);
#endif
    per_sec_init();
    mros_init(Serial);
    init_gpio();
    init_can();

    i2c1.begin(SDA1, SCL1, 400000);
    i2c2.begin(SDA2, SCL2, 400000);

    bno.begin();
    bno.setMode(OPERATION_MODE_IMUPLUS);
    bno.setExtCrystalUse(true);
    bno.setSensorOffsets(savedOffsets);
    this->bno055 = &bno;

    if (mTof.begin(MTOF_ADDR, i2c1)) {
        mTof.setResolution(8 * 8);
        mTof.setRangingFrequency(10);
        mTof.startRanging();
    }

    pinMode(XSHUT_F, OUTPUT);
    pinMode(XSHUT_B, OUTPUT);
    pinMode(XSHUT_L, OUTPUT);

    digitalWrite(XSHUT_F, LOW);
    digitalWrite(XSHUT_B, LOW);
    digitalWrite(XSHUT_L, LOW);
    delay(10);

    digitalWrite(XSHUT_F, HIGH);
    delay(10);
    tof_f.setBus(&i2c1);
    if (tof_f.init()) tof_f.startContinuous();
    tof_f.setAddress(TOF_F_ADDR);

    digitalWrite(XSHUT_B, HIGH);
    delay(10);
    tof_b.setBus(&i2c1);
    if (tof_b.init()) tof_b.startContinuous();
    tof_b.setAddress(TOF_B_ADDR);

    digitalWrite(XSHUT_L, HIGH);
    delay(10);
    tof_l.setBus(&i2c1);
    if (tof_l.init()) tof_l.startContinuous();
    tof_l.setAddress(TOF_L_ADDR);

#ifdef WIFI_MODE
    server_init();
#endif

    return STATUS_OK;
}

MCStatus_t MController::update(void) {
    updateBNO();
    if (mTof.isDataReady()) mTof.getRangingData(&measurementData);
    tof_data[0] = tof_f.readRangeContinuousMillimeters();
    tof_data[1] = tof_b.readRangeContinuousMillimeters();
    tof_data[2] = tof_l.readRangeContinuousMillimeters();

    pc_timeout(&powerc);
    mc_timeout(&motorc);

    return STATUS_OK;
}

MCStatus_t MController::mrosGetUpdate(void) {
    // this->cmd_vel.x     = mros_get_cmd_vel_lin_x();
    // this->cmd_vel.y     = mros_get_cmd_vel_lin_y();
    // this->cmd_vel.theta = mros_get_cmd_vel_ang_z();

    if (cmd_vel.timeout_c > CMD_VEL_TIMEOUT + 10) return STATUS_TO;
    if (cmd_vel.timeout_c++ > CMD_VEL_TIMEOUT) {
        this->cmd_vel.m_left  = 0;
        this->cmd_vel.m_right = 0;
        return STATUS_TO;
    }

    this->cmd_vel.m_left  = mros_get_motor_l();
    this->cmd_vel.m_right = mros_get_motor_r();

    return STATUS_OK;
}