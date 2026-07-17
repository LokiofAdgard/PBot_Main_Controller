#ifndef CANBUS_H
#define CANBUS_H

#include "driver/twai.h"
#include "pins.h"
#include "structs.h"

typedef enum {
    PC_ID_STA = 0x100,
    PC_ID_INA = 0x101,

    MC_ID_STA = 0x1F0,
    MC_ID_ENC = 0x1F1
} Rxid_t;

typedef enum {
    PC_SET_MODE = 0x200,
    PC_DATA_REQ = 0x201,

    MC_SET_MODE = 0x300,
    MC_CMD_VEL  = 0x301
} Txid_t;

typedef enum {
    GET_STAT    = 0x00,
    GET_INA     = 0x01,
    GET_ALL     = 0x02,
    SET_PWR_ON  = 0x03,
    SET_PWR_SAV = 0x04
} Req_t;

typedef enum {
    MODE_INIT    = 0x00,
    MODE_PWR_ON  = 0x01,
    MODE_PWR_SAV = 0x02,
    MODE_ERR     = 0x03
} pc_mode_t;

#define INA_SOL_ADDR  (0x40 << 1)
#define INA_MPPT_ADDR (0x41 << 1)
#define INA_BAT_ADDR  (0x42 << 1)
#define INA_V5_ADDR   (0x43 << 1)
#define INA_V12A_ADDR (0x44 << 1)
#define INA_V12B_ADDR (0x45 << 1)

extern volatile bool  can_available;
extern twai_message_t rx;

void init_can(void);
void can_update(MController* mc);
bool can_tx(uint32_t id, const uint8_t* data, uint8_t dlc);
void can_req(Txid_t id, Req_t req);

#endif