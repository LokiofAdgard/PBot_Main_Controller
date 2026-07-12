#ifndef CANBUS_H
#define CANBUS_H

#include "driver/twai.h"
#include "pins.h"
#include "structs.h"

#define CAN_ID_STA 0x100
#define CAN_ID_INA 0x101
#define CAN_ID_TMP 0x102

#define INA_SOL_ADDR  (0x40 << 1)
#define INA_MPPT_ADDR (0x41 << 1)
#define INA_BAT_ADDR  (0x42 << 1)
#define INA_V5_ADDR   (0x43 << 1)
#define INA_V12A_ADDR (0x44 << 1)
#define INA_V12B_ADDR (0x45 << 1)

extern twai_message_t rx;

void init_can(void);
void can_update(MController* mc);

#endif