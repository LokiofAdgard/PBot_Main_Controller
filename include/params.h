#include <Arduino.h>
#include <Wire.h>

#include "mros.h"
#include "structs.h"

extern bool volatile per_ms10_flag;
extern bool volatile per_ms100_flag;
extern bool volatile per_sec_flag;
extern bool volatile per_sec10_flag;

void per_sec_init(void);

class MController {
    private:
    MCState_t state;
    uint8_t   err_reg;

    public:
    uint8_t   vbat;
    Cmd_vel_t cmd_vel;
    PC_t      powerc;
    MC_t      motorc;

    MCStatus_t init(void);
    MCStatus_t update(void);
};
