#include <Arduino.h>
#include <Wire.h>

#include "structs.h"
#include "mros.h"

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
