#include "canBus.h"

extern MController mController;

volatile bool  can_available = false;
twai_message_t rx;

void can_rx_task(void* arg);

void can_rx_task(void* arg) {
    while (true) {
        if (twai_receive(&rx, portMAX_DELAY) == ESP_OK && !rx.rtr) {
            can_available = true;
            can_update(&mController);
        }
    }
}

void init_can() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX,
        (gpio_num_t)CAN_RX,
        TWAI_MODE_NORMAL);

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err;

    err = twai_driver_install(&g_config, &t_config, &f_config);
    err = twai_start();

    xTaskCreate(can_rx_task, "CAN_RX", 2048, NULL, 20, NULL);
}

void can_update(MController* mc) {
    switch (rx.identifier) {
        case PC_ID_INA:
            PBus_t* bus;
            switch (rx.data[6]) {
                case INA_SOL_ADDR:
                    bus = &mc->powerc.solar;
                    break;
                case INA_MPPT_ADDR:
                    bus = &mc->powerc.mppt;
                    break;
                case INA_BAT_ADDR:
                    bus = &mc->powerc.bat;
                    break;
                case INA_V5_ADDR:
                    bus = &mc->powerc.v5;
                    break;
                case INA_V12A_ADDR:
                    bus = &mc->powerc.v12a;
                    break;
                case INA_V12B_ADDR:
                    bus = &mc->powerc.v12b;
                    break;

                default:
                    break;
            }
            bus->voltage = (rx.data[1] << 8 | rx.data[0]);
            bus->current = (rx.data[3] << 8 | rx.data[2]);
            bus->power   = (rx.data[5] << 8 | rx.data[4]);
            break;

        case PC_ID_STA:
            mc->powerc.state.raw = (rx.data[1] << 8 | rx.data[0] << 0);
            mc->powerc.temp      = (rx.data[3] << 8 | rx.data[2] << 0);
            break;

        case MC_ID_STA:
            mc->powerc.state.raw = (rx.data[1] << 8 | rx.data[0] << 0);
            mc->motorc.temp      = (rx.data[3] << 8 | rx.data[2] << 0);
            break;

        case MC_ID_ENC:
            break;

        default:
            break;
    }
}

bool can_tx(Txid_t id, const uint8_t* data, uint8_t dlc) {
    twai_message_t msg   = {};
    msg.identifier       = id;
    msg.extd             = 0;  // Standard ID
    msg.rtr              = 0;  // Data frame
    msg.data_length_code = dlc;

    for (int i = 0; i < dlc; i++)
        msg.data[i] = data[i];

    return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
}

void pc_set_mode(Txid_t id, pc_mode_t mode) {
    uint8_t buf[8];
    buf[0] = mode;
    can_tx(PC_SET_MODE, buf, 1);
}

void can_req(Txid_t id, Req_t req) {
    uint8_t buf[1] = {req};
    can_tx(id, buf, 1);
}

void can_tx_cmdvel(Cmd_vel_t cmd_vel) {
    uint8_t buf[8];
    buf[0] = (uint8_t)((cmd_vel.m_left >> 0) & 0xFF);
    buf[1] = (uint8_t)((cmd_vel.m_left >> 8) & 0xFF);
    buf[2] = (uint8_t)((cmd_vel.m_right >> 0) & 0xFF);
    buf[3] = (uint8_t)((cmd_vel.m_right >> 8) & 0xFF);
    can_tx(MC_CMD_VEL, buf, 4);
}
