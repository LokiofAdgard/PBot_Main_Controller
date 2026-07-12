#include "canBus.h"

twai_message_t rx;

// bool can_query_id(uint32_t id, uint32_t timeout_ms) {
//     twai_message_t rtr   = {};
//     rtr.identifier       = id;
//     rtr.extd             = 0;
//     rtr.rtr              = 1;
//     rtr.data_length_code = 0;

//     if (twai_transmit(&rtr, pdMS_TO_TICKS(10)) != ESP_OK) {
//         return false;
//     }

//     uint32_t       start = millis();
//     twai_message_t resp;

//     while (millis() - start < timeout_ms) {
//         if (twai_receive(&resp, pdMS_TO_TICKS(10)) == ESP_OK) {
//             if (!resp.rtr && resp.identifier == id) {
//                 return true;
//             }
//         }
//     }

//     return false;
// }

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
    // if (err == ESP_OK) {
    // } else {
    // }
}

void can_update(MController* mc) {
    if (twai_receive(&rx, 0) != ESP_OK && !rx.rtr) return;

    switch (rx.identifier) {
        case CAN_ID_INA:
            PBus_t* bus;
            switch (rx.data[6]) {
                case INA_SOL_ADDR:
                    bus = &mc->powerc.solar;
                    break;
                case INA_MPPT_ADDR:
                    bus = &mc->powerc.mppt;
                    break;

                default:
                    break;
            }
            bus->voltage = (int16_t(rx.data[1] << 8 | rx.data[0])) * 1.25e-3f;
            bus->current = (int16_t(rx.data[3] << 8 | rx.data[2])) * 0.4f;
            bus->power   = (int16_t(rx.data[5] << 8 | rx.data[4])) * 0.0025f * 0.4f;
            break;

        default:
            break;
    }
}