#ifndef BLE_SPP_SERVER_H
#define BLE_SPP_SERVER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void ble_spp_server_init(void);
    void ble_spp_server_send_data(uint8_t *data, uint16_t len);
    void update_ble_connection_status(bool connected);

#ifdef __cplusplus
}
#endif

#endif // BLE_SPP_SERVER_H
