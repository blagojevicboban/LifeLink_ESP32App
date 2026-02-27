#ifndef AXP2101_H
#define AXP2101_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/i2c.h"
#include "esp_err.h"

// I2C Address
#define AXP2101_I2C_ADDR 0x34

    /**
     * @brief Initialize AXP2101 Power Management Unit
     * @param i2c_num I2C port number (must be initialized by main app)
     * @return ESP_OK on success
     */
    esp_err_t axp_init(i2c_port_t i2c_num);

    /**
     * @brief Enable AXP2101 Power Rails (LDOs/DC-DCs) required for peripherals
     * @return ESP_OK on success
     */
    esp_err_t axp_enable_power(void);

    /**
     * @brief Get Battery Voltage in millivolts
     * @return voltage in mV or -1 on error
     */
    int axp_get_batt_vol(void);

    /**
     * @brief Get Battery Percentage (0-100)
     * @return percentage or -1 on error
     */
    int axp_get_batt_percent(void);

    /**
     * @brief Set battery charge current
     * @param ma Target charge current in mA (0, 100, 125, 150, 175, 200, 300, 400, 500, 600, 700, 800, 900, 1000)
     * @return ESP_OK on success
     */
    esp_err_t axp_set_charge_current(uint16_t ma);

    /**
     * @brief Check if charging
     * @return true if charging, false otherwise
     */
    bool axp_is_charging(void);

#ifdef __cplusplus
}
#endif

#endif // AXP2101_H
