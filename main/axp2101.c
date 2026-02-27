#include "axp2101.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "AXP2101";
static i2c_port_t g_axp_i2c_port = I2C_NUM_0;

esp_err_t axp_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(g_axp_i2c_port, AXP2101_I2C_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

esp_err_t axp_read_byte(uint8_t reg, uint8_t *data)
{
    return i2c_master_write_read_device(g_axp_i2c_port, AXP2101_I2C_ADDR, &reg, 1, data, 1, pdMS_TO_TICKS(100));
}

esp_err_t axp_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(g_axp_i2c_port, AXP2101_I2C_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

esp_err_t axp_enable_power(void)
{
    esp_err_t err = ESP_OK;

    // Enable all LDOs in Reg 0x90 (ALDO1-4, BLDO1-2, CPUSLDO, DLDO1)
    err |= axp_write_byte(0x90, 0xFF);

    // Enable DLDO2 in Reg 0x91 (bit 0)
    err |= axp_write_byte(0x91, 0x01);

    // Default voltages are usually set by the hardware pull-ups or bootloader
    // But forcing the LDOs on ensures the GSM rail is up.

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "AXP2101 Power Rails (LDOs) Enabled.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to enable AXP2101 Power Rails.");
    }

    return err;
}

esp_err_t axp_init(i2c_port_t i2c_num)
{
    g_axp_i2c_port = i2c_num;
    uint8_t val;
    esp_err_t ret = axp_read_byte(0x03, &val); // Read IC type or similar check
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "AXP2101 Detected. ID Reg: 0x%02X", val);

        // Enable ADC for battery voltage (usually enabled by default on AXP2101, but good to check)
        // AXP2101 is quite automated compared to AXP192.

        // Turn on all power rails (LDOs/DC-DCs) to ensure GSM module gets power
        axp_enable_power();
        
        // --- Configure Battery Charging ---
        // 1. Enable Charging (Register 0x18, Bit 1 is charge enable, Bit 0 is charge path)
        uint8_t power_cfg;
        if (axp_read_byte(0x18, &power_cfg) == ESP_OK) {
            axp_write_byte(0x18, power_cfg | 0x02); // Ensure CHG_EN is set
        }

        // 2. Set Target Charge Voltage to 4.2V (Register 0x64 [2:0], 0=4.1V, 1=4.2V, 2=4.35V)
        uint8_t target_vol;
        if (axp_read_byte(0x64, &target_vol) == ESP_OK) {
            target_vol = (target_vol & 0xF8) | 0x01; // Set bits [2:0] to 001 for 4.2V
            axp_write_byte(0x64, target_vol);
        }

        // 3. Set Fast Charge Current to 1000mA (BP-5M is a big battery)
        axp_set_charge_current(1000);

        return ESP_OK;
    }
    ESP_LOGE(TAG, "AXP2101 Not Found!");
    return ESP_FAIL;
}

int axp_get_batt_vol(void)
{
    uint8_t buf[2];
    // Battery Voltage: Reg 0x34 (High), 0x35 (Low) - 14 bit
    if (axp_read_bytes(0x34, buf, 2) == ESP_OK)
    {
        uint16_t raw = ((buf[0] & 0x3F) << 8) | buf[1];
        // Based on typical AXP2101 scaling, raw is half the millivolts
        return (int)raw * 2;
    }
    return -1;
}

int axp_get_batt_percent(void)
{
    uint8_t val;
    // Battery Percentage: Reg 0xA4. 0-100%
    if (axp_read_byte(0xA4, &val) == ESP_OK)
    {
        // MSB bit 7 is "valid" flag? Or just straight value.
        // Usually 0-100 decimal.
        return (int)(val & 0x7F);
    }
    return -1;
}

esp_err_t axp_set_charge_current(uint16_t ma)
{
    uint8_t val = 0;
    
    // According to AXP2101 documentation, register 0x62 sets the ICC charger current.
    // Range 1 is 0mA-200mA in 25mA steps. Range 2 is 200mA-1000mA (or 1500mA) in 100mA steps.
    
    if (ma <= 200) {
        val = ma / 25; // 0=0mA, 1=25mA, 4=100mA, 8=200mA
    } else if (ma <= 1000) { // Limit to 1000mA for safety
        val = 8 + ((ma - 200) / 100); // 8=200mA, 9=300mA... 16=1000mA
    } else {
        val = 16; // Max 1000mA default safe limit
    }

    ESP_LOGI(TAG, "Setting AXP2101 charge current to %d mA (val: 0x%02X)", ma, val);
    
    // Register 0x62 [4:0] sets the charge current
    uint8_t current_reg;
    esp_err_t err = axp_read_byte(0x62, &current_reg);
    if (err == ESP_OK) {
        current_reg = (current_reg & 0xE0) | (val & 0x1F); // Keep top 3 bits, update bottom 5 bits
        return axp_write_byte(0x62, current_reg);
    }
    return err;
}

bool axp_is_charging(void)
{
    uint8_t val;
    // Status Register 0x01. Bit 5 typically indicates charging active on AXP series.
    // Register 0x00 indicates power status (VBUS presence etc)
    if (axp_read_byte(0x01, &val) == ESP_OK)
    {
        // On AXP2101, reg 0x01 bit 5 (0x20) indicates Battery Charging
        return (val & 0x20) ? true : false;
    }
    return false;
}
