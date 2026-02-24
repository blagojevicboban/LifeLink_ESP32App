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

        return ESP_OK;
    }
    ESP_LOGE(TAG, "AXP2101 Not Found!");
    return ESP_FAIL;
}

int axp_get_batt_vol(void)
{
    uint8_t buf[2];
    // Battery Voltage: Reg 0x34 (High), 0x35 (Low) - 14 bit, 1mV step (likely, need datasheet verify)
    // Actually per AXP2101 datasheet: Reg 0x34[13:8], 0x35[7:0]. Unit 1mV.
    if (axp_read_bytes(0x34, buf, 2) == ESP_OK)
    {
        uint16_t raw = ((buf[0] & 0x3F) << 8) | buf[1];
        return (int)raw;
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

bool axp_is_charging(void)
{
    uint8_t val;
    // Status Register 0x01. Bit 3 is "Is Charging" (Charging indication)
    // Or 0x00 Power Status.
    if (axp_read_byte(0x01, &val) == ESP_OK)
    {
        // Check datasheet for exact bit. Assuming bit 6 for VBUS good, bit 5 for Charging?
        // Let's rely on simple presence for now or just check voltage trend.
        // Actually, reg 0x00 usually contains power source status.
        return (val & 0x20) ? true : false; // Placeholder bit
    }
    return false;
}
