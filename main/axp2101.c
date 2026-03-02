#include "axp2101.h"
#include "esp_log.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

    // --- Set Voltages for Waveshare AMOLED requirements ---
    // ALDO1 = 3.3V (Reg 0x92, value 3300mV - 500mV / 100mV step? No, AXP2101 is usually 0.5V to 3.5V)
    // For AXP2101: 0x00=0.5V, 0x1E=3.5V. (Value = (V - 0.5) / 0.1)
    // 3.3V => (3.3 - 0.5) / 0.1 = 28 (0x1C)
    // 1.8V => (1.8 - 0.5) / 0.1 = 13 (0x0D)
    
    axp_write_byte(0x92, 0x1C); // ALDO1 = 3.3V
    axp_write_byte(0x93, 0x0D); // ALDO2 = 1.8V
    vTaskDelay(pdMS_TO_TICKS(10));

    // --- Enable LDOs ---
    uint8_t current_90 = 0;
    if (axp_read_byte(0x90, &current_90) == ESP_OK) {
        // Enable ALDO1, ALDO2, ALDO3, ALDO4, BLDO1, BLDO2 (0xFF enables all)
        err |= axp_write_byte(0x90, 0xFF); 
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Enable DLDO1/2 in Reg 0x91
    err |= axp_write_byte(0x91, 0x03);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "AXP2101 Power Rails (LDOs) Configured & Enabled.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to configure AXP2101 Power Rails.");
    }

    return err;
}

esp_err_t axp_init(i2c_port_t i2c_num)
{
    g_axp_i2c_port = i2c_num;

    // =========================================================================
    // CRITICAL FIX (FIRST ACTION - NO DELAYS):
    // A previous firmware accidentally wrote 0x02 to AXP2101 REG[10h].
    // On AXP2101, REG[10h] bit1 = "PWROFF" (power-off request).
    // This value is battery-backed and PERSISTS across reboots/reflashes!
    // AXP2101 executes the PWROFF ~1 second after startup if bit1 is still set.
    // Fix: Read REG[10h] immediately and clear bit1 before anything else runs.
    // =========================================================================
    uint8_t reg10 = 0;
    if (axp_read_byte(0x10, &reg10) == ESP_OK) {
        ESP_LOGI(TAG, "REG[10h] on startup = 0x%02X", reg10);
        if (reg10 & 0x02) {
            ESP_LOGW(TAG, "!!! PWROFF bit is SET in REG[10h] - clearing it NOW to stop 1s shutdown loop !!!");
            axp_write_byte(0x10, reg10 & ~0x02); // Clear ONLY the PWROFF bit (bit1)
            ESP_LOGI(TAG, "REG[10h] PWROFF bit cleared. System will stay ON.");
        } else {
            ESP_LOGI(TAG, "REG[10h] PWROFF bit is clear - OK.");
        }
    }

    uint8_t val;
    esp_err_t ret = axp_read_byte(0x03, &val); // Read chip ID
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "AXP2101 Detected. ID Reg: 0x%02X", val);

        // --- DIAGNOSTIC: Read power status registers ---
        uint8_t pow_src, pow_stat, chg_stat, reg27;
        axp_read_byte(0x00, &pow_src);
        axp_read_byte(0x01, &pow_stat);
        axp_read_byte(0x04, &chg_stat);
        axp_read_byte(0x27, &reg27);
        ESP_LOGI(TAG, "AXP STAT: [00h]=%02X [01h]=%02X [04h]=%02X [27h]=%02X",
                 pow_src, pow_stat, chg_stat, reg27);

        if (pow_src & 0x08) {
            ESP_LOGI(TAG, "AXP: Battery PRESENT");
        } else {
            ESP_LOGW(TAG, "AXP: Battery NOT detected in REG[00h]");
        }
        if (pow_src & 0x20) {
            ESP_LOGI(TAG, "AXP: VBUS (USB) present");
        }

        // --- Enable power rails (staggered) ---
        axp_enable_power();

        // --- Battery Charging config ---
        uint8_t power_cfg;
        if (axp_read_byte(0x18, &power_cfg) == ESP_OK) {
            // bit 1: charge enable, bit 3: fuel gauge enable
            axp_write_byte(0x18, power_cfg | 0x0A); 
        }
        uint8_t target_vol;
        if (axp_read_byte(0x64, &target_vol) == ESP_OK) {
            axp_write_byte(0x64, (target_vol & 0xF8) | 0x01); // 4.2V
        }
        axp_set_charge_current(500);

        ESP_LOGI(TAG, "AXP2101 fully initialized. Boot loop should be GONE.");
        return ESP_OK;
    }
    ESP_LOGE(TAG, "AXP2101 Not Found on I2C!");
    return ESP_FAIL;
}

int axp_get_batt_vol(void)
{
    uint8_t buf[2];
    // Battery Voltage: Reg 0x34 (High), 0x35 (Low) - 14 bit
    if (axp_read_bytes(0x34, buf, 2) == ESP_OK)
    {
        uint16_t raw = ((buf[0] & 0x3F) << 8) | buf[1];
        // AXP2101 ADC step for battery voltage is exactly 1mV per LSB.
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
