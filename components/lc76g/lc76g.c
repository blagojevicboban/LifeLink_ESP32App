#include "lc76g.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "LC76G";
static i2c_port_t g_i2c_port = I2C_NUM_0;

esp_err_t lc76g_init(i2c_port_t i2c_num)
{
    g_i2c_port = i2c_num;
    ESP_LOGI(TAG, "Initialized LC76G on I2C port %d", i2c_num);
    // Note: I2C driver is assumed to be initialized by the main app/sensorlib
    return ESP_OK;
}

esp_err_t lc76g_read_data(uint8_t *buffer, size_t max_len, size_t *read_len)
{
    *read_len = 0;
    esp_err_t ret;

    // 1. Initial Write to checking availability or trigger
    // Protocol from Arduino: { 0x08, 0x00, 0x51, 0xAA, 0x04, 0x00, 0x00, 0x00 }
    uint8_t init_cmd[] = {0x08, 0x00, 0x51, 0xAA, 0x04, 0x00, 0x00, 0x00};

    ret = i2c_master_write_to_device(g_i2c_port, LC76G_ADDR_W, init_cmd, sizeof(init_cmd), pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        // It's possible the device NACKs if busy or sleeping, suppress excessive logs?
        // ESP_LOGW(TAG, "Failed to write init command: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Arduino uses 100ms

    // 2. Read Data Length (4 bytes) from 0x54
    // 2. Read Data Length (4 bytes) from 0x54
    uint8_t len_buf[4] = {0};
    int retries = 3;
    while (retries > 0)
    {
        ret = i2c_master_read_from_device(g_i2c_port, LC76G_ADDR_R, len_buf, sizeof(len_buf), pdMS_TO_TICKS(1000));
        if (ret == ESP_OK)
            break;

        // Don't log error immediately, just wait and retry
        vTaskDelay(pdMS_TO_TICKS(20));
        retries--;
    }

    if (ret != ESP_OK)
    {
        // Only log error if all retries failed
        ESP_LOGD(TAG, "Failed to read length: %s", esp_err_to_name(ret));
        return ret;
    }

    uint32_t data_len = (len_buf[0]) | (len_buf[1] << 8) | (len_buf[2] << 16) | (len_buf[3] << 24);

    if (data_len == 0)
    {
        // No data available
        return ESP_OK;
    }

    if (data_len > max_len)
    {
        ESP_LOGD(TAG, "Data length %lu exceeds buffer %u, truncating", data_len, max_len);
        data_len = max_len;
    }

    // 3. Prepare Read Command
    // Protocol: { 0x00, 0x20, 0x51, 0xAA } + 4 bytes of length previously read
    uint8_t read_cmd[8];
    uint8_t cmd_header[] = {0x00, 0x20, 0x51, 0xAA};
    memcpy(read_cmd, cmd_header, sizeof(cmd_header));
    memcpy(read_cmd + 4, len_buf, 4);

    vTaskDelay(pdMS_TO_TICKS(10)); // Wait a bit before requesting data

    ret = i2c_master_write_to_device(g_i2c_port, LC76G_ADDR_W, read_cmd, sizeof(read_cmd), pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        ESP_LOGD(TAG, "Failed to send read request: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for data preparation

    // 4. Read Actual Data
    ret = i2c_master_read_from_device(g_i2c_port, LC76G_ADDR_R, buffer, data_len, pdMS_TO_TICKS(200));
    if (ret == ESP_OK)
    {
        *read_len = data_len;
    }
    else
    {
        ESP_LOGD(TAG, "Failed to read data payload: %s", esp_err_to_name(ret));
    }

    return ret;
}
