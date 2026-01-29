#ifndef LC76G_H
#define LC76G_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/i2c.h"
#include "esp_err.h"

// I2C Addresses for LC76G
#define LC76G_ADDR_W 0x50
#define LC76G_ADDR_R 0x54

// Default I2C Configuration (should match your existing system)
#define LC76G_I2C_MASTER_NUM I2C_NUM_0
#define LC76G_I2C_MASTER_FREQ_HZ 100000

    /**
     * @brief Initialize the LC76G driver.
     *
     * @param i2c_num The I2C port number to use (e.g., I2C_NUM_0).
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t lc76g_init(i2c_port_t i2c_num);

    /**
     * @brief Read available NMEA data from the LC76G.
     *
     * This function performs the custom write-read sequence required by the module.
     * It allocates a buffer internally if data is available, copies it to the output,
     * and user must free it? No, let's keep it simple: user provides buffer.
     *
     * @param buffer Pointer to the buffer to store data.
     * @param max_len Maximum length of the buffer.
     * @param read_len Pointer to store the actual number of bytes read.
     * @return esp_err_t ESP_OK on success (even if 0 bytes read), ESP_FAIL on I2C error.
     */
    esp_err_t lc76g_read_data(uint8_t *buffer, size_t max_len, size_t *read_len);

#ifdef __cplusplus
}
#endif

#endif // LC76G_H
