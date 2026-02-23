#ifndef GSM_A6_H
#define GSM_A6_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/uart.h"
#include "esp_err.h"

// Define the GPIO pins. We will use UART1 for the GSM to avoid UART0 console conflict completely,
// but map the pins to the ones the user specified on the board (RXD/TXD silkscreen usually maps to GPIO43/44, but let's provide macros in case they mean actual UART0 pins).
// From wavehsare wiki: TXD=43, RXD=44 on standard ESP32-S3 boards.
#define GSM_UART_NUM UART_NUM_1   // Using UART1 for GSM
#define GSM_TXD_PIN (GPIO_NUM_16) // ESP GPIO 16 -> Modul URX
#define GSM_RXD_PIN (GPIO_NUM_17) // ESP GPIO 17 -> Modul UTX
#define GSM_PWR_PIN (GPIO_NUM_18) // Power toggle pin

    /**
     * @brief Initialize the UART interface and the GPIO power pin for the GSM module.
     *        This will also trigger the boot sequence (toggling PWR pin).
     * @return ESP_OK on success
     */
    esp_err_t gsm_a6_init(void);

    /**
     * @brief Send an AT command to the GSM module and wait for a response.
     * @param cmd The AT command to send (e.g., "AT\r\n").
     * @param expected_response The expected substring in the response (e.g., "OK").
     * @param timeout_ms How long to wait for the response.
     * @return ESP_OK if the expected response was received, ESP_FAIL otherwise.
     */
    esp_err_t gsm_send_at_cmd(const char *cmd, const char *expected_response, uint32_t timeout_ms);

    /**
     * @brief Check if the module is registered to the cellular network.
     * @return ESP_OK if registered, ESP_FAIL otherwise.
     */
    esp_err_t gsm_check_network(void);

    /**
     * @brief Hook for updating the UI with GSM connection status.
     * Implemented in lifelink.cpp.
     */
    void gsm_update_ui_status(const char *text);

    /**
     * @brief Send an SMS message to a specific number.
     * @param phone_number The recipient's phone number as a string (e.g., "+381641234567").
     * @param message The text message to send.
     * @return ESP_OK on success, ESP_FAIL on failure.
     */
    esp_err_t gsm_send_sms(const char *phone_number, const char *message);
    void gsm_send_sms_async(const char *phone_number, const char *message);

#ifdef __cplusplus
}
#endif

#endif // GSM_A6_H
