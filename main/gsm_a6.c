#include "gsm_a6.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "GSM_A6";
#define UART_BUF_SIZE (1024)

static esp_err_t gsm_power_on(void)
{
    ESP_LOGI(TAG, "Starting GSM A6 Power Sequence...");

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GSM_PWR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Provide a pulse to trigger the module to power on
    // SIM800L module requires a high pulse on PWR for > 2 seconds to turn on/off
    gpio_set_level(GSM_PWR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(2500)); // 2.5 seconds
    gpio_set_level(GSM_PWR_PIN, 0);

    gsm_update_ui_status("PWR ON Pls Wait");
    ESP_LOGI(TAG, "Waiting 2s for module base boot...");

    for (int i = 2; i >= 1; i--)
    {
        char msg[32];
        snprintf(msg, sizeof(msg), "Boot Wait: %ds", i);
        gsm_update_ui_status(msg);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return ESP_OK;
}

esp_err_t gsm_a6_init(void)
{
    // We now have stable 3.7V direct battery power for SIM800L, restrict testing strictly to 115200 baud
    const int TEST_BAUDS[] = {115200};
    int num_bauds = sizeof(TEST_BAUDS) / sizeof(TEST_BAUDS[0]);

    // Test straight connection first, then crossed
    const gpio_num_t TEST_TX[] = {GSM_TXD_PIN, GSM_RXD_PIN};
    const gpio_num_t TEST_RX[] = {GSM_RXD_PIN, GSM_TXD_PIN};

    // Removed the SD pin forcing. We are now on pure GPIO 16/17 which do not need artificial pull-ups.

    // Give logic analyzer time to settle
    vTaskDelay(pdMS_TO_TICKS(100));

    static bool uart_installed = false;
    if (!uart_installed)
    {
        // Install UART driver once
        // Using default UART parameters (8N1) for data bits, parity, stop bits.
        ESP_ERROR_CHECK_WITHOUT_ABORT(uart_driver_install(GSM_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
        uart_installed = true;
    }

    // Always do a power cycle to start fresh
    gsm_power_on();

    for (int pin_cfg = 0; pin_cfg < 2; pin_cfg++)
    {
        gpio_num_t current_tx = TEST_TX[pin_cfg];
        gpio_num_t current_rx = TEST_RX[pin_cfg];

        ESP_LOGI(TAG, ">>> Testing Pin Configuration %d: TX=IO%d, RX=IO%d <<<", pin_cfg + 1, current_tx, current_rx);
        uart_set_pin(GSM_UART_NUM, current_tx, current_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

        for (int baud_idx = 0; baud_idx < num_bauds; baud_idx++)
        {
            int current_baud = TEST_BAUDS[baud_idx];
            ESP_LOGI(TAG, "  Syncing at Baud Rate: %d...", current_baud);

            char ui_msg[32];
            snprintf(ui_msg, sizeof(ui_msg), "Sync %d", current_baud);
            gsm_update_ui_status(ui_msg);

            uart_set_baudrate(GSM_UART_NUM, current_baud);
            uart_flush_input(GSM_UART_NUM);

            // Aggressive Blind Sync: Send AT many times to force auto-baud lock
            for (int j = 0; j < 5; j++)
            {
                uart_write_bytes(GSM_UART_NUM, "AT\r\n", 4);
                vTaskDelay(pdMS_TO_TICKS(400));
            }

            // Now check for actual response with longer timeout
            for (int i = 0; i < 12; i++)
            {
                snprintf(ui_msg, sizeof(ui_msg), "B%d (%d/12)", current_baud, i + 1);
                gsm_update_ui_status(ui_msg);

                if (gsm_send_at_cmd("AT\r\n", "OK", 2000) == ESP_OK)
                {
                    ESP_LOGI(TAG, ">>> SUCCESS! Module responding at %d baud on TX=IO%d/RX=IO%d <<<",
                             current_baud, current_tx, current_rx);

                    gsm_update_ui_status("Baud OK!");
                    gsm_send_at_cmd("ATE0\r\n", "OK", 1500); // Echo Off

                    // Send initialization commands
                    ESP_LOGI(TAG, "Configuring GSM Module for Network Access...");
                    gsm_send_at_cmd("AT+CFUN=1\r\n", "OK", 5000); // Full functionality
                    gsm_send_at_cmd("AT+CMEE=2\r\n", "OK", 1500); // Verbose error reporting

                    // Try manual registration to mt:s (MCC=220, MNC=03)
                    // If this fails, 2G might be unavailable for this SIM/operator
                    ESP_LOGW(TAG, "Attempting MANUAL registration to mt:s (22003)...");
                    gsm_update_ui_status("Reg mts...");
                    if (gsm_send_at_cmd("AT+COPS=1,2,\"22003\"\r\n", "OK", 30000) != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Manual registration to mt:s FAILED!");
                        ESP_LOGW(TAG, "Trying Yettel (22001)...");
                        gsm_update_ui_status("Reg Yettel...");
                        if (gsm_send_at_cmd("AT+COPS=1,2,\"22001\"\r\n", "OK", 30000) != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Manual registration to Yettel FAILED!");
                            ESP_LOGW(TAG, "Trying A1 (22005)...");
                            gsm_update_ui_status("Reg A1...");
                            if (gsm_send_at_cmd("AT+COPS=1,2,\"22005\"\r\n", "OK", 30000) != ESP_OK)
                            {
                                ESP_LOGE(TAG, "ALL manual registrations FAILED! 2G may not be available.");
                                gsm_update_ui_status("No 2G Net!");
                            }
                        }
                    }

                    // Fall back to automatic
                    gsm_send_at_cmd("AT+COPS=0\r\n", "OK", 5000);

                    return ESP_OK;
                }
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }

    ESP_LOGE(TAG, "GSM Module failed to respond to ALL configurations (pin swaps and bauds).");
    return ESP_FAIL;
}

esp_err_t gsm_send_at_cmd(const char *cmd, const char *expected_response, uint32_t timeout_ms)
{
    // Clear UART RX buffer to avoid reading old data
    uart_flush_input(GSM_UART_NUM);

    // Send command
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));
    // ESP_LOGD(TAG, "Sent: %s", cmd);

    uint8_t data[UART_BUF_SIZE];
    int length = 0;
    uint32_t elapsed_time = 0;
    const uint32_t step_ms = 100;

    // Buffer for accumulation
    char rx_buffer[UART_BUF_SIZE * 2] = {0}; // Increased buffer size
    int rx_index = 0;

    while (elapsed_time < timeout_ms)
    {
        length = uart_read_bytes(GSM_UART_NUM, data, UART_BUF_SIZE - 1, pdMS_TO_TICKS(step_ms));
        if (length > 0)
        {
            data[length] = '\0';

            // Wait for full response lines, some modules send "AT\r\n\r\nOK\r\n"
            if ((rx_index + length) < (sizeof(rx_buffer) - 1))
            {
                strncat(rx_buffer, (char *)data, length);
                rx_index += length;
            }

            if (strstr(rx_buffer, expected_response) != NULL)
            {
                ESP_LOGI(TAG, "Matched '%s' in: %s", expected_response, rx_buffer);
                return ESP_OK;
            }
            if (strstr(rx_buffer, "ERROR") != NULL)
            {
                ESP_LOGE(TAG, "Received ERROR for command: %s. Raw RX: %s", cmd, rx_buffer);
                return ESP_FAIL;
            }
        }
        elapsed_time += step_ms;
    }

    // Convert raw rx to hex string for deep debugging
    char hex_str[128] = {0};
    for (int i = 0; i < (rx_index < 30 ? rx_index : 30); i++)
    {
        sprintf(hex_str + strlen(hex_str), "%02X ", (uint8_t)rx_buffer[i]);
    }

    ESP_LOGW(TAG, "Timeout waiting for expected response. Command sent. Raw RX: %s | HEX: %s", rx_buffer, hex_str);
    return ESP_FAIL;
}

esp_err_t gsm_check_network(void)
{
    // Send AT+CREG? to check network registration
    // CREG second digit meanings:
    //   0 = Not registered, not searching
    //   1 = Registered, home network
    //   2 = Not registered, searching
    //   3 = Registration denied
    //   5 = Registered, roaming

    uart_flush_input(GSM_UART_NUM);
    uart_write_bytes(GSM_UART_NUM, "AT+CREG?\r", 9);

    uint8_t data[UART_BUF_SIZE];
    int length = 0;
    char rx_buffer[UART_BUF_SIZE] = {0};

    // Simplistic read loop for 2 seconds max
    for (int i = 0; i < 20; i++)
    {
        length = uart_read_bytes(GSM_UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        if (length > 0)
        {
            data[length] = '\0';
            strncat(rx_buffer, (char *)data, length);

            if (strstr(rx_buffer, "OK") != NULL)
            {
                if (strstr(rx_buffer, ",1") != NULL || strstr(rx_buffer, ",5") != NULL)
                {
                    ESP_LOGI(TAG, "Network registration successful! (Status: %s)", rx_buffer);
                    return ESP_OK;
                }
                else
                {
                    // Detect specific failure states
                    bool gave_up = (strstr(rx_buffer, ",0") != NULL); // Not searching
                    bool denied = (strstr(rx_buffer, ",3") != NULL);  // Registration denied

                    if (gave_up)
                    {
                        ESP_LOGE(TAG, "Module STOPPED searching (CREG ,0). Forcing re-registration...");
                        gsm_update_ui_status("GSM: Re-Reg...");
                        // Force automatic operator selection to restart search
                        gsm_send_at_cmd("AT+COPS=0\r\n", "OK", 10000);
                    }
                    else if (denied)
                    {
                        ESP_LOGE(TAG, "Network DENIED registration (CREG ,3)!");
                        gsm_update_ui_status("GSM: DENIED");
                        // Reset radio to retry
                        gsm_send_at_cmd("AT+CFUN=0\r\n", "OK", 5000);
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        gsm_send_at_cmd("AT+CFUN=1\r\n", "OK", 5000);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Network not registered yet. CREG status: %s", rx_buffer);
                    }

                    // Diagnostics
                    ESP_LOGI(TAG, "Querying Signal Quality (CSQ)...");
                    gsm_send_at_cmd("AT+CSQ\r\n", "OK", 2000);

                    ESP_LOGI(TAG, "Querying Current Operator (COPS)...");
                    gsm_send_at_cmd("AT+COPS?\r\n", "OK", 5000);

                    ESP_LOGI(TAG, "Querying SIM Status (CPIN)...");
                    gsm_send_at_cmd("AT+CPIN?\r\n", "OK", 2000);

                    ESP_LOGI(TAG, "Querying IMEI...");
                    gsm_send_at_cmd("AT+GSN\r\n", "OK", 2000);

                    ESP_LOGI(TAG, "Querying SIM CCID...");
                    gsm_send_at_cmd("AT+CCID\r\n", "OK", 2000);

                    return ESP_FAIL;
                }
            }
        }
    }
    ESP_LOGE(TAG, "Network check timeout.");
    return ESP_FAIL;
}

esp_err_t gsm_send_sms(const char *phone_number, const char *message)
{
    if (phone_number == NULL || message == NULL)
        return ESP_FAIL;

    ESP_LOGI(TAG, "Attempting to send SMS to %s...", phone_number);

    // Set SMS format to text mode
    if (gsm_send_at_cmd("AT+CMGF=1\r", "OK", 2000) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set SMS text mode.");
        return ESP_FAIL;
    }

    // Command to send SMS: AT+CMGS="phoneNumber"
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"\r", phone_number);

    if (gsm_send_at_cmd(cmd, ">", 2000) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initiate SMS send.");
        return ESP_FAIL;
    }

    // Send the message body followed by CTRL+Z (ASCII 26)
    uart_write_bytes(GSM_UART_NUM, message, strlen(message));
    char ctrl_z = 26;
    uart_write_bytes(GSM_UART_NUM, &ctrl_z, 1);

    // Wait for SMS sent confirmation (+CMGS: <index>)
    // This can take a while depending on network, allowing 15 seconds.
    if (gsm_send_at_cmd("", "+CMGS:", 15000) == ESP_OK)
    {
        ESP_LOGI(TAG, "SMS successfully sent.");
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "SMS failed to send or timed out.");
        return ESP_FAIL;
    }
}

typedef struct
{
    char phone[32];
    char msg[256];
} sms_task_params_t;

static void send_sms_task(void *arg)
{
    sms_task_params_t *params = (sms_task_params_t *)arg;
    gsm_send_sms(params->phone, params->msg);
    free(params);
    vTaskDelete(NULL);
}

void gsm_send_sms_async(const char *phone_number, const char *message)
{
    if (phone_number == NULL || message == NULL)
        return;
    sms_task_params_t *params = (sms_task_params_t *)malloc(sizeof(sms_task_params_t));
    if (params)
    {
        strncpy(params->phone, phone_number, sizeof(params->phone) - 1);
        params->phone[sizeof(params->phone) - 1] = '\0';
        strncpy(params->msg, message, sizeof(params->msg) - 1);
        params->msg[sizeof(params->msg) - 1] = '\0';
        xTaskCreate(send_sms_task, "sms_task", 6144, params, 5, NULL);
    }
}
