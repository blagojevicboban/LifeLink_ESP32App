#include <stdio.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "SensorLib.h"
#include "TouchDrvCST92xx.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lcd_sh8601.h"
#include "ui/ui.h"
#include "SensorQMI8658.hpp" // Ensure this path is correct
#include "ble_spp_server.h"
#include "max30102.h"
#include "max30102.h"
#include "algorithm.h"
#include "lc76g.h"
#include "fft_algo.h"
#include "axp2101.h"
#include "gsm_a6.h"
#define SCREEN_TIMEOUT_MS 15000 // 15 seconds timeout

extern "C" void start_fall_countdown_ui(bool is_simulated);

// SensorQMI8658 ACCELEROMETER BEGIN
// I2C configuration
#define I2C_MASTER_SCL 14
#define I2C_MASTER_SDA 15
#define I2C_MASTER_NUM I2C_NUM_0
#define QMI8658_ADDRESS 0x6B // Replace with your QMI8658 address
// --- Konstante za detekciju pada (Advanced) ---
#define FALL_THRESHOLD_LOW 0.6f    // Free fall threshold (<0.6G)
#define FALL_THRESHOLD_HIGH 3.5f   // Impact threshold (>3.5G)
#define STILLNESS_TOLERANCE 0.2f   // Tolerance for 1G stillness
#define ANGLE_THRESHOLD_DEG 60.0f  // Orientation change threshold
#define STILLNESS_DURATION_MS 5000 // Duration to confirm stillness

enum FallDetectionState
{
    IDLE,
    FREE_FALL,
    IMPACT_DETECTED,
    WAITING_FOR_STILLNESS
};
FallDetectionState fallState = IDLE;
unsigned long stateTimer = 0;
int fallCount = 0;
int potentialFallCount = 0;

// Variables for Orientation Check
float ref_ax = 0, ref_ay = 0, ref_az = 0;    // Pre-fall orientation
float curr_ax = 0, curr_ay = 0, curr_az = 0; // Current orientation

SensorQMI8658 qmi;
IMUdata acc;
IMUdata gyr;
MAX30102 max30102;

static const char *TAGA = "QMI8658"; // Define a tag for logging

// I2C configuration consolidated below
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_SDA_IO (gpio_num_t) I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO (gpio_num_t) I2C_MASTER_SCL

// i2c_master_init removed - using shared i2c_init instead

// Variables for MAX30102 algorithm (Moved to Top for Scope Visibility)
#define MAX_BRIGHTNESS 255
#define TEST_BUFFER_LENGTH 512 // Power of 2 for FFT

uint32_t irBuffer[TEST_BUFFER_LENGTH];
uint32_t redBuffer[TEST_BUFFER_LENGTH];

// Float buffers for FFT (Global to save stack)
float irBufferFloat[TEST_BUFFER_LENGTH];
float redBufferFloat[TEST_BUFFER_LENGTH];

int32_t bufferLength = TEST_BUFFER_LENGTH;
int32_t spo2 = 0;
int8_t validSPO2 = 0;
int32_t heartRate = 0;
int8_t validHeartRate = 0;
float fft_hr = 0;
float fft_spo2 = 0;

// --- GPS Globals ---
float g_latitude = 0.0f;
float g_longitude = 0.0f;

// Forward Declarations
static bool example_lvgl_lock(int timeout_ms);
static void example_lvgl_unlock(void);

// Helper: Convert NMEA scalar (DDDMM.MMMM) to Decimal Degrees
float nmea_to_decimal(float nmea_val)
{
    int degrees = (int)(nmea_val / 100);
    float minutes = nmea_val - (degrees * 100);
    return degrees + (minutes / 60.0f);
}

// Simple NMEA Parser (Handles $GNRMC)
void parse_nmea(char *line)
{
    if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0)
    {
        // Example: $GNRMC,123519.000,A,4807.038,N,01131.000,E,...
        // We will simple-scan for simplicity. For robust parsing, use minmea or similar.
        // But sscanf is often enough if format is standard.

        char type[10];
        char time[15];
        char status; // for RMC
        float raw_lat, raw_lon;
        char ns, ew;

        // Try RMC first
        if (strncmp(line, "$GNRMC", 6) == 0)
        {
            // $GNRMC,time,status,lat,NS,lon,EW,spd,cog,date,...
            // Status: A=Active, V=Void
            if (sscanf(line, "%[^,],%[^,],%c,%f,%c,%f,%c", type, time, &status, &raw_lat, &ns, &raw_lon, &ew) >= 7)
            {
                if (status == 'A')
                {
                    float lat = nmea_to_decimal(raw_lat);
                    if (ns == 'S')
                        lat = -lat;
                    float lon = nmea_to_decimal(raw_lon);
                    if (ew == 'W')
                        lon = -lon;

                    g_latitude = lat;
                    g_longitude = lon;
                    ESP_LOGI("GPS_PARSED", "Lat: %.5f, Lon: %.5f", g_latitude, g_longitude);

                    // Update GPS Status Logic (Green)
                    if (example_lvgl_lock(-1))
                    {
                        lv_obj_set_style_text_color(ui_LabelGPS, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);

                        // Update Time (Time format: HHMMSS.XX)
                        // time[0-1]=HH, time[2-3]=MM
                        char clock_str[6];
                        if (strlen(time) >= 4)
                        {
                            snprintf(clock_str, sizeof(clock_str), "%c%c:%c%c", time[0], time[1], time[2], time[3]);
                            if (ui_LabelTime)
                                lv_label_set_text(ui_LabelTime, clock_str);
                        }
                        example_lvgl_unlock();
                    }
                }
                else
                {
                    // Update GPS Status Logic (Red - No Fix)
                    if (example_lvgl_lock(-1))
                    {
                        lv_obj_set_style_text_color(ui_LabelGPS, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
                        example_lvgl_unlock();
                    }
                }
            }
        }
        // Try GGA (if RMC fails or we want alt) - purely as fallback for coords
        else if (strncmp(line, "$GNGGA", 6) == 0)
        {
            // $GNGGA,time,lat,NS,lon,EW,fix,...
            int fix;
            if (sscanf(line, "%[^,],%[^,],%f,%c,%f,%c,%d", type, time, &raw_lat, &ns, &raw_lon, &ew, &fix) >= 7)
            {
                if (fix > 0)
                {
                    float lat = nmea_to_decimal(raw_lat);
                    if (ns == 'S')
                        lat = -lat;
                    float lon = nmea_to_decimal(raw_lon);
                    if (ew == 'W')
                        lon = -lon;

                    g_latitude = lat;
                    g_longitude = lon;
                    // ESP_LOGI("GPS_PARSED", "GGA Lat: %.5f, Lon: %.5f", g_latitude, g_longitude);
                }
            }
        }
    }
}

void read_sensor_data(void *arg); // Function declaration

void setup_accel()
{
    // i2c_master_init(); // Removed: I2C is already initialized by i2c_init() in app_main

    // Initialize QMI8658 sensor with 4 parameters (port number, address, SDA, SCL)
    if (!qmi.begin(I2C_MASTER_NUM, QMI8658_ADDRESS, I2C_MASTER_SDA, I2C_MASTER_SCL))
    {
        ESP_LOGE(TAGA, "Failed to find QMI8658 - check your wiring!");
        vTaskDelete(NULL); // Handle error gracefully
    }

    // Get chip ID
    ESP_LOGI(TAGA, "Device ID: %x", qmi.getChipID());

    // Configure accelerometer
    qmi.configAccelerometer(
        SensorQMI8658::ACC_RANGE_4G,
        SensorQMI8658::ACC_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_0,
        true);

    // Configure gyroscope
    qmi.configGyroscope(
        SensorQMI8658::GYR_RANGE_64DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_3,
        true);

    // Enable gyroscope and accelerometer
    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    ESP_LOGI(TAGA, "Ready to read data...");
}

void setup_max30102()
{
    if (!max30102.begin(I2C_MASTER_NUM))
    {
        ESP_LOGE("MAX30102", "MAX30102 not found");
    }
    else
    {
        ESP_LOGI("MAX30102", "MAX30102 initialized");
        // Power=0x1F (Low/Medium ~6.4mA), Avg=4, Mode=2(Red+IR), Rate=400Hz, Width=411, Range=4096
        // Power=0x1F (Low/Medium ~6.4mA), Avg=4, Mode=2(Red+IR), Rate=400Hz, Width=411, Range=4096
        max30102.setup(0x1F, 4, 2, 400, 411, 4096);
        fft_init(TEST_BUFFER_LENGTH);
    }
}
// SensorQMI8658 ACCELEROMETER END

static const char *TAG = "example";
static SemaphoreHandle_t lvgl_mux = NULL;
esp_lcd_panel_handle_t panel_handle = NULL; // Moved to global for power management

#define LCD_HOST SPI2_HOST
#define TOUCH_HOST I2C_NUM_0

#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL (16)
#endif

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_LCD_CS (GPIO_NUM_12)
#define EXAMPLE_PIN_NUM_LCD_PCLK (GPIO_NUM_38)
#define EXAMPLE_PIN_NUM_LCD_DATA0 (GPIO_NUM_4)
#define EXAMPLE_PIN_NUM_LCD_DATA1 (GPIO_NUM_5)
#define EXAMPLE_PIN_NUM_LCD_DATA2 (GPIO_NUM_6)
#define EXAMPLE_PIN_NUM_LCD_DATA3 (GPIO_NUM_7)
#define EXAMPLE_PIN_NUM_LCD_RST (GPIO_NUM_39)
#define EXAMPLE_PIN_NUM_BK_LIGHT (-1)

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 466
#define EXAMPLE_LCD_V_RES 466

#define EXAMPLE_USE_TOUCH 1

#if EXAMPLE_USE_TOUCH
#define EXAMPLE_PIN_NUM_TOUCH_SCL (GPIO_NUM_14)
#define EXAMPLE_PIN_NUM_TOUCH_SDA (GPIO_NUM_15)
#define EXAMPLE_PIN_NUM_TOUCH_RST (GPIO_NUM_40)
#define EXAMPLE_PIN_NUM_TOUCH_INT (GPIO_NUM_11)

// Consolidated definitions at the top
// #define I2C_MASTER_NUM (i2c_port_t)1
// #define I2C_MASTER_FREQ_HZ 100000
// #define I2C_MASTER_SDA_IO (gpio_num_t)15
// #define I2C_MASTER_SCL_IO (gpio_num_t)14
#define Touch_INT (gpio_num_t)11
#define Touch_RST (gpio_num_t)40

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000
uint8_t touchAddress = 0x5A;

TouchDrvCST92xx touch;
int16_t x[5], y[5];
bool isPressed = false;

#endif

#define EXAMPLE_LVGL_BUF_HEIGHT (EXAMPLE_LCD_V_RES / 4)
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY 2

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 0},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
    {0x63, (uint8_t[]){0xFF}, 1, 0},
    {0x2A, (uint8_t[]){0x00, 0x06, 0x01, 0xD7}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xD1}, 4, 600},
    {0x11, NULL, 0, 600}, // 命令后延时 600ms
    {0x29, NULL, 0, 0},   // 无延时
};

esp_err_t i2c_init(void)
{
    i2c_config_t i2c_conf;
    memset(&i2c_conf, 0, sizeof(i2c_conf));
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = I2C_MASTER_SDA_IO;
    i2c_conf.scl_io_num = I2C_MASTER_SCL_IO;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    return i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void read_sensor_data(void *arg); // Function declaration
void gps_task(void *arg);
void gsm_status_task(void *arg);

void setup_sensor()
{
    uint8_t touchAddress = 0x5A;

    touch.setPins(Touch_RST, Touch_INT);
    touch.begin(I2C_MASTER_NUM, touchAddress, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    touch.reset();
    touch.setMaxCoordinates(466, 466);
    touch.setMirrorXY(true, true);
}

// --- Screen Timeout Variables ---
static uint64_t last_touch_time = 0;
static bool screen_is_on = true;

void reset_screen_timer()
{
    last_touch_time = esp_timer_get_time() / 1000;
    if (!screen_is_on)
    {
        screen_is_on = true;
        // Wake up LCD (AMOLED uses commands, not just backlight)
        if (panel_handle)
        {
            esp_lcd_panel_disp_on_off(panel_handle, true);
        }
        // Turn on backlight (if exists)
#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
        gpio_set_level((gpio_num_t)EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif
        ESP_LOGI("PWR", "Screen WAKE");
    }
}

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    const int offsetx1 = area->x1; //+ 0x16;
    const int offsetx2 = area->x2; //+ 0x16;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

#if LCD_BIT_PER_PIXEL == 24
    uint8_t *to = (uint8_t *)color_map;
    uint8_t temp = 0;
    uint16_t pixel_num = (offsetx2 - offsetx1 + 1) * (offsety2 - offsety1 + 1);

    // Special dealing for first pixel
    temp = color_map[0].ch.blue;
    *to++ = color_map[0].ch.red;
    *to++ = color_map[0].ch.green;
    *to++ = temp;
    // Normal dealing for other pixels
    for (int i = 1; i < pixel_num; i++)
    {
        *to++ = color_map[i].ch.red;
        *to++ = color_map[i].ch.green;
        *to++ = color_map[i].ch.blue;
    }
#endif

    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void example_lvgl_update_cb(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;

    switch (drv->rotated)
    {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;

    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    // round the start of coordinate down to the nearest 2M number
    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;
    // round the end of coordinate up to the nearest 2N+1 number
    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}

#if EXAMPLE_USE_TOUCH
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint8_t touched = touch.getPoint(x, y, 2);
    if (touched)
    {

        for (int i = 0; i < 1; ++i)
        {
            data->point.x = x[0];
            data->point.y = y[0];
            data->state = LV_INDEV_STATE_PRESSED;
            ESP_LOGI(TAG, "Touch[%d]: X=%d Y=%d", i, x[i], y[i]);

            // Reset Timeout on Touch
            reset_screen_timer();
        }
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static bool example_lvgl_lock(int timeout_ms)
{
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}

extern "C" void gsm_update_ui_status(const char *text)
{
    if (lvgl_mux != NULL && example_lvgl_lock(-1))
    {
        if (ui_LabelGSM)
        {
            lv_label_set_text(ui_LabelGSM, text);
        }
        if (ui_LabelGSM_Icon)
        {
            lv_obj_set_style_text_color(ui_LabelGSM_Icon, lv_color_hex(0xFFA500), LV_PART_MAIN); // Orange indicating connecting/processing
            lv_obj_set_style_text_color(ui_LabelGSM_Text, lv_color_hex(0xFFA500), LV_PART_MAIN);
        }
        example_lvgl_unlock();
    }
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1))
        {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

extern "C" void app_main(void)
{
    // Initialize BLE first to ensure resources are available
    ble_spp_server_init();

    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = GPIO_NUM_38;
    buscfg.data0_io_num = GPIO_NUM_4;
    buscfg.data1_io_num = GPIO_NUM_5;
    buscfg.data2_io_num = GPIO_NUM_6;
    buscfg.data3_io_num = GPIO_NUM_7;
    buscfg.max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(uint16_t);
    buscfg.flags = SPICOMMON_BUSFLAG_QUAD;
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS,
                                                                                example_notify_lvgl_flush_ready,
                                                                                &disp_drv);
    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // esp_lcd_panel_handle_t panel_handle = NULL; // Removed local declaration
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_LOGI(TAG, "Install SH8601 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if EXAMPLE_USE_TOUCH

    ESP_ERROR_CHECK(i2c_init());

    setup_sensor();
    setup_max30102();

#endif

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
    reset_screen_timer(); // Initialize timer
#endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = static_cast<lv_color_t *>(heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_SPIRAM));
    assert(buf1);
    lv_color_t *buf2 = static_cast<lv_color_t *>(heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_SPIRAM));
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.rounder_cb = example_lvgl_rounder_cb;
    disp_drv.drv_update_cb = example_lvgl_update_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#if EXAMPLE_USE_TOUCH
    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = &touch;
    lv_indev_drv_register(&indev_drv);
#endif

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);

    // --- Initialize PMU and GSM AFTER LVGL Mutex is created ---
#if EXAMPLE_USE_TOUCH
    if (axp_init(I2C_MASTER_NUM) == ESP_OK)
    {
        ESP_LOGI("PMU", "AXP2101 Initialized");
    }

    // Spawn a task for GSM Init so it runs continuously in the background
    xTaskCreate([](void *arg)
                {
        while(1) 
        {
            if (gsm_a6_init() == ESP_OK)
            {
                ESP_LOGI("MAIN", "GSM A6 Module Initialized Successfully.");
                if (example_lvgl_lock(-1))
                {
                    if (ui_LabelGSM) {
                        lv_label_set_text(ui_LabelGSM, "GSM: OK!");
                        lv_obj_set_style_text_color(ui_LabelGSM, lv_color_hex(0x00FF00), LV_PART_MAIN);
                    }
                    if (ui_LabelGSM_Icon) {
                        lv_obj_set_style_text_color(ui_LabelGSM_Icon, lv_color_hex(0x00FF00), LV_PART_MAIN);
                        lv_obj_set_style_text_color(ui_LabelGSM_Text, lv_color_hex(0x00FF00), LV_PART_MAIN);
                    }
                    example_lvgl_unlock();
                }
                xTaskCreate(gsm_status_task, "gsm_status_task", 8192, NULL, 5, NULL);
                break; // Exit loop on connection success
            }
            else
            {
                ESP_LOGE("MAIN", "GSM Init Failed. Retrying in 3 seconds...");
                if (example_lvgl_lock(-1))
                {
                    if (ui_LabelGSM) {
                        lv_label_set_text(ui_LabelGSM, "GSM: RETRYING...");
                        lv_obj_set_style_text_color(ui_LabelGSM, lv_color_hex(0xFF0000), LV_PART_MAIN);
                    }
                    if (ui_LabelGSM_Icon) {
                        lv_obj_set_style_text_color(ui_LabelGSM_Icon, lv_color_hex(0xFF0000), LV_PART_MAIN);
                        lv_obj_set_style_text_color(ui_LabelGSM_Text, lv_color_hex(0xFF0000), LV_PART_MAIN);
                    }
                    example_lvgl_unlock();
                }
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
        }
        vTaskDelete(NULL); }, "gsm_init_task", 8192, NULL, 5, NULL);

#endif

    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL demos");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1))
    {

        // lv_demo_widgets();      /* A widgets example */
        // lv_demo_music(); /* A modern, smartphone-like music player demo. */
        // lv_demo_stress();       /* A stress test for LVGL. */
        // lv_demo_benchmark();    /* A demo to measure the performance of LVGL or to compare different settings. */
        ui_init(); // LifeLink UI initialization

        // Initial values
        lv_label_set_text(ui_LabelTime, "12:00");
        lv_label_set_text(ui_LabelInfo, "100%");

        // Setup Sensor & Tasks immediately for responsive UI
        setup_accel();
        xTaskCreate(read_sensor_data, "sensor_read_task", 4096, NULL, 10, NULL);
        xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);

        // --- Power Management Task (or just add to loop if lightweight) ---
        // For simplicity, we can do it in read_sensor_data or a timer.
        // Let's rely on read_sensor_data for battery updates and main loop/timer for timeout.

        // Release the mutex
        example_lvgl_unlock();
    }
}

// Variables for MAX30102 algorithm
void read_sensor_data(void *arg)
{
    char x_str[20], y_str[20], z_str[20], g_str[20], gx_str[20], gy_str[20], gz_str[20];
    char info_str[64];
    float g_total = 1.0f; // Default to 1G if not ready
    float gyro_total = 0.0f;
    static int g_batt_pct = 0; // Static to persist for Heartbeat

    // Initialize labels
    // snprintf(info_str, sizeof(info_str), "Nadzor (Pot:%d, Pad:%d)", potentialFallCount, fallCount);
    // lv_label_set_text(ui_LabelInfo, info_str);

    int samplesCollected = 0;

    while (1)
    {
        // --- PEFORM SENSOR READS (NON-BLOCKING) ---

        // 1. MAX30102 Polling & Buffering
        if (screen_is_on)
        {
            max30102.check();
            while (max30102.available())
            {
                if (samplesCollected < TEST_BUFFER_LENGTH)
                {
                    redBuffer[samplesCollected] = max30102.getRed();
                    irBuffer[samplesCollected] = max30102.getIR();
                    max30102.nextSample();
                    samplesCollected++;

                    // Algorithm: Run every time we fill the buffer (sliding window)
                    if (samplesCollected == TEST_BUFFER_LENGTH)
                    {
                        // Convert to float for FFT
                        for (int i = 0; i < TEST_BUFFER_LENGTH; i++)
                        {
                            redBufferFloat[i] = (float)redBuffer[i];
                            irBufferFloat[i] = (float)irBuffer[i];
                        }

                        // Run FFT Algorithm (100Hz sampling rate)
                        fft_process(redBufferFloat, irBufferFloat, TEST_BUFFER_LENGTH, 100, &fft_hr, &fft_spo2);

                        // Update global variables
                        heartRate = (int32_t)fft_hr;
                        spo2 = (int32_t)fft_spo2;
                        validHeartRate = (heartRate > 0);
                        validSPO2 = (spo2 > 0);

                        // ESP_LOGI("MAX30102", "HR: %ld, SpO2: %ld, Valid: %d/%d", heartRate, spo2, validHeartRate, validSPO2);

                        // Shift buffer left by 50 samples (0.5s slide)
                        int shift = 50;
                        for (int i = shift; i < TEST_BUFFER_LENGTH; i++)
                        {
                            redBuffer[i - shift] = redBuffer[i];
                            irBuffer[i - shift] = irBuffer[i];
                        }
                        samplesCollected = TEST_BUFFER_LENGTH - shift;
                    }
                }
                else
                {
                    // Safety: drain if overflow logic fails
                    max30102.nextSample();
                }
            }
        }
        else
        {
            // Reset buffer logic when screen is off to prevent old data upon waking
            samplesCollected = 0;
            max30102.clearFIFO(); // Prevent internal sensor buffer overflow
        }

        // 2. QMI8658 Polling
        bool qmi_updated = false;
        if (qmi.getDataReady())
        {
            if (qmi.getAccelerometer(acc.x, acc.y, acc.z) && qmi.getGyroscope(gyr.x, gyr.y, gyr.z))
            {
                g_total = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
                gyro_total = sqrt(gyr.x * gyr.x + gyr.y * gyr.y + gyr.z * gyr.z);
                qmi_updated = true;
            }
        }

        // --- UPDATE UI & LOGIC ---
        if (example_lvgl_lock(-1))
        {
            // Calculate avg raw value for debugging
            uint32_t avgRed = 0;
            uint32_t avgIR = 0;
            if (bufferLength > 25)
            {
                for (int k = bufferLength - 25; k < bufferLength; k++)
                {
                    avgRed += redBuffer[k];
                    avgIR += irBuffer[k];
                }
                avgRed /= 25;
                avgIR /= 25;
            }

            // Finger Detection Threshold
            // With 0x7F power, valid finger signal should be > 100,000.
            // Ambient noise is usually < 20,000.
            if (avgIR < 50000)
            {
                heartRate = 0;
                spo2 = 0;
                validHeartRate = 0;
                validSPO2 = 0;
            }

            // Always update Pulse/SpO2 if valid
            char puls_str[16] = "--";
            char spo_str[16] = "--";
            if (validHeartRate && heartRate > 30 && heartRate < 220)
                snprintf(puls_str, sizeof(puls_str), "%ld", heartRate);
            if (validSPO2 && spo2 > 50 && spo2 <= 100)
                snprintf(spo_str, sizeof(spo_str), "%ld", spo2);

            // Rate limiting for Log and UI
            static uint64_t last_report_time = 0;
            bool report_now = (esp_timer_get_time() / 1000 - last_report_time) > 100;

            if (report_now)
            {
                last_report_time = esp_timer_get_time() / 1000;

                // Only update labels if we have a valid reading or strict "--"
                lv_label_set_text(ui_LabelPuls, puls_str);
                lv_label_set_text(ui_LabelSpo, spo_str);

                static uint64_t last_sensor_log = 0;
                if ((esp_timer_get_time() / 1000 - last_sensor_log) > 3000)
                {
                    last_sensor_log = esp_timer_get_time() / 1000;
                    // Log Raw values to debug saturation (Max is ~262143 for 18-bit)
                    ESP_LOGI("MAX30102", "HR: %ld, SpO2: %ld, Val: %d/%d, RawRed: %lu, RawIR: %lu",
                             heartRate, spo2, validHeartRate, validSPO2, avgRed, avgIR);
                }
            }

            if (qmi_updated)
            {
                // UI Updates for Accelerometer (Rate Limited)
                if (report_now)
                {
                    bool debug_active = (ui_BtnDebug != NULL) && lv_obj_has_state(ui_BtnDebug, LV_STATE_CHECKED);

                    if (debug_active)
                    {
                        snprintf(g_str, sizeof(g_str), "%.2f", g_total);
                        // Update Debug Screen (Screen 2)
                        if (ui_LabelG)
                            lv_label_set_text(ui_LabelG, g_str);

                        snprintf(x_str, sizeof(x_str), "%.2f", acc.x);
                        snprintf(y_str, sizeof(y_str), "%.2f", acc.y);
                        snprintf(z_str, sizeof(z_str), "%.2f", acc.z);
                        if (ui_LabelX)
                            lv_label_set_text(ui_LabelX, x_str);
                        if (ui_LabelY)
                            lv_label_set_text(ui_LabelY, y_str);
                        if (ui_LabelZ)
                            lv_label_set_text(ui_LabelZ, z_str);

                        snprintf(gx_str, sizeof(gx_str), "%.0f", gyr.x);
                        snprintf(gy_str, sizeof(gy_str), "%.0f", gyr.y);
                        snprintf(gz_str, sizeof(gz_str), "%.0f", gyr.z);
                        if (ui_LabelGX)
                            lv_label_set_text(ui_LabelGX, gx_str);
                        if (ui_LabelGY)
                            lv_label_set_text(ui_LabelGY, gy_str);
                        if (ui_LabelGZ)
                            lv_label_set_text(ui_LabelGZ, gz_str);
                    }
                }

                // Fall Detection State Machine (Runs fast)
                // Advanced Fall Detection State Machine
                uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());

                switch (fallState)
                {
                case IDLE:
                    // Phase 1: Free Fall (or Pre-Impact)
                    if (g_total < FALL_THRESHOLD_LOW)
                    {
                        // Store current orientation as reference before the chaos starts
                        ref_ax = acc.x;
                        ref_ay = acc.y;
                        ref_az = acc.z;

                        potentialFallCount++;
                        snprintf(info_str, sizeof(info_str), "FreeFall? (Pot:%d)", potentialFallCount);
                        lv_label_set_text(ui_LabelInfo, info_str);

                        fallState = FREE_FALL;
                        stateTimer = now;
                    }
                    else
                    {
                        // Constant update of reference vector while stable (optional, but good for tracking)
                        if (abs(g_total - 1.0f) < 0.1f)
                        {
                            ref_ax = acc.x;
                            ref_ay = acc.y;
                            ref_az = acc.z;
                        }
                    }
                    break;

                case FREE_FALL:
                    // Phase 2: Impact
                    if (g_total > FALL_THRESHOLD_HIGH)
                    {
                        ESP_LOGW(TAG, "!!! IMPACT DETECTED (G: %.2f) !!!", g_total);
                        snprintf(info_str, sizeof(info_str), "IMPACT! (G:%.1f)", g_total);
                        lv_label_set_text(ui_LabelInfo, info_str);

                        fallState = WAITING_FOR_STILLNESS;
                        stateTimer = now;
                    }
                    // Timeout if impact doesn't happen shortly after free fall
                    else if (now - stateTimer > 500)
                    {
                        fallState = IDLE;
                        snprintf(info_str, sizeof(info_str), "Monitoring...");
                        lv_label_set_text(ui_LabelInfo, info_str);
                    }
                    break;

                case WAITING_FOR_STILLNESS:
                    // Phase 3: Stillness & Orientation Check

                    // Allow some time for "settling" after impact (first 1s might be chaotic)
                    if (now - stateTimer < 1000)
                        break;

                    // If we exceeded the stillness duration, check the result
                    if (now - stateTimer > STILLNESS_DURATION_MS)
                    {
                        // Calculate Orientation Change
                        // Dot Product: A . B = |A|*|B|*cos(theta)
                        // theta = acos( (AxBx + AyBy + AzBz) / (NormA * NormB) )

                        float curr_norm = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
                        float ref_norm = sqrt(ref_ax * ref_ax + ref_ay * ref_ay + ref_az * ref_az);

                        float dot = (acc.x * ref_ax + acc.y * ref_ay + acc.z * ref_az);
                        float cos_theta = dot / (curr_norm * ref_norm);

                        // Clamp for float errors
                        if (cos_theta > 1.0f)
                            cos_theta = 1.0f;
                        if (cos_theta < -1.0f)
                            cos_theta = -1.0f;

                        float angle_deg = acosf(cos_theta) * 180.0f / 3.14159f;

                        ESP_LOGI(TAG, "Post-Fall Analysis: Angle Change: %.1f deg", angle_deg);

                        if (angle_deg > ANGLE_THRESHOLD_DEG)
                        {
                            fallCount++;
                            ESP_LOGE(TAG, "!!! FALL CONFIRMED (Angle: %.1f, Stillness Verified) !!!", angle_deg);
                            ESP_LOGE(TAG, "!!! FALL CONFIRMED (Pad:%d)\nAngle:%.1f G:%.2f Lat:%.5f Lon:%.5f", fallCount, angle_deg, g_total, g_latitude, g_longitude);
                            char ble_msg[128];
                            snprintf(ble_msg, sizeof(ble_msg), "FALL_ACCEPTED Angle:%.1f G:%.2f Lat:%.5f Lon:%.5f", angle_deg, g_total, g_latitude, g_longitude);
                            ble_spp_server_send_data((uint8_t *)ble_msg, strlen(ble_msg));

                            snprintf(info_str, sizeof(info_str), "FALL CONFIRMED! (Pad:%d)\nAngle:%.1f G:%.2f Lat:%.5f Lon:%.5f", fallCount, angle_deg, g_total, g_latitude, g_longitude);

                            // Send SMS Alert (Deferred via Screen 4 Countdown)
                            if (example_lvgl_lock(-1))
                            {
                                start_fall_countdown_ui(false);
                                example_lvgl_unlock();
                            }
                        }
                        else
                        {
                            ESP_LOGW(TAG, "Fall rejected: Angle change too small (%.1f)", angle_deg);
                            snprintf(info_str, sizeof(info_str), "False Alarm (Angle:%.0f)", angle_deg);
                        }

                        lv_label_set_text(ui_LabelInfo, info_str);
                        fallState = IDLE;
                    }
                    break;

                case IMPACT_DETECTED:
                    // Unused state in this new simplified flow
                    fallState = WAITING_FOR_STILLNESS;
                    break;
                }
            } // end qmi_updated

            // --- Battery & Screen Timeout Logic (Periodic) ---
            if (report_now)
            {
                // 1. Check Battery
                int batt_mv = axp_get_batt_vol();
                g_batt_pct = axp_get_batt_percent(); // Update global/static
                int batt_pct = g_batt_pct;
                bool charging = axp_is_charging(); // Optional

                static uint64_t last_pwr_log = 0;
                if ((esp_timer_get_time() / 1000 - last_pwr_log) > 3000)
                {
                    last_pwr_log = esp_timer_get_time() / 1000;
                    // Update UI log
                    ESP_LOGI("PWR", "Bat: %d%% (%dmV) - Charging: %d", batt_pct, batt_mv, charging);
                }

                // Update Battery on Watch Face
                if (batt_pct >= 0)
                {
                    char bat_str[32];
                    if (charging)
                    {
                        snprintf(bat_str, sizeof(bat_str), LV_SYMBOL_CHARGE " %d%%", batt_pct);
                        if (ui_LabelInfo)
                        {
                            lv_label_set_text(ui_LabelInfo, bat_str);
                            lv_obj_set_style_text_color(ui_LabelInfo, lv_color_hex(0x00FF00), LV_PART_MAIN); // Green
                        }
                    }
                    else
                    {
                        snprintf(bat_str, sizeof(bat_str), "%d%%", batt_pct);
                        if (ui_LabelInfo)
                        {
                            lv_label_set_text(ui_LabelInfo, bat_str);
                            lv_obj_set_style_text_color(ui_LabelInfo, lv_color_hex(0xFFFFFF), LV_PART_MAIN); // White
                        }
                    }
                }

                // Update Time (Mockup or from GPS)
                // If GPS has time, use it. Else mock or use system time.
                // For now, let's just make it tick or showing something static until we parse time
                // ui_LabelTime is updated elsewhere or here.

                // 2. Check Screen Timeout
                uint64_t now_ms = esp_timer_get_time() / 1000;
                if (screen_is_on && (now_ms - last_touch_time > SCREEN_TIMEOUT_MS))
                {
                    screen_is_on = false;
                    ESP_LOGI("PWR", "Screen TIMEOUT");
                    // Turn off LCD (Sleep)
                    if (panel_handle)
                    {
                        esp_lcd_panel_disp_on_off(panel_handle, false);
                    }

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
                    gpio_set_level((gpio_num_t)EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL);
#endif
                }
            }

            // --- HEARTBEAT (1Hz) ---
            static uint64_t last_heartbeat = 0;
            if (esp_timer_get_time() / 1000 - last_heartbeat > 1000)
            {
                last_heartbeat = esp_timer_get_time() / 1000;
                char beat_msg[64];
                // STATUS G:%.2f P:%d S:%d B:%d L:%.5f,%.5f
                snprintf(beat_msg, sizeof(beat_msg), "STATUS G:%.2f P:%ld S:%ld B:%d Lat:%.5f Lon:%.5f",
                         g_total, heartRate, spo2, g_batt_pct, g_latitude, g_longitude);
                ble_spp_server_send_data((uint8_t *)beat_msg, strlen(beat_msg));
                // ESP_LOGI("BLE", "Sent Heartbeat: %s", beat_msg);
            }

            example_lvgl_unlock();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

extern "C" void update_ble_connection_status(bool connected)
{
    if (example_lvgl_lock(-1))
    {
        if (connected)
        {
            lv_obj_set_style_text_color(ui_LabelBLT, lv_color_hex(0x00FF00), LV_PART_MAIN);
        }
        else
        {
            lv_obj_set_style_text_color(ui_LabelBLT, lv_color_hex(0xFF0000), LV_PART_MAIN); // Red for disconnect
        }
        example_lvgl_unlock();
    }
}

void gps_task(void *arg)
{
    ESP_LOGI("GPS", "Starting GPS Task");

    // Ensure GPS Label is Red initially
    if (example_lvgl_lock(-1))
    {
        lv_obj_set_style_text_color(ui_LabelGPS, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
        example_lvgl_unlock();
    }

    lc76g_init(I2C_NUM_0); // Using shared I2C port

    // --- DEBUG: I2C SCANNER (Disabled) ---
    /*
    ESP_LOGI("I2C_SCAN", "Scanning I2C bus...");
    for (int i = 0; i < 128; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK)
        {
            ESP_LOGI("I2C_SCAN", "Found device at: 0x%02X", i);
        }
    }
    */
    // --------------------------

    uint8_t *buffer = (uint8_t *)malloc(512); // Buffer for NMEA data
    size_t read_len = 0;

    // Line buffer for assembling sentences
    static char line_buf[128];
    static int line_idx = 0;

    if (!buffer)
    {
        ESP_LOGE("GPS", "Failed to allocate GPS buffer");
        vTaskDelete(NULL);
    }

    while (1)
    {
        // Read up to 511 bytes
        esp_err_t ret = lc76g_read_data(buffer, 511, &read_len);
        if (ret == ESP_OK && read_len > 0)
        {
            // Process byte by byte to find newlines
            for (int i = 0; i < read_len; i++)
            {
                char c = (char)buffer[i];
                if (c == '\r' || c == '\n')
                {
                    if (line_idx > 0)
                    {
                        line_buf[line_idx] = 0; // Null terminate
                        parse_nmea(line_buf);   // Parse the complete line
                        line_idx = 0;           // Reset
                    }
                }
                else
                {
                    if (line_idx < sizeof(line_buf) - 1)
                    {
                        line_buf[line_idx++] = c;
                    }
                    else
                    {
                        // Buffer overflow, reset
                        line_idx = 0;
                    }
                }
            }
            // Raw logging (optional, maybe too noisy now)
            // buffer[read_len] = 0;
            // ESP_LOGI("GPS_NMEA", "%s", (char *)buffer);
        }

        // Poll every 0.1s (GPS usually updates at 1Hz or 10Hz)
        vTaskDelay(pdMS_TO_TICKS(100));

        // --- DEMO MODE FALLBACK ---
        // If no GPS fix after startup (e.g., indoor), use Mock Location for App testing
        // Mock Location: Belgrade (44.7866, 20.4489)
        if (g_latitude == 0.0f && g_longitude == 0.0f)
        {
            g_latitude = 44.7866f;
            g_longitude = 20.4489f;
        }
    }

    free(buffer);
}

void gsm_status_task(void *arg)
{
    ESP_LOGI("GSM_TASK", "Starting GSM Status Task");
    while (1)
    {
        // Periodic check (every 10 seconds)
        esp_err_t ret = gsm_check_network();

        if (example_lvgl_lock(-1))
        {
            if (ui_LabelGSM)
            {
                if (ret == ESP_OK)
                {
                    lv_label_set_text(ui_LabelGSM, "GSM: Registered");
                    lv_obj_set_style_text_color(ui_LabelGSM, lv_color_hex(0x00FF00), LV_PART_MAIN);

                    if (ui_LabelGSM_Icon)
                    {
                        lv_obj_set_style_text_color(ui_LabelGSM_Icon, lv_color_hex(0x00FF00), LV_PART_MAIN);
                        lv_obj_set_style_text_color(ui_LabelGSM_Text, lv_color_hex(0x00FF00), LV_PART_MAIN);
                    }
                }
                else
                {
                    lv_label_set_text(ui_LabelGSM, "GSM: No Network");
                    lv_obj_set_style_text_color(ui_LabelGSM, lv_color_hex(0xFF0000), LV_PART_MAIN);

                    if (ui_LabelGSM_Icon)
                    {
                        lv_obj_set_style_text_color(ui_LabelGSM_Icon, lv_color_hex(0xFF0000), LV_PART_MAIN);
                        lv_obj_set_style_text_color(ui_LabelGSM_Text, lv_color_hex(0xFF0000), LV_PART_MAIN);
                    }
                }
            }
            example_lvgl_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// --- SETTINGS TOGGLES (External C) ---
bool g_sound_enabled = true;

extern "C" void toggle_ble(bool enable)
{
    // Use the TAG defined at top of file, or define local
    ESP_LOGI("SETTINGS", "Toggle BLE: %d", enable);
    if (enable)
    {
        ble_spp_server_advertise();
    }
    else
    {
        ble_spp_server_stop_advertising();
        update_ble_connection_status(false);
    }
}

extern "C" void trigger_fall_sms(void)
{
    ESP_LOGE("ALERT", "=== FALL SIMULATION TRIGGERED ===");

    char sms_msg[160];
    snprintf(sms_msg, sizeof(sms_msg), "UPOZORENJE! Detektovan SIMULIRAN pad!\nLokacija: https://maps.google.com/?q=%.6f,%.6f\nPuls: %ld",
             g_latitude, g_longitude, heartRate);

    gsm_send_sms_async(ui_get_phone_number(), sms_msg);
}

extern "C" void trigger_real_fall_sms(void)
{
    ESP_LOGE("ALERT", "=== REAL FALL DETECTED (Timer reached 0) ===");

    char sms_msg[160];
    snprintf(sms_msg, sizeof(sms_msg), "UPOZORENJE! Detektovan pad!\nLokacija: https://maps.google.com/?q=%.6f,%.6f\nPuls: %ld",
             g_latitude, g_longitude, heartRate);

    gsm_send_sms_async(ui_get_phone_number(), sms_msg);
}

extern "C" void toggle_sound(bool enable)
{
    g_sound_enabled = enable;
    ESP_LOGI("SETTINGS", "Toggle Sound: %d", enable);
}
