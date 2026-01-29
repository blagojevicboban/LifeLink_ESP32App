#include "max30102.h"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "MAX30102";

// Register Addresses
#define MAX30102_INT_STAT1 0x00
#define MAX30102_INT_STAT2 0x01
#define MAX30102_INT_ENABLE1 0x02
#define MAX30102_INT_ENABLE2 0x03
#define MAX30102_FIFO_WR_PTR 0x04
#define MAX30102_OVF_COUNTER 0x05
#define MAX30102_FIFO_RD_PTR 0x06
#define MAX30102_FIFO_DATA 0x07
#define MAX30102_FIFO_CONFIG 0x08
#define MAX30102_MODE_CONFIG 0x09
#define MAX30102_SPO2_CONFIG 0x0A
#define MAX30102_LED1_PA 0x0C
#define MAX30102_LED2_PA 0x0D
#define MAX30102_LED3_PA 0x0E
#define MAX30102_PILOT_PA 0x10
#define MAX30102_MULTILED_CONFIG1 0x11
#define MAX30102_MULTILED_CONFIG2 0x12
#define MAX30102_TEMP_INT 0x1F
#define MAX30102_TEMP_FRAC 0x20
#define MAX30102_TEMP_CONFIG 0x21
#define MAX30102_PROX_INT_THRESH 0x30
#define MAX30102_REV_ID 0xFE
#define MAX30102_PART_ID 0xFF

MAX30102::MAX30102()
{
    head = 0;
    tail = 0;
}

bool MAX30102::begin(i2c_port_t wirePort, uint8_t i2cAddr)
{
    _i2cPort = wirePort;
    _i2cAddr = i2cAddr;

    // Check if sensor is connected by reading PART ID
    uint8_t partID = readRegister8(MAX30102_PART_ID);
    if (partID != 0x15)
    { // MAX30102 Part ID
        ESP_LOGE(TAG, "MAX30102 not found (ID: 0x%02X)", partID);
        return false;
    }

    setup(); // Run default setup
    return true;
}

void MAX30102::setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange)
{
    // Reset all configuration, threshold, and data registers to POR values
    writeRegister8(MAX30102_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay after reset

    // FIFO Configuration
    // smp_ave: 0=1, 1=2, 2=4, 3=8, 4=16, 5=32
    // fifo_rollover_en: 1=enable
    // fifo_a_full: 15 (warn when 0 slots remain)
    uint8_t smp_ave = 0;
    switch (sampleAverage)
    {
    case 1:
        smp_ave = 0;
        break;
    case 2:
        smp_ave = 1;
        break;
    case 4:
        smp_ave = 2;
        break;
    case 8:
        smp_ave = 3;
        break;
    case 16:
        smp_ave = 4;
        break;
    case 32:
        smp_ave = 5;
        break;
    default:
        smp_ave = 2;
        break; // Default 4
    }
    writeRegister8(MAX30102_FIFO_CONFIG, (smp_ave << 5) | 0x1F); // Roll over enabled (0x10) | almost full = 15 (0x0F) -> 0x1F? No, bit 4 is rollover. 0x10 | 0xF = 0x1F.

    // Mode Configuration
    // 0x02 for Red only, 0x03 for Red + IR, 0x07 Multi-LED
    if (ledMode == 3)
        ledMode = 0x07; // Multi-LED
    else if (ledMode == 2)
        ledMode = 0x03; // SpO2 mode
    else
        ledMode = 0x02; // Heart Rate mode
    writeRegister8(MAX30102_MODE_CONFIG, ledMode);

    // SpO2 Configuration
    // ADC Range: 0=2048, 1=4096, 2=8192, 3=16384
    uint8_t spo2_adc_range = 0;
    if (adcRange < 4096)
        spo2_adc_range = 0;
    else if (adcRange < 8192)
        spo2_adc_range = 1;
    else if (adcRange < 16384)
        spo2_adc_range = 2;
    else
        spo2_adc_range = 3;

    // Sample Rate: 0=50, 1=100, 2=200, 3=400, 4=800, 5=1000, 6=1600, 7=3200
    uint8_t spo2_sample_rate = 1; // Default 100
    if (sampleRate <= 50)
        spo2_sample_rate = 0;
    else if (sampleRate <= 100)
        spo2_sample_rate = 1;
    else if (sampleRate <= 200)
        spo2_sample_rate = 2;
    else if (sampleRate <= 400)
        spo2_sample_rate = 3;
    else if (sampleRate <= 800)
        spo2_sample_rate = 4;
    else if (sampleRate <= 1000)
        spo2_sample_rate = 5;
    else if (sampleRate <= 1600)
        spo2_sample_rate = 6;
    else
        spo2_sample_rate = 7;

    // Pulse Width: 0=69, 1=118, 2=215, 3=411
    uint8_t spo2_pulse_width = 3; // Default 411
    if (pulseWidth <= 69)
        spo2_pulse_width = 0;
    else if (pulseWidth <= 118)
        spo2_pulse_width = 1;
    else if (pulseWidth <= 215)
        spo2_pulse_width = 2;
    else
        spo2_pulse_width = 3;

    writeRegister8(MAX30102_SPO2_CONFIG, (spo2_adc_range << 5) | (spo2_sample_rate << 2) | spo2_pulse_width);

    // LED Pulse Amplitude Configuration
    writeRegister8(MAX30102_LED1_PA, powerLevel); // Red
    writeRegister8(MAX30102_LED2_PA, powerLevel); // IR

    // Clear FIFO pointers
    writeRegister8(MAX30102_FIFO_WR_PTR, 0x00);
    writeRegister8(MAX30102_OVF_COUNTER, 0x00);
    writeRegister8(MAX30102_FIFO_RD_PTR, 0x00);
}

// Low-level I2C Write
void MAX30102::writeRegister8(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(_i2cPort, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

// Low-level I2C Read
uint8_t MAX30102::readRegister8(uint8_t reg)
{
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(_i2cPort, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return data;
}

void MAX30102::check()
{
    // Read Register 0x04 (Write Pointer) and 0x06 (Read Pointer)
    uint8_t readPtr = readRegister8(MAX30102_FIFO_RD_PTR);
    uint8_t writePtr = readRegister8(MAX30102_FIFO_WR_PTR);

    if (readPtr != writePtr)
    {
        // Calculate number of samples
        int numSamples = writePtr - readPtr;
        if (numSamples < 0)
            numSamples += 32;

        for (int i = 0; i < numSamples; i++)
        {
            readFIFO();
        }
    }
}

// Read FIFO
bool MAX30102::readFIFO()
{
    uint8_t data[6]; // 3 bytes Red, 3 bytes IR
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MAX30102_FIFO_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(_i2cPort, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
        return false;

    uint32_t un_temp;
    un_temp = (uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[2];
    un_temp &= 0x03FFFF; // Mask to 18 bits
    redBuffer[head] = un_temp;

    un_temp = (uint32_t)data[3] << 16 | (uint32_t)data[4] << 8 | (uint32_t)data[5];
    un_temp &= 0x03FFFF;
    irBuffer[head] = un_temp;

    head++;
    head %= STORAGE_SIZE; // Circular buffer

    return true;
}

bool MAX30102::available()
{
    return (head != tail);
}

void MAX30102::nextSample()
{
    if (available())
    {
        tail++;
        tail %= STORAGE_SIZE;
    }
}

uint32_t MAX30102::getRed()
{
    return redBuffer[tail];
}

uint32_t MAX30102::getIR()
{
    return irBuffer[tail];
}

void MAX30102::shutDown()
{
    bitMask(MAX30102_MODE_CONFIG, 0x80, 0x80);
}

void MAX30102::wakeUp()
{
    bitMask(MAX30102_MODE_CONFIG, 0x80, 0x00);
}

void MAX30102::setPulseAmplitudeRed(uint8_t amplitude)
{
    writeRegister8(MAX30102_LED1_PA, amplitude);
}

void MAX30102::setPulseAmplitudeIR(uint8_t amplitude)
{
    writeRegister8(MAX30102_LED2_PA, amplitude);
}

void MAX30102::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
    // Grab current register context
    uint8_t originalContents = readRegister8(reg);

    // Zero-out the portions of the register we are interested in
    originalContents = originalContents & ~mask;

    // Change contents
    writeRegister8(reg, originalContents | thing);
}

void MAX30102::clearFIFO()
{
    writeRegister8(MAX30102_FIFO_WR_PTR, 0);
    writeRegister8(MAX30102_OVF_COUNTER, 0);
    writeRegister8(MAX30102_FIFO_RD_PTR, 0);
}
