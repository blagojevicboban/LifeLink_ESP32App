#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

#define MAX30102_ADDRESS 0x57

class MAX30102
{
public:
    MAX30102();

    // Initialize the sensor
    // wirePort: I2C port number
    // i2cAddr: I2C address (default 0x57)
    bool begin(i2c_port_t wirePort = I2C_NUM_0, uint8_t i2cAddr = MAX30102_ADDRESS);

    // Configuration
    void setup(uint8_t powerLevel = 0x1F, uint8_t sampleAverage = 4, uint8_t ledMode = 2, int sampleRate = 100, int pulseWidth = 411, int adcRange = 4096);

    // Data collection
    void check();      // Checks for new data and updates internal buffer
    bool available();  // Returns true if there is new data
    void nextSample(); // Advances to the next sample

    uint32_t getRed(); // Returns current Red value
    uint32_t getIR();  // Returns current IR value

    // Sensor controls
    void shutDown();
    void wakeUp();
    void setPulseAmplitudeRed(uint8_t amplitude);
    void setPulseAmplitudeIR(uint8_t amplitude);

    // Read Registers
    uint8_t readRegister8(uint8_t reg);
    void writeRegister8(uint8_t reg, uint8_t value);

    // Helpers used for the algorithm
    void clearFIFO();

private:
    i2c_port_t _i2cPort;
    uint8_t _i2cAddr;

// Circular buffer for readings (minimal implementation for now)
#define STORAGE_SIZE 4
    uint32_t redBuffer[STORAGE_SIZE];
    uint32_t irBuffer[STORAGE_SIZE];
    uint8_t head;
    uint8_t tail;

    bool readFIFO();
    void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
};

#endif
