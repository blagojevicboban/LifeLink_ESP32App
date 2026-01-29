#ifndef FFT_ALGO_H
#define FFT_ALGO_H

#include <stdint.h>
#include <math.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // Initialize FFT resources (tables, window)
    // Call this once at startup
    esp_err_t fft_init(int fft_size);

    // Process a batch of data using FFT
    // red_samples: Array of raw RED sensor data (must be of length fft_size)
    // ir_samples: Array of raw IR sensor data (must be of length fft_size)
    // fft_size: Number of samples (MUST be Power of 2, e.g., 512)
    // sampling_freq: Frequency at which samples were collected (e.g., 50 Hz)
    // out_hr: Pointer to store calculated Heart Rate (BPM)
    // out_spo2: Pointer to store calculated SpO2 (%)
    void fft_process(float *red_samples, float *ir_samples, int fft_size, int sampling_freq, float *out_hr, float *out_spo2);

#ifdef __cplusplus
}
#endif

#endif // FFT_ALGO_H
