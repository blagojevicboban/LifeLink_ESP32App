#include "fft_algo.h"
#include "dsps_fft2r.h"
#include "dsps_view.h"
#include "dsps_wind.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "FFT_ALGO";

static float *window_factor = NULL;
static float *fft_input = NULL;
static float *red_magnitude = NULL;
static float *ir_magnitude = NULL;

esp_err_t fft_init(int fft_size)
{
    esp_err_t ret;

    // Initialize FFT table
    ret = dsps_fft2r_init_fc32(NULL, fft_size);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "FFT Init failed: %d", ret);
        return ret;
    }

    // Allocate buffers if not already allocated
    // We assume fft_size is constant for this application
    if (window_factor == NULL)
    {
        window_factor = (float *)calloc(fft_size, sizeof(float));
        fft_input = (float *)calloc(fft_size * 2, sizeof(float)); // Complex: Real, Imag
        red_magnitude = (float *)calloc(fft_size / 2, sizeof(float));
        ir_magnitude = (float *)calloc(fft_size / 2, sizeof(float));

        if (!window_factor || !fft_input || !red_magnitude || !ir_magnitude)
        {
            ESP_LOGE(TAG, "Failed to allocate FFT buffers");
            return ESP_ERR_NO_MEM;
        }

        // Generate Hann window
        dsps_wind_hann_f32(window_factor, fft_size);
    }

    ESP_LOGI(TAG, "FFT Initialized with size %d", fft_size);
    return ESP_OK;
}

// Helper to process one channel and get magnitude spectrum
// Returns the DC component of the signal
static float process_channel_fft(float *raw_data, int fft_size, float *magnitude_out)
{
    float mean_dc = 0;

    // 1. Calculate Mean (DC)
    for (int i = 0; i < fft_size; i++)
    {
        mean_dc += raw_data[i];
    }
    mean_dc /= fft_size;

    // 2. Remove DC & Apply Window
    for (int i = 0; i < fft_size; i++)
    {
        fft_input[i * 2] = (raw_data[i] - mean_dc) * window_factor[i]; // Real
        fft_input[i * 2 + 1] = 0;                                      // Imag
    }

    // 3. FFT Execution
    dsps_fft2r_fc32(fft_input, fft_size);
    dsps_bit_rev_fc32(fft_input, fft_size);

    // 4. Calculate Magnitude
    // We only need the first half (N/2) because input is real
    for (int i = 0; i < fft_size / 2; i++)
    {
        float real = fft_input[i * 2];
        float imag = fft_input[i * 2 + 1];
        magnitude_out[i] = sqrtf(real * real + imag * imag);
    }

    return mean_dc;
}

void fft_process(float *red_samples, float *ir_samples, int fft_size, int sampling_freq, float *out_hr, float *out_spo2)
{
    if (window_factor == NULL)
    {
        ESP_LOGE(TAG, "FFT not initialized!");
        return;
    }

    // Process IR Channel (Better for HR detection)
    float ir_dc = process_channel_fft(ir_samples, fft_size, ir_magnitude);

    // Process Red Channel
    float red_dc = process_channel_fft(red_samples, fft_size, red_magnitude);

    // --- Find Peak Frequency (Heart Rate) in IR Spectrum ---
    int max_idx = 0;
    float max_mag = 0;

    // Search range: 0.7 Hz (42 BPM) to 3.5 Hz (210 BPM)
    int bin_start = (int)(0.7f * fft_size / sampling_freq);
    int bin_end = (int)(3.5f * fft_size / sampling_freq);

    if (bin_start < 2)
        bin_start = 2; // Skip DC/very low freq
    if (bin_end >= fft_size / 2)
        bin_end = fft_size / 2 - 1;

    for (int i = bin_start; i <= bin_end; i++)
    {
        if (ir_magnitude[i] > max_mag)
        {
            max_mag = ir_magnitude[i];
            max_idx = i;
        }
    }

    // Calculate HR
    float freq_hz = (float)max_idx * sampling_freq / fft_size;
    float bpm = freq_hz * 60.0f;
    *out_hr = bpm;

    // --- Calculate SpO2 ---
    // Use the magnitude at the HR peak frequency for both channels
    float ac_ir = ir_magnitude[max_idx];
    float ac_red = red_magnitude[max_idx];

    // R = (AC_red / DC_red) / (AC_ir / DC_ir)
    float R = (ac_red / red_dc) / (ac_ir / ir_dc);

    // Standard calibration formula (e.g., typically SpO2 = 110 - 25 * R)
    // Adjust based on Maxim/reference data
    float spo2 = 110.0f - 25.0f * R;

    // Clamp values
    if (spo2 > 100)
        spo2 = 100;
    if (spo2 < 0)
        spo2 = 0;

    *out_spo2 = spo2;

    // ESP_LOGI(TAG, "FFT Result -> Peak Bin: %d, Freq: %.2f Hz, HR: %.1f, R: %.3f, SpO2: %.1f",
    //          max_idx, freq_hz, bpm, R, spo2);
}
