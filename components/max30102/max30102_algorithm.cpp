/** \file algorithm.cpp
 *******************************************************************************
 * \brief Algorithm for calculating heart rate and SpO2
 *
 * This file contains the implementation of the algorithm to calculate heart rate
 * and SpO2 from the IR and Red LED data.
 *******************************************************************************
 */

#include "algorithm.h"
#include <math.h>

const uint16_t auw_hamm[31] = {41, 276, 512, 276, 41}; // hamming window
const uint8_t uch_spo2_table[184] = {95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
                                     99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                                     100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
                                     97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
                                     90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
                                     80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
                                     66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
                                     49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
                                     28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
                                     3, 2, 1};
static int32_t an_dx[BUFFER_SIZE - MA4_SIZE]; // delta
static int32_t an_x[BUFFER_SIZE];             // ir
static int32_t an_y[BUFFER_SIZE];             // red

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid)
{
    uint32_t un_ir_mean, un_only_once;
    int32_t k, n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks, n_c_min;
    int32_t n_peak_interval_sum;
    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc;
    int32_t n_y_dc_max, n_x_dc_max;
    int32_t n_y_dc_max_idx, n_x_dc_max_idx;
    int32_t an_ir_valley_locs[15];
    int32_t n_exact_ir_valley_locs[15];
    int32_t n_dx_peak_locs[15];
    int32_t n_ratio_average;
    int32_t n_n_ratio_stored;
    int32_t i_ratio_count_legend;
    int32_t n_ratio_sorted[5];
    int32_t n_ratio;

    // calculates DC mean and subtract DC from ir
    un_ir_mean = 0;
    for (k = 0; k < n_ir_buffer_length; k++)
        un_ir_mean += pun_ir_buffer[k];
    un_ir_mean = un_ir_mean / n_ir_buffer_length;
    for (k = 0; k < n_ir_buffer_length; k++)
        an_x[k] = pun_ir_buffer[k] - un_ir_mean;

    // 4 pt Moving Average
    for (k = 0; k < n_ir_buffer_length - MA4_SIZE; k++)
    {
        an_dx[k] = (an_x[k + 3] - an_x[k]);
    }

    // using dx and x to find peak location
    n_exact_ir_valley_locs_count = 0;
    for (k = 0; k < n_ir_buffer_length - MA4_SIZE; k++)
    {
        if (an_dx[k] > 0)
        {
            // no change
        }
        else
        {
            n_middle_idx = k + MA4_SIZE / 2;
            if (an_dx[k - 1] > 0)
            {
                n_exact_ir_valley_locs[n_exact_ir_valley_locs_count] = n_middle_idx;
                n_exact_ir_valley_locs_count++;
                if (n_exact_ir_valley_locs_count > 15)
                    n_exact_ir_valley_locs_count = 15;
            }
        }
    }

    // load raw value again for SPO2 calculation : RED(=y) and IR(=x)
    for (k = 0; k < n_ir_buffer_length; k++)
    {
        an_x[k] = pun_ir_buffer[k];
        an_y[k] = pun_red_buffer[k];
    }

    // find precise min near an_ir_valley_locs
    n_exact_ir_valley_locs_count = 0;
    for (k = 0; k < n_ir_buffer_length - MA4_SIZE; k++)
    {
        if (an_dx[k] > 0)
        {
            //
        }
        else
        {
            if (an_dx[k - 1] > 0)
            {
                n_middle_idx = k + MA4_SIZE / 2;
                n_c_min = 16777216; // 2^24
                n_th1 = 0;
                for (i = 0; i < 5; i++)
                {
                    if (an_x[n_middle_idx - 2 + i] < n_c_min)
                    {
                        n_c_min = an_x[n_middle_idx - 2 + i];
                        n_th1 = n_middle_idx - 2 + i;
                    }
                }
                an_ir_valley_locs[n_exact_ir_valley_locs_count] = n_th1;
                n_exact_ir_valley_locs_count++;
                if (n_exact_ir_valley_locs_count > 15)
                    n_exact_ir_valley_locs_count = 15;
            }
        }
    }

    // Calculate HR
    n_peak_interval_sum = 0;
    if (n_exact_ir_valley_locs_count >= 2)
    {
        for (k = 1; k < n_exact_ir_valley_locs_count; k++)
            n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k - 1]);
        n_peak_interval_sum = n_peak_interval_sum / (n_exact_ir_valley_locs_count - 1);
        *pn_heart_rate = (int32_t)(6000 / n_peak_interval_sum);
        *pch_hr_valid = 1;
    }
    else
    {
        *pn_heart_rate = -999;
        *pch_hr_valid = 0;
    }

    // Calculate SPO2
    n_n_ratio_stored = 0;
    for (k = 0; k < n_exact_ir_valley_locs_count; k++)
    {
        if (an_ir_valley_locs[k] > BUFFER_SIZE - 20)
            break; // prevent buffer overflow

        n_y_dc_max_idx = -1;
        n_x_dc_max_idx = -1;

        n_y_dc_max = -16777216;
        n_x_dc_max = -16777216;

        if (k > 0)
        {
            // Find max between two valleys
            for (i = an_ir_valley_locs[k - 1]; i < an_ir_valley_locs[k]; i++)
            {
                if (an_x[i] > n_x_dc_max)
                {
                    n_x_dc_max = an_x[i];
                    n_x_dc_max_idx = i;
                }
                if (an_y[i] > n_y_dc_max)
                {
                    n_y_dc_max = an_y[i];
                    n_y_dc_max_idx = i;
                }
            }
            n_y_ac = (an_y[an_ir_valley_locs[k - 1]] - n_y_dc_max) * (an_y[an_ir_valley_locs[k]] - n_y_dc_max); // red
            n_y_ac = (int32_t)sqrt(n_y_ac);
            n_x_ac = (an_x[an_ir_valley_locs[k - 1]] - n_x_dc_max) * (an_x[an_ir_valley_locs[k]] - n_x_dc_max); // ir
            n_x_ac = (int32_t)sqrt(n_x_ac);

            n_y_dc_max = an_y[an_ir_valley_locs[k - 1]]; // DC red
            n_x_dc_max = an_x[an_ir_valley_locs[k - 1]]; // DC ir

            if (n_y_ac > 0 && n_x_ac > 0 && n_y_dc_max > 0 && n_x_dc_max > 0)
            {
                n_n_ratio_stored++;
                n_ratio = n_y_ac * 100 * n_x_dc_max;       // numerator
                n_ratio = n_ratio / (n_x_ac * n_y_dc_max); // denominator
                if (n_n_ratio_stored <= 5)
                    n_ratio_sorted[n_n_ratio_stored - 1] = n_ratio;
            }
        }
    }

    if (n_n_ratio_stored > 2)
    {
        n_ratio_average = 0;
        for (k = 0; k < n_n_ratio_stored; k++)
            n_ratio_average += n_ratio_sorted[k];
        n_ratio_average = n_ratio_average / n_n_ratio_stored;

        if (n_ratio_average > 2 && n_ratio_average < 184)
        {
            n_spo2_calc = uch_spo2_table[n_ratio_average];
            *pn_spo2 = n_spo2_calc;
            *pch_spo2_valid = 1;
        }
        else
        {
            *pn_spo2 = -999;
            *pch_spo2_valid = 0;
        }
    }
    else
    {
        *pn_spo2 = -999;
        *pch_spo2_valid = 0;
    }
}
