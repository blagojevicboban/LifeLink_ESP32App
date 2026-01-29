#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include <stdint.h>

#define true 1
#define false 0
#define FS 100
#define BUFFER_SIZE (FS * 5)
#define HR_FIFO_SIZE 7
#define MA4_SIZE 4     // DO NOT CHANGE
#define HAMMING_SIZE 5 // DO NOT CHANGE
#define min(x, y) ((x) < (y) ? (x) : (y))

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);

#endif /* ALGORITHM_H_ */
