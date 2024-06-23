/*
 * microphone.h
 *
 *  Created on: Jun 22, 2024
 *      Author: schaefer
 */

#ifndef CUSTOM_MICROPHONE_H_
#define CUSTOM_MICROPHONE_H_

// byte samples come with 2.625 MHz / 8 = 328.125 kHz
// resampling by 32 gives 10.254 kHz
#define RESAMPLING_RATIO 32
#define NUM_BUFFERS 2 // double buffered for DMA and for uSD writing
#define MIC_DMA_BUFSIZE_HALFWORDS 8192 // half filled after 25ms
#define MIC_DMA_BUFSIZE_BYTES (MIC_DMA_BUFSIZE_HALFWORDS * 2)
#define SAMPLE_BUFSIZE (MIC_DMA_BUFSIZE_BYTES / RESAMPLING_RATIO / NUM_BUFFERS)

extern Queue <uint8_t *> mic_data_pinter_Q;
extern int8_t samples_at_5_kHz[2][SAMPLE_BUFSIZE];

#endif /* CUSTOM_MICROPHONE_H_ */
