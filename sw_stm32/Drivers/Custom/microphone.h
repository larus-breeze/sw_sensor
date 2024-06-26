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
#define MIC_DMA_BUFSIZE_HALFWORDS 4096 // filled after 50ms
#define MIC_DMA_BUFSIZE_BYTES (MIC_DMA_BUFSIZE_HALFWORDS * sizeof(uint16_t))
// the sample buffer contains 512 bytes sampled at 10.254 kHz
// acquisition time 50ms -> 20 Hz min freq.
#define SAMPLE_BUFSIZE (MIC_DMA_BUFSIZE_BYTES / RESAMPLING_RATIO)
#define SAMPLE_BUFSIZE_HALF (SAMPLE_BUFSIZE / 2)

extern Queue <uint8_t *> mic_data_pointer_Q;
extern int8_t audio_samples[2][SAMPLE_BUFSIZE];

#endif /* CUSTOM_MICROPHONE_H_ */
