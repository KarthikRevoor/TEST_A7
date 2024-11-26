/*
 * init.h
 *
 *  Created on: Nov 24, 2024
 *      Author: karth
 */
#include "stm32f091xc.h"
#define SINE_TABLE_SIZE 256 // Number of samples in one cycle
#define BUFFER_SIZE 1024    // Total buffer size for DMA
#define DAC_RESOLUTION 4096 // 12-bit DAC resolution
#define SAMPLE_RATE 48000   // Output sampling rate (48 kHz)
#define NOTE_DURATION_MS 2000 // Duration of each note in milliseconds





void DAC_Init(void);
//void Timer6_Init(void);
void DMA_Init(uint16_t *buffer, uint16_t buffer_size);
void generate_waveform();
void generate_sine_table();
void test();
void Timer6_Init(void);
void generate_and_fill_dma_buffer(int frequency);
extern uint16_t sine_table[SINE_TABLE_SIZE];

extern const int frequencies[];
extern volatile uint32_t current_index;
extern volatile uint32_t elapsed_time_ms;
extern volatile uint8_t current_freq_index;

void switch_to_next_frequency(void);
void Timer7_Init(void);
void fill_dma_buffer();

extern uint16_t dma_buffer[BUFFER_SIZE];
