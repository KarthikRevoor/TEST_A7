/*
 * init.h
 *
 *  Created on: Nov 24, 2024
 *      Author: karth
 */
#include "stm32f091xc.h"
#define BUFFER_SIZE 1024    // Total buffer size for DMA
#define DAC_RESOLUTION 4096 // 12-bit DAC resolution
#define SAMPLE_RATE 48000   // Output sampling rate (48 kHz)
#define NOTE_DURATION_MS 2000 // Duration of each note in milliseconds

#define SAMPLE_SIZE_440Hz 109
#define SAMPLE_SIZE_587Hz 82
#define SAMPLE_SIZE_659Hz 73
#define SAMPLE_SIZE_880Hz 55

#define SINE_TABLE_SIZE_440Hz 981
#define SINE_TABLE_SIZE_587Hz 984
#define SINE_TABLE_SIZE_659Hz 1022
#define SINE_TABLE_SIZE_880Hz 990

void Generate_SineWave(uint16_t* sine_wave, uint16_t buffer_size, uint16_t sample_size);
void Initialize_SineWaves();
void DAC_Init(void);
void DMA_Init(void);
void begin_DMA(uint32_t *source, uint32_t length);
void Timer1_Init(void);
void Timer6_Init(void);
void Systick_Init(void);
void Systick_Reset(void);
void SysTick_Handler(void);
void ADC_INIT(void);
uint32_t get_Systick_time(void);
void Generate_SineWave_880(uint16_t Buffer_Size);
void Generate_SineWave_659(uint16_t Buffer_Size);
void Generate_SineWave_587(uint16_t Buffer_Size);
void Generate_SineWave_440(uint16_t Buffer_Size);


