#include <stdint.h>
#include "fp_trig.h"
#include "init.h"
#include <stdio.h>
#include "test_sine.h"

// Frequencies for musical notes
const int frequencies[] = {440, 587, 659, 880}; // A4, D5, E5, A5
const int num_notes = sizeof(frequencies) / sizeof(frequencies[0]);

uint16_t dma_buffer[BUFFER_SIZE]; // Buffer to hold samples
volatile uint32_t elapsed_time_ms = 0; // Timer-based time tracker
volatile uint8_t current_freq_index = 0; // Index of the current frequency

void generate_and_fill_dma_buffer(int frequency) {
    int cycle_samples = SAMPLE_RATE / frequency; // Samples needed for one cycle
    int total_samples = (BUFFER_SIZE / cycle_samples) * cycle_samples; // Total samples to fill in buffer

    // Fill the DMA buffer with calculated samples
    for (int i = 0; i < total_samples; i++) {
        float angle = (i % cycle_samples) * (TWO_PI / cycle_samples);
        dma_buffer[i] = (uint16_t)((fp_sin(angle) + TRIG_SCALE_FACTOR) * (DAC_RESOLUTION - 1) / (2 * TRIG_SCALE_FACTOR));

        // Debug: Print each DAC value generated
        printf("DAC Value[%d] = %d\n\r", i, dma_buffer[i]);
    }

    // Debug: Print first few values of the DMA buffer
    printf("First 5 DMA buffer values: %d, %d, %d, %d, %d\n\r",
           dma_buffer[0], dma_buffer[1], dma_buffer[2], dma_buffer[3], dma_buffer[4]);
}

void DAC_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable GPIOA clock
    GPIOA->MODER |= (3U << (4 * 2));    // Set PA4 to Analog mode

    RCC->APB1ENR |= RCC_APB1ENR_DACEN; // Enable DAC clock
    DAC->CR |= DAC_CR_TEN1 | (0b000 << DAC_CR_TSEL1_Pos);

    DAC->CR |= DAC_CR_EN1;             // Enable DAC Channel 1

    DAC->CR &= ~DAC_CR_BOFF1;          // Enable DAC output buffer
}

void DMA_Init(uint16_t *buffer, uint16_t buffer_size) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    DMA1_Channel3->CMAR = (uint32_t)buffer;
    DMA1_Channel3->CPAR = (uint32_t)&DAC->DHR12R1;
    DMA1_Channel3->CNDTR = buffer_size;

    DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR;

    DMA1_Channel3->CCR |= DMA_CCR_EN;

    DAC->CR |= DAC_CR_DMAEN1;
}

void Timer6_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    TIM6->PSC = 0;
    TIM6->ARR = SAMPLE_RATE / BUFFER_SIZE - 1;

    TIM6->DIER |= TIM_DIER_UDE;
    TIM6->CR2 |= TIM_CR2_MMS_1;
    TIM6->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void Timer7_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    TIM7->PSC = 47999;
    TIM7->ARR = 1; // Adjusted for a 1 ms tick

    TIM7->DIER |= TIM_DIER_UIE;
    TIM7->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM7_IRQn);
}

void TIM7_IRQHandler(void) {
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF;
        elapsed_time_ms++;

        if (elapsed_time_ms >= 2000) {  // Switch every 2 seconds
            switch_to_next_frequency();
            elapsed_time_ms = 0;
            printf("Switching to frequency: %d Hz\n\r", frequencies[current_freq_index]);
        }
    }
}

void switch_to_next_frequency(void) {
    current_freq_index = (current_freq_index + 1) % num_notes;

    // Generate and fill the DMA buffer for the current frequency
    generate_and_fill_dma_buffer(frequencies[current_freq_index]);
}
