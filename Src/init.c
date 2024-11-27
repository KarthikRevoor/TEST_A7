#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <utilities.h>
#include <stm32f091xc.h>
#include <stm32f0xx.h>
#include <string.h>
#include "fp_trig.h"
#include "autocorrelate.h"
#include <string.h>
#include "init.h"


#define SAMP_FREQ 96000


volatile uint32_t Systick_ms = 0;
int count = 1;
uint16_t ADC_Buffer[BUFFER_SIZE];

static uint16_t *dma_buffer = NULL; // Pointer to the buffer
static uint32_t dma_buffer_size = 0; // Size of the buffer

typedef enum {
    SINE_440 = 440,
    SINE_587 = 587,
    SINE_659 = 659,
    SINE_880 = 880
} WaveFrequency;



WaveFrequency current_state = SINE_440;  // Start with 440Hz sine wave

uint16_t sine_wave_440[BUFFER_SIZE];
uint16_t sine_wave_587[BUFFER_SIZE];
uint16_t sine_wave_659[BUFFER_SIZE];
uint16_t sine_wave_880[BUFFER_SIZE];

void Generate_SineWave_440(uint16_t Buffer_Size) {
	memset(sine_wave_440,0,sizeof(sine_wave_440));
	 for (int i = 0; i < Buffer_Size; i++) {
		 sine_wave_440[i] = fp_sin(i * TWO_PI / SAMPLE_SIZE_440Hz) + TRIG_SCALE_FACTOR;
	    }
}

void Generate_SineWave_587(uint16_t Buffer_Size) {
	memset(sine_wave_587,0,sizeof(sine_wave_587));
	 for (int i = 0; i < Buffer_Size; i++) {
		 sine_wave_587[i] = fp_sin(i * TWO_PI / SAMPLE_SIZE_587Hz) + TRIG_SCALE_FACTOR;
	    }
}

void Generate_SineWave_659(uint16_t Buffer_Size) {
	memset(sine_wave_659,0,sizeof(sine_wave_659));
	 for (int i = 0; i < Buffer_Size; i++) {
		 sine_wave_659[i] = fp_sin(i * TWO_PI / SAMPLE_SIZE_659Hz) + TRIG_SCALE_FACTOR;
	    }
}

void Generate_SineWave_880(uint16_t Buffer_Size) {
	memset(sine_wave_880,0,sizeof(sine_wave_880));
	 for (int i = 0; i < Buffer_Size; i++) {
		 sine_wave_880[i] = fp_sin(i * TWO_PI / SAMPLE_SIZE_880Hz) + TRIG_SCALE_FACTOR;
	    }
}


void ADC_INIT(void) {
	    // Enable peripheral clocks for ADC and GPIOA
	    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;    // Enable ADC clock
	    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;   // Enable GPIOA clock

	    // Configure PA4 as analog input
	    //GPIOA->MODER |= (3U << GPIO_MODER_MODER4_Pos); // Set MODER4[1:0] = 11 (Analog mode)

	    // Enable HSI14 oscillator for ADC and wait for it to be ready
	    RCC->CR2 |= RCC_CR2_HSI14ON;    // Enable HSI14
	    while (!(RCC->CR2 & RCC_CR2_HSI14RDY)) {
	        // Add a timeout here for robustness if needed
	    }

	    // Configure ADC clock to use HSI14 (CKMODE = 00)
		MODIFY_FIELD(ADC1->CFGR2, ADC_CFGR2_CKMODE, 0);

	    // Set the sampling time (shortest time for faster conversion)
	    ADC1->SMPR = 0; // SMP[2:0] = 000 (1.5 ADC clock cycles)

	    // Configure ADC channel selection for PA4 (channel 4)
	    ADC1->CHSELR = ADC_CHSELR_CHSEL4;

	    // Configure ADC external trigger for TIM1_TRGO
		ADC1->CR |= ADC_CR_ADSTART;
		ADC1->CFGR1|=0<<ADC_CFGR1_CONT_Pos;
	    ADC1->CFGR1 |= (0 << ADC_CFGR1_EXTSEL_Pos)| ADC_CFGR1_EXTEN_1;

	    // Enable ADC
	    if (ADC1->ISR & ADC_ISR_ADRDY) {
	        ADC1->ISR |= ADC_ISR_ADRDY; // Clear ADRDY flag if set
	    }
	    ADC1->CR |= ADC_CR_ADEN; // Enable ADC
	    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {
	        // Wait until ADC is ready (add timeout if required)
	    }
}

void Timer1_Init(void) {
    // Enable Timer 1 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Configure Timer 1 prescaler and ARR
    TIM1->PSC = 0;              // Prescaler for 1 MHz timer clock
    TIM1->ARR = 499;              // Auto-reload for 96 kHz frequency
    TIM1->CR1 |= TIM_CR1_ARPE;   // Auto-reload preload enable

    // Set trigger output (TRGO) on update event
    TIM1->CR2 |= TIM_CR2_MMS_1;  // TRGO = Update event
	TIM1->DIER = TIM_DIER_UIE;
	TIM1->DIER = TIM_DIER_TIE;

    // Enable Timer 1
    TIM1->CR1 |= TIM_CR1_CEN;
}

void DAC_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable GPIOA clock
    GPIOA->MODER |= (3U << (4 * 2));    // Set PA4 to Analog mode

    RCC->APB1ENR |= RCC_APB1ENR_DACEN; // Enable DAC clock
    DAC->CR |= DAC_CR_TEN1 | (0b000 << DAC_CR_TSEL1_Pos);

    DAC->CR |= DAC_CR_EN1;             // Enable DAC Channel 1

    DAC->CR &= ~DAC_CR_BOFF1;          // Enable DAC output buffer
}

void DMA_Init(void) {
    if (dma_buffer == NULL || dma_buffer_size == 0) {
        printf("Error: DMA buffer or size not set!\n");
        return;
    }
	 // Enable DMA2 clock
	    RCC->AHBENR |= RCC_AHBENR_DMA2EN;

	    // Memory to peripheral mode, 16-bit data
	    DMA2_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_CIRC;
	    MODIFY_FIELD(DMA2_Channel3->CCR, DMA_CCR_MSIZE, 1);
	    MODIFY_FIELD(DMA2_Channel3->CCR, DMA_CCR_PSIZE, 1);
	    MODIFY_FIELD(DMA2_Channel3->CCR, DMA_CCR_PL, 3);

	    NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch3_5_IRQn, 3);
	    NVIC_ClearPendingIRQ(DMA1_Ch4_7_DMA2_Ch3_5_IRQn);
	    NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch3_5_IRQn);

	    // DMA2 Channel 3 requests come from TIM6 UP update (CxS = 0001)
	    MODIFY_FIELD(DMA2->CSELR, DMA_CSELR_C3S, 1);
	    // Configure source and destination
	    DMA2_Channel3->CMAR = (uint32_t)dma_buffer;            // Memory address
	    DMA2_Channel3->CPAR = (uint32_t)&(DAC->DHR12R1);       // DAC data register
	    DMA2_Channel3->CNDTR = dma_buffer_size;                // Number of transfers

	    // Start DMA
	    DMA2_Channel3->CCR |= DMA_CCR_EN;
	}

void begin_DMA(uint32_t *source, uint32_t length)
{
    DMA2_Channel3->CCR |= DMA_CCR_TCIE;
    DMA2_Channel3->CNDTR = length;
    DMA2_Channel3->CMAR = (uint32_t) source;
    DMA2_Channel3->CPAR = (uint32_t) &(DAC->DHR12R1);
    DMA2_Channel3->CCR |= DMA_CCR_EN;
}

void DMA1_CH4_5_6_7_DMA2_CH3_4_5_IRQHandler(void)
{

    if (DMA2->ISR & DMA_ISR_TCIF3)
    {
        // DMA2 Channel 3 transfer complete, so do something
        // if needed here. Restart, etc.
    }
    DMA2->IFCR |= DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3;

}

void Timer6_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    TIM6->PSC = 0;
    TIM6->ARR = 1000 - 1;

    TIM6->DIER |= TIM_DIER_UDE;
    TIM6->CR1 |= TIM_CR1_CEN;

}


uint32_t get_Systick_time(void)
{
    return Systick_ms;
}

void Systick_Init(void) {
    SysTick_Config(SystemCoreClock / 4);  // 1 ms tick
}

void Systick_Reset(void)
{
	Systick_ms = 0;
}

void audio_input(void)
{
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        uint16_t value;
        ADC1->CR |= ADC_CR_ADSTART;
        // Busy wait until the conversion is done
        while (!(ADC1->ISR & ADC_ISR_EOC)) {}

        value = ADC1->DR;
        ADC_Buffer[i] = value;
    }
}

void analyze_audio(void)
{
	uint32_t min = 0xFFFF, max = 0, sum = 0;

	for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
		if (ADC_Buffer[i] < min) min = ADC_Buffer[i];
		if (ADC_Buffer[i] > max) max = ADC_Buffer[i];
		sum += ADC_Buffer[i];
	}
	uint32_t mean = sum / BUFFER_SIZE;

	int period_value = autocorrelate_detect_period(ADC_Buffer, BUFFER_SIZE , kAC_12bps_unsigned);
	printf("\r\nMin: %lu, Max: %lu, Mean: %lu, Period: %d, Sample Freq: %d\n\r",
			min, max, mean, period_value, (SAMP_FREQ/(period_value)));
}

void SysTick_Handler(void) {
	Systick_ms++;

    // Check if 2 seconds (2000 ms) have elapsed
    if (get_Systick_time() >= 8) {
        Systick_Reset(); // Reset SysTick time

        if (count != 1){
        	analyze_audio();
        }
        count++;

        // Switch to the next state
        switch (current_state) {
            case SINE_440:
                current_state = SINE_587;
                DMA_Init();
                begin_DMA((uint32_t *)sine_wave_440, SINE_TABLE_SIZE_440Hz);
                printf("\r\n\nGenerated %d Samples at 440 Hz. Computed Period = %d Samples\n\r",
                		SINE_TABLE_SIZE_440Hz, SAMPLE_SIZE_440Hz);
                break;

            case SINE_587:
                current_state = SINE_659;
                DMA_Init();
                begin_DMA((uint32_t *)sine_wave_587, SINE_TABLE_SIZE_587Hz);
                printf("\r\n\nGenerated %d Samples at 587 Hz. Computed Period = %d Samples\n\r",
                		SINE_TABLE_SIZE_587Hz, SAMPLE_SIZE_587Hz);
                break;

            case SINE_659:
                current_state = SINE_880;
                DMA_Init();
                begin_DMA((uint32_t *)sine_wave_659, SINE_TABLE_SIZE_659Hz);
                printf("\r\n\nGenerated %d Samples at 659 Hz. Computed Period = %d Samples\n\r",
                		SINE_TABLE_SIZE_659Hz, SAMPLE_SIZE_659Hz);
                break;

            case SINE_880:
                current_state = SINE_440;
                DMA_Init();
                begin_DMA((uint32_t *)sine_wave_880, SINE_TABLE_SIZE_880Hz);
                printf("\r\n\nGenerated %d Samples at 880 Hz. Computed Period = %d Samples\n\r",
                		SINE_TABLE_SIZE_880Hz, SAMPLE_SIZE_880Hz);
                break;
        }
        audio_input();
    }
}
