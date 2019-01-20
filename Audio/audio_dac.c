/*
 * audio_dac.c
 *
 *  Created on: 15 янв. 2019 г.
 *      Author: dmitry
 */
#include "audio_dac.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"

#include "stm32f4_discovery.h"
#include "stm32_ub_usb_msc_host.h"
#include "ff.h"

#define SIZE_OF_BUFF	256
uint8_t AudBuff[SIZE_OF_BUFF*2];

#define TIM6_CLOCK		84000000
#define SAMPLING_RATE	44100

FIL file;
FRESULT res;
UINT cnt;

int is_play = 0;
int dma_htif = 0;
int dma_tcif = 0;

void audio_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM6EN;

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	DAC_InitTypeDef DAC_InitStructure;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	DMA1_Stream5->CR = 0;
	while(DMA1_Stream5->CR & DMA_SxCR_EN);

	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&DAC->DHR12L1;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&AudBuff[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = SIZE_OF_BUFF;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

	DMA_Cmd(DMA1_Stream5, ENABLE);
	DAC_Cmd(DAC_Channel_1, ENABLE);

	DAC_DMACmd(DAC_Channel_1, ENABLE);

	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC | DMA_IT_HT, ENABLE);

	NVIC_InitTypeDef      NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//TIM6->PSC = 0;  			// Prescaler = 1
	//TIM6->ARR = (uint16_t)(TIM6_CLOCK/SAMPLING_RATE); // 1904
	TIM6->CR1 = TIM_CR1_ARPE;	// autoreload preload enable, no other function on
	TIM6->CR2 = TIM_CR2_MMS_1;  // MMS : TRGO Update mode 0b010
}

void setSimpleRate(uint32_t simple) {
	TIM6->PSC = 0;  								  // Prescaler = 1
	TIM6->ARR = (uint16_t)(TIM6_CLOCK/simple); // 1904
}

void wav_convert(uint8_t *buffer, int len)
{
	uint16_t *p = (uint16_t*)(buffer);
	int i;
	for( i = 0 ; i < len/2 ; i++ )
	{
		p[i] += 0x8000;
	}
}

void audio_read_buffer(uint8_t *buffer, int len)
{
	res = f_read ( &file, buffer, len, &cnt );
	if( ( cnt < len ) || (res != FR_OK) ) {
		TIM6->CR1 &= (uint16_t)~TIM_CR1_CEN;
		//GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13);
		f_close(&file);
		is_play = 0;
	}

	wav_convert( buffer, len );
}

int audio_play_start(const char *name)
{
    res = f_open ( &file, name, FA_READ );
    if ( res != FR_OK )
    {
    	//LCD_PutString(5, NEXT_LINE, "ERROR: not f_open ");
    	STM_EVAL_LEDOn(LED3);
    	return 1;
    }

    WAVHEADER header;
    res = f_read( &file, &header, sizeof(WAVHEADER), &cnt );
    if( header.chunkId[0] == 0x52 && header.chunkId[1] == 0x49 && header.chunkId[2] == 0x46 && header.chunkId[3] == 0x46 )
    {
    	//LCD_PutString(5, nextLinePos(), "big endian ");
    }
    else {
    	//LCD_PutString(5, nextLinePos(), "little endian ");
    	STM_EVAL_LEDOn(LED3);
    	return 1;
    }

    if( header.format[0] == 0x57 && header.format[1] == 0x41 && header.format[2] == 0x56 && header.format[3] == 0x45 )
    {
    	//LCD_PutStringShorten(5, nextLinePos(), "is WAVE format ");
    }
    else {
    	//LCD_PutString(5, nextLinePos(), "not WAVE format ");
    	STM_EVAL_LEDOn(LED3);
    	return 1;
    }

    if( header.bitsPerSample != 16 ) {
    	STM_EVAL_LEDOn(LED3);
    	return 1;
    }

    setSimpleRate(header.sampleRate);

    res = f_lseek( &file, 0x2c );

    res = f_read ( &file, &AudBuff[0], sizeof(AudBuff), &cnt );
    if ( res != FR_OK ) {
    	return 1;
    }

    wav_convert( &AudBuff[0], sizeof(AudBuff) );

    TIM6->CR1 |= TIM_CR1_CEN;
	is_play = 1;
	return 0;
}

void audio_play_stop(void)
{
	TIM6->CR1 &= (uint16_t)~TIM_CR1_CEN;
	f_close(&file);
	is_play = 0;
}

void audio_play_loop(void)
{
	if(is_play)
	{
		  if( dma_htif ) {
			  audio_read_buffer(&AudBuff[0], SIZE_OF_BUFF);
			  dma_htif = 0;
		  }
		  else
			  if( dma_tcif ) {
			  audio_read_buffer(&AudBuff[SIZE_OF_BUFF], SIZE_OF_BUFF);
			  dma_tcif = 0;
		  }
	}
}

void DMA1_Stream5_IRQHandler(void)
{
	if( DMA_GetITStatus(DMA1_Stream5, DMA_IT_HTIF5) )
	{
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_HTIF5);
		dma_htif = 1;
	}
	else
		if( DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5) )
		{
			DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
			dma_tcif = 1;
		}
}

