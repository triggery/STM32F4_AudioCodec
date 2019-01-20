#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"

#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio_codec.h"
#include "waveplayer.h"
#include "stm32_ub_usb_msc_host.h"
#include "ff.h"
//#include "usart_init.h"
//#include "audio_dac.h"
//#include "string.h"

const char filename[] = "sound_2.wav";
DIR fddir;
FRESULT res1;

RCC_ClocksTypeDef RCC_Clocks;
__IO uint8_t RepeatState = 0;
__IO uint16_t CCR_Val = 16826;

int main(void) {

	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1)
			;
	}

	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);

	/*STM_EVAL_LEDOn(LED3);
	 STM_EVAL_LEDOn(LED4);
	 STM_EVAL_LEDOn(LED5);
	 STM_EVAL_LEDOn(LED6);*/

	//audio_init();
	EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);
	uint32_t flag = EVAL_AUDIO_Init(OUTPUT_DEVICE_HEADPHONE, 50 , I2S_AudioFreq_48k);

	//init_USART1();
	UB_USB_MSC_HOST_Init();

	while (UB_USB_MSC_HOST_Do() != USB_MSC_DEV_CONNECTED) {};

	if (UB_Fatfs_Mount(USB_0) == FATFS_OK) {

		if ((res1 = f_opendir(&fddir, "/")) == FR_OK) {
			//audio_play_start(filename);
			WavePlayerStart();
		}
	}

	while (1) {
		//audio_play_loop();
	}
}
