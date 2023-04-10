#include "main.h"
#include "sys.h"
#include "gpio.h"
#include "exti.h"
#include "tim.h"
#include "pwm.h"
#include "usart.h"
#include "crc.h"


#ifdef STM32F4xx

#define LED_GPIO_PORT GPIOC
#define LED_PIN 13

#define HC12_SET_PORT GPIOA
#define HC12_SET_PIN 7


io_buffer_t* uart_buf;
volatile struct {
	uint8_t throttle;
	uint8_t steering;
	uint16_t flags;
} command;

/*
		CRC->CR |= CRC_CR_RESET;	// resetting crc
		CRC->DR = data			& 0xff;
		CRC->DR = (data >> 8)	& 0xff;
		CRC->DR = (data >> 16)	& 0xff;
		CRC->DR = (data >> 24) 	& 0xff;
		uint32_t crc = CRC->DR;		// reading the crc result
*/

extern void TIM3_IRQHandler(void) {
	TIM3->SR &= ~TIM_SR_UIF;
	volatile uint32_t data, data_crc, crc;
	// difference with unsigned int so that the code continues when i index is lower
	while (((uint32_t)(uart_buf->i - uart_buf->o)) > 8) {
		// find start byte (0xFF) TODO: see if it is possible to remove this
		while ((*(uint8_t*)(uart_buf->ptr + uart_buf->o)) != 0xff) {
			uart_buf->o = (uart_buf->o + 1) % uart_buf->size;
			if ((uart_buf->i - uart_buf->o) < 9) { return; }
		}; uart_buf->o = (uart_buf->o + 1) % uart_buf->size;

		data = *((uint32_t*)(uart_buf->ptr + uart_buf->o));
		data_crc = *((uint32_t*)(uart_buf->ptr + uart_buf->o + 4));

		reset_CRC();
		CRC->DR = data;				// loading data into the crc device
		crc = CRC->DR;        		// reading the crc result

		if (data_crc == crc) { *((uint32_t*)&command) = data; }
	}
}

int main(void) {
	// sys_clock: 25Mhz / 15 * 120 / 2 = 100Mhz
	SYS_CLK_Config_t* sys_config = new_SYS_CLK_config();
	set_SYS_PLL_config(sys_config, 15, 120, PLL_P_DIV2, 0, PLL_SRC_HSE);
	set_SYS_CLOCK_config(sys_config, SYS_CLK_SRC_PLL, AHB_CLK_NO_DIV, APBx_CLK_DIV2, APBx_CLK_NO_DIV, 0);
	set_SYS_FLASH_config(sys_config, FLASH_LATENCY4, 1, 1, 1);  // latency is set automatically (when need be)
	set_SYS_tick_config(sys_config, 1, 1);
	sys_clock_init(sys_config); free(sys_config);

	// GPIO output
	config_GPIO(HC12_SET_PORT, HC12_SET_PIN, GPIO_output, GPIO_no_pull, GPIO_open_drain);  // OD IMPORTANT!!!
	GPIO_write(HC12_SET_PORT, HC12_SET_PIN, 1);  // high to disable settings mode
	config_GPIO(LED_GPIO_PORT, LED_PIN, GPIO_output, GPIO_no_pull, GPIO_push_pull);
	GPIO_write(LED_GPIO_PORT, LED_PIN, 1);  // led is active low

	// UART input
	enable_CRC();
	uart_buf = new_buffer(1024);
	if (!uart_buf) { return -1; }  // allocation error
	fconfig_UART(USART1_TX_A9, USART1_RX_A10, 9600, USART_OVERSAMPLING_16);
	start_USART_receive_irq(USART1, uart_buf, 1);

	// UART buffer polling interrupt
	config_TIM(TIM3, 50, 2000);  // 500 Hz
	start_TIM_update_irq(TIM3);  // TIM3_IRQHandler
	start_TIM(TIM3);

	// PWM output
	config_PWM(TIM2_CH1_A0, 100, 20000);		TIM2->CCR1 = 950;	// steering 750 - 950 - 1150
	config_PWM(TIM2_CH3_B10, 100, 20000);	TIM2->CCR3 = 1500;	// throttle 1500 - ... TODO: limits


	// main loop
	for(;;) {
		TIM2->CCR3 = 1500 + command.throttle;
		TIM2->CCR1 = (950 + (int16_t)((command.steering - 128) * 1.5625));  // multiplied by constant that scales it from [-128, 127] to [-200, 200]
		//USART_print(USART1, "hello USART1!?", 100);	delay_ms(333);
	}
}


#elif defined(STM32F3xx)
/* STM32F3xx:
 * 	_WFI(); works here without any additional config
 */
#endif