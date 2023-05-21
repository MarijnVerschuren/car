#include "main.h"
#include "sys.h"
#include "gpio.h"
#include "tim.h"
#include "pwm.h"
#include "usart.h"
#include "crc.h"
#include "i2c.h"
#include "encoder.h"
#include "watchdog.h"


#define HC12_SET_PORT GPIOB
#define HC12_SET_PIN 0


io_buffer_t* uart_buf;
volatile struct {
	uint8_t throttle;
	uint8_t steering;
	uint16_t reverse	: 1;
	uint16_t boost		: 1;	// enable throttle multiplication
	uint16_t flags		: 14;
} command;
volatile struct {
	uint32_t rpm_a;
	uint32_t rpm_b;
	uint32_t rpm_c;
	uint32_t rpm_d;
} state;


extern void TIM1_UP_TIM10_IRQHandler(void) {
	TIM10->SR = 0x0;  // clear interrupt flags
	uint32_t data, data_crc, crc;

	while (((uint32_t)(uart_buf->i - uart_buf->o)) > 8) {
		uart_buf->o = (uart_buf->o + 1) % uart_buf->size;
		if (uart_buf->size - uart_buf->o < 8) { uart_buf->o = uart_buf->size - 1; continue;	}

		data = *((uint32_t*)(uart_buf->ptr + uart_buf->o));
		data_crc = *((uint32_t*)(uart_buf->ptr + uart_buf->o + 4));

		reset_CRC();
		CRC->DR = data;		// loading data into the crc device
		crc = CRC->DR;		// reading the crc result

		if (data_crc == crc) { reset_watchdog(); *((uint32_t*)&command) = data; }
	}
}

extern void TIM1_TRG_COM_TIM11_IRQHandler(void) {
	TIM11->SR = 0x0;  // clear interrupt flags
	uint16_t mask = TIM_SR_CC1OF | TIM_SR_CC2OF;
	state.rpm_a = TIM2->CNT; TIM2->CNT = 0;
	state.rpm_b = TIM3->CNT; TIM3->CNT = 0;
	state.rpm_c = TIM4->CNT; TIM4->CNT = 0;
	state.rpm_d = TIM5->CNT; TIM5->CNT = 0;
	if (TIM2->SR & mask) { state.rpm_a = 0xffffffff; TIM2->SR &= ~mask; }
	if (TIM3->SR & mask) { state.rpm_b = 0xffffffff; TIM3->SR &= ~mask; }
	if (TIM4->SR & mask) { state.rpm_c = 0xffffffff; TIM4->SR &= ~mask; }
	if (TIM5->SR & mask) { state.rpm_d = 0xffffffff; TIM5->SR &= ~mask; }
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

	// UART input
	enable_CRC();
	uart_buf = new_buffer(128);
	if (!uart_buf) { return -1; }  // allocation error
	fconfig_UART(USART1_TX_A9, USART1_RX_A10, 9600, USART_OVERSAMPLING_16);
	start_USART_read_irq(USART1, uart_buf, 1);

	// UART buffer polling interrupt
	config_TIM(TIM10, 100, 10000);  // 100 Hz
	start_TIM_update_irq(TIM10);  // TIM1_UP_TIM10_IRQHandler
	start_TIM(TIM10);

	// I2C TODO: finish EEPROM library
	config_I2C(I2C1_SCL_B8, I2C1_SDA_B9, 0x00);

	// encoders
	config_encoder_S0S90(TIM2_CH1_A15, TIM2_CH2_B3);
	config_encoder_S0S90(TIM3_CH1_A6, TIM3_CH2_A7);
	config_encoder_S0S90(TIM4_CH1_B6, TIM4_CH2_B7);
	config_encoder_S0S90(TIM5_CH1_A0, TIM5_CH2_A1);
	start_encoder_S0S90(TIM2);
	start_encoder_S0S90(TIM3);
	start_encoder_S0S90(TIM4);
	start_encoder_S0S90(TIM5);

	// encoder polling interrupt
	config_TIM(TIM11, 100, 10000);  // 100 Hz
	start_TIM_update_irq(TIM11);  // TIM1_TRG_COM_TIM11_IRQn
	start_TIM(TIM11);

	// watchdog
	// 32kHz / (4 << prescaler)
	config_watchdog(1, 0xffful);  // 1s timeout
	start_watchdog();

	// PWM output
	config_PWM(TIM9_CH1_A2, 100, 20000);	TIM9->CCR1 = 950;	// steering 750 - 950 - 1150
	config_PWM(TIM9_CH2_A3, 100, 20000);	TIM9->CCR2 = 1500;	// throttle 1500 - 2500

	// main loop
	for(;;) {
		uint32_t throttle = command.throttle;
		if (command.boost) { throttle *= 3.5; }
		TIM9->CCR1 = (950 + (int16_t)((command.steering - 128) * 1.5625));		// multiplied by constant that scales it from [-128, 127] to [-200, 200]
		TIM9->CCR2 = (1500 + (throttle * (1 + -2 * command.reverse)));			// idle +- 512  (around + 1000 is max)
	}
}