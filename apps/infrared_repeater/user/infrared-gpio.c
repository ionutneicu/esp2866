/*
 * infrared-gpio.c
 *
 *  Created on: Mar 8, 2021
 *      Author: Ionut Neicu
 *      Contact: ionut.neicu@gmail.com
 *
 */
#include "esp_common.h"
#include "esp_system.h"
#include "uart.h"
#include "freertos/task.h"

#include "gpio.h"
#include "uart.h"
#include "infrared-gpio.h"
#include "esp_common.h"
#include "esp_system.h"
#include "c_types.h"

#define GPIO_INPUT(gpio_bits)       ((gpio_input_get()&gpio_bits)?1:0)
#define GPIO_INTERRUPT_ENABLE 		 _xt_isr_unmask(1 << ETS_GPIO_INUM)

extern void gpio_config(GPIO_ConfigTypeDef *pGPIOConfig);

static inline uint32_t asm_ccount(void) {

	uint32_t r;
	asm volatile ("rsr %0, ccount" : "=r"(r));
	return r;
}
#define TICKS()  asm_ccount() / 80
#define USE_IR_MODULATION

typedef struct s_io_port_edge
{
	int 		m_transition_type;  	//0 fall, 1 rise
	uint32		m_transition_timestamp;

} io_port_edge_t;

#define MAX_TRANSITIONS 1500
static volatile uint16  m_num_transitions = 0;
static volatile io_port_edge_t m_transitions[MAX_TRANSITIONS];



#define GPIO_OUTPUT(gpio_bits, bit_value) \
		if(bit_value) gpio_output_conf(gpio_bits, 0, gpio_bits, 0);\
		else gpio_output_conf(0, gpio_bits, gpio_bits, 0)

static void IRAM_ATTR intr_handler() {
	//gpio_pin_intr_state_set(GPIO_ID_PIN(5) , GPIO_PIN_INTR_DISABLE);
	u32 transition_time =  TICKS();
	u32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	if( m_num_transitions < MAX_TRANSITIONS )
	{
		m_transitions[m_num_transitions].m_transition_type = GPIO_INPUT(GPIO_Pin_5);
		m_transitions[m_num_transitions].m_transition_timestamp = transition_time;
		++ m_num_transitions;
	}
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(5) );
	//gpio_pin_intr_state_set(GPIO_ID_PIN(5), GPIO_PIN_INTR_ANYEDGE);
}



void delay_microseconds( uint32 microseconds, bool transmit )
{
	uint32 start_timestamp = TICKS();
#ifdef USE_IR_MODULATION
	uint32 toggle_timestamp = start_timestamp;
	uint8  toggle_value = 0;
	GPIO_OUTPUT(GPIO_Pin_12,  toggle_value );
	while(1)
	{

		uint32 current_ts_microseconds = TICKS();

		if( ! transmit )
		{
			if( current_ts_microseconds - start_timestamp >= microseconds )
			{
				GPIO_OUTPUT(GPIO_Pin_12, 0 );
				break;
			}
		}
		else
			if( current_ts_microseconds - toggle_timestamp >= 13 )
			{
				if( current_ts_microseconds - start_timestamp >= microseconds )
				{
					GPIO_OUTPUT(GPIO_Pin_12, 0 );
					break;
				}

				toggle_value = ( toggle_value ? 0 : 1 );
				GPIO_OUTPUT(GPIO_Pin_12,  toggle_value);
				toggle_timestamp = current_ts_microseconds;
			}
	}
#else
	GPIO_OUTPUT(GPIO_Pin_12,   transmit );
	uint32 current_ts_microseconds;
	do{
		current_ts_microseconds = TICKS();
	}
	while( current_ts_microseconds - start_timestamp < microseconds );
#endif
}


void play_ir()
{
	if( m_num_transitions > 0 )
	{
		uint32 prev_transition = m_transitions[0].m_transition_timestamp;
		int transition;
		taskENTER_CRITICAL();
		for( transition = 1; transition < m_num_transitions; ++ transition )
		{
			delay_microseconds( m_transitions[transition].m_transition_timestamp - prev_transition , ! m_transitions[transition-1].m_transition_type );
			prev_transition = m_transitions[transition].m_transition_timestamp;
		}
		taskEXIT_CRITICAL();
	}
	GPIO_OUTPUT(GPIO_Pin_12, 0);
}


void taskInfrared(void *pvParameters)
{
	GPIO_ConfigTypeDef gpio_in_cfg5;
	GPIO_ConfigTypeDef gpio_in_cfg12;

	gpio_in_cfg5.GPIO_Pin  = GPIO_Pin_5;
	gpio_in_cfg5.GPIO_IntrType = GPIO_PIN_INTR_ANYEDGE;
	gpio_in_cfg5.GPIO_Mode = GPIO_Mode_Input;
	gpio_in_cfg5.GPIO_Pullup = GPIO_PullUp_DIS;
	gpio_config(&gpio_in_cfg5);
	gpio_intr_handler_register(intr_handler, NULL);


	gpio_in_cfg12.GPIO_Pin  = GPIO_Pin_12;
	gpio_in_cfg12.GPIO_IntrType = GPIO_PIN_INTR_DISABLE;
	gpio_in_cfg12.GPIO_Mode = GPIO_Mode_Output;
	gpio_in_cfg12.GPIO_Pullup = GPIO_PullUp_DIS;
	gpio_config(&gpio_in_cfg12);

	GPIO_OUTPUT(GPIO_Pin_12, 0 );

	GPIO_INTERRUPT_ENABLE;
	while (1) {
		int m_previous_transitions_received = m_num_transitions;
		vTaskDelay(200);    //Read every 200milli Sec
		if( m_num_transitions == MAX_TRANSITIONS  )
		{
			// too many samples, drop everything?
			os_printf("too many samples, drop \n");
			m_num_transitions = 0;
		}
		if( m_num_transitions == m_previous_transitions_received && m_num_transitions > 0 )
		{
			os_printf("received\n");
			gpio_pin_intr_state_set(GPIO_ID_PIN(5) , GPIO_PIN_INTR_DISABLE);
			if( m_num_transitions > 0 )
			{
				uint32 prev_transition = m_transitions[0].m_transition_timestamp;
				int transition;
				for( transition = 0; transition < m_num_transitions; ++ transition )
				{
					os_printf("transition %d duration=%d abs_timestamp=%d type: %d\n",
							transition,
							m_transitions[transition].m_transition_timestamp - prev_transition,
							m_transitions[transition].m_transition_timestamp - m_transitions[0].m_transition_timestamp,
							m_transitions[transition].m_transition_type);
					prev_transition = m_transitions[transition].m_transition_timestamp;
					vTaskDelay(1);
				}
			}

			play_ir();
			m_num_transitions = 0;
			gpio_pin_intr_state_set(GPIO_ID_PIN(5),GPIO_PIN_INTR_ANYEDGE);
		}
		else
		{
			os_printf("idle\n");
		}
	}

}




void infrared_gpio_task_start()
{
	xTaskCreate(taskInfrared,
				"task_infrared",
				512,                // Increase this parameter to 2048 byte if you wanna use SSL tests
				NULL,
				tskIDLE_PRIORITY,
				NULL);
}
