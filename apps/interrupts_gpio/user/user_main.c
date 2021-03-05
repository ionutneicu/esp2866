/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "esp_common.h"
#include "esp_system.h"
#include "uart.h"
#include "freertos/task.h"

#include "gpio.h"

#define GPIO_INPUT(gpio_bits)       ((gpio_input_get()&gpio_bits)?1:0)
#define GPIO_INTERRUPT_ENABLE 		 _xt_isr_unmask(1 << ETS_GPIO_INUM)
void gpio_config(GPIO_ConfigTypeDef *pGPIOConfig);


/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;
        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}
/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
extern int  uart_div_modify( unsigned int port, unsigned int divider);

typedef struct s_io_port_edge
{
	int 		m_transition_type;  //0 fall, 1 rise
	uint32		m_transition_timestamp;
	
} io_port_edge_t;

#define MAX_TRANSITIONS 1000
static volatile uint16  m_num_transitions = 0;
static volatile io_port_edge_t m_transitions[MAX_TRANSITIONS];
static volatile uint8  m_receive = 1;

#define GPIO_OUTPUT(gpio_bits, bit_value) \
    if(bit_value) gpio_output_conf(gpio_bits, 0, gpio_bits, 0);\
    else gpio_output_conf(0, gpio_bits, gpio_bits, 0)

static void intr_handler() {

    if( m_receive )
    {
    	 u32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	 gpio_pin_intr_state_set(GPIO_ID_PIN(5) , GPIO_PIN_INTR_DISABLE);
	 GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(5) );
    	 if( m_num_transitions < MAX_TRANSITIONS )
    	 {
	    m_transitions[m_num_transitions].m_transition_type = GPIO_INPUT(GPIO_Pin_5);
	    m_transitions[m_num_transitions].m_transition_timestamp = system_get_time();
	    ++ m_num_transitions;
    	 }
    	gpio_pin_intr_state_set(GPIO_ID_PIN(5) ,GPIO_PIN_INTR_ANYEDGE);
    	os_printf("interrupt %d\n", m_num_transitions);
    }
}

/*static void intr_handler() {

    u32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    gpio_pin_intr_state_set(GPIO_ID_PIN(5) , GPIO_PIN_INTR_DISABLE);
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(5) );
    gpio_pin_intr_state_set(GPIO_ID_PIN(5) ,GPIO_PIN_INTR_ANYEDGE);
    //os_printf("interrupt%d us: gpio_value=%d\n",system_get_time(),GPIO_INPUT(GPIO_Pin_5));
    os_printf("interrupt\n");
}*/

static void gpio_intr_handler() {
    uint32 gpio_mask = _xt_read_ints();
    uint32_t gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    os_printf("interrupt@%dus: mask=0x%02x, status=0x%02x\n",system_get_time(),gpio_mask,gpio_status);
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status );     //clear interrupt status
}

void taskMain(void *pvParameters)
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
    
    GPIO_INTERRUPT_ENABLE;

    GPIO_OUTPUT(GPIO_Pin_12, 0 );
    while (1) {
        vTaskDelay(200);    //Read every 200milli Sec
        os_printf("tick2\n");

    }
}

void delay_microseconds( uint32 microseconds, bool transmit )
{
	uint32 current_timestamp = system_get_time();
	uint32 toggle_timestamp = current_timestamp;
	uint8  toggle_value = 0;
	GPIO_OUTPUT(GPIO_Pin_12,  toggle_value );
	while( system_get_time() - current_timestamp < microseconds )
	{
		if( transmit )
		{
			uint32 current_ts_microseconds = system_get_time();	
			if( current_ts_microseconds - toggle_timestamp > 26 )
			{
				toggle_value = ( toggle_value ? 0 : 1 );
				GPIO_OUTPUT(GPIO_Pin_12,  toggle_value);
				toggle_timestamp = current_ts_microseconds;
			}
		}
	}
	GPIO_OUTPUT(GPIO_Pin_12, 0 );

}

void play_ir()
{
        if( m_num_transitions > 0 )
        {
          uint32 prev_transition = m_transitions[0].m_transition_timestamp;
          int transition;
          for( transition = 1; transition < m_num_transitions; ++ transition )
          {
		delay_microseconds( m_transitions[transition].m_transition_timestamp - prev_transition , ! m_transitions[transition-1].m_transition_type );
		prev_transition = m_transitions[transition].m_transition_timestamp;
          }
	}
}

void user_init(void)
{
    uart_div_modify(0, UART_CLK_FREQ / 115200);
    os_printf("SDK version:%s\n", system_get_sdk_version());
    os_printf("GPIO TEST");


    xTaskCreate(taskMain,
                "taskMain",
                512,                // Increase this parameter to 2048 byte if you wanna use SSL tests
                NULL,
                tskIDLE_PRIORITY,
                NULL);

    //Connect to TOUT Pin(Voltage Range between 0~1V)
    while (1) {
        vTaskDelay(500);    //Read every 200milli Sec
	os_printf("tick\n");
	gpio_pin_intr_state_set(GPIO_ID_PIN(5) , GPIO_PIN_INTR_DISABLE);
	m_receive = 0;
	if( m_num_transitions > 0 )
	{
		uint32 prev_transition = m_transitions[0].m_transition_timestamp;
		int transition;
		for( transition = 0; transition < m_num_transitions; ++ transition )
		{
			os_printf("transition %d duration=%d type: %d\n", transition, m_transitions[transition].m_transition_timestamp - prev_transition, m_transitions[transition].m_transition_type);
			prev_transition = m_transitions[transition].m_transition_timestamp;
		}
	}
	play_ir();
	m_num_transitions = 0;
	m_receive=1;
        gpio_pin_intr_state_set(GPIO_ID_PIN(5) ,GPIO_PIN_INTR_ANYEDGE);
    }
}

