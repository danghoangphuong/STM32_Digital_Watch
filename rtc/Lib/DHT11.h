#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f1xx.h"

typedef struct{
	uint8_t int_RH;
	uint8_t dec_RH;
	uint8_t int_Temp;
	uint8_t dec_Temp;
	uint8_t check_sum;
}DHT11_Typedef;
void DHT11_Delay_us(uint16_t us);
void DHT11_Delay_ms(uint16_t ms);
void DHT11_Set_output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Set_input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_send_start(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DHT11_check_response(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DHT11_read_byte(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DHT11_handle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, DHT11_Typedef *dht_value);
void DHT11_init(TIM_HandleTypeDef *htim);



#endif
