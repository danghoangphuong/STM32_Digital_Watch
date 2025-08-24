#include "DHT11.h"
#define DHT11_TIMEOUT_US 100
TIM_HandleTypeDef *htim;

void DHT11_Delay_us(uint16_t us)
{
//	htim->Instance->CNT = 0;
//	HAL_TIM_Base_Start(htim);
//	while(htim->Instance->CNT < us);
//	HAL_TIM_Base_Stop(htim);
	uint16_t start = __HAL_TIM_GET_COUNTER(htim);
    while(((uint16_t)(__HAL_TIM_GET_COUNTER(htim) - start)) < us);
}

void DHT11_Delay_ms(uint16_t ms)
{
	for(uint16_t i=0; i<ms; i++)
	{
		DHT11_Delay_us(1000);
	}
}

void DHT11_Set_output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Set_input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_send_start(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	DHT11_Set_output(GPIOx, GPIO_Pin);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);
	DHT11_Delay_ms(20);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
	DHT11_Delay_us(20);
	DHT11_Set_input(GPIOx, GPIO_Pin);
	//==============================
}

static int DHT11_wait_for_level(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState level, uint32_t timeout_us)
{
    uint32_t start = htim->Instance->CNT;
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == level)
    {
        if (((htim->Instance->CNT - start) & 0xFFFF) > timeout_us) {
            return -1; // timeout
        }
    }
    return 0; // ok
}

uint8_t DHT11_check_response(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	DHT11_Delay_us(40);
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == 0)
	{
		//DHT11_Delay_us(80);
		if (DHT11_wait_for_level(GPIOx, GPIO_Pin, GPIO_PIN_RESET, 100) < 0) return -1; // wait logic 1
        if (DHT11_wait_for_level(GPIOx, GPIO_Pin, GPIO_PIN_SET, 100) < 0) return -1;   // wait logic 0
        return 1; // OK
	}
	return -1;
}	

uint8_t DHT11_read_byte(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) //read each byte
{
	uint8_t value = 0;
	for(int i=0; i<8; i++)
	{
		if(DHT11_wait_for_level(GPIOx, GPIO_Pin, GPIO_PIN_RESET, DHT11_TIMEOUT_US) < 0) 
		{
			return 0xFF; // timeout
		}
		DHT11_Delay_us(40); 
		
		// After 40us, check if bit 0 -> receive bit 0 else bit 1
		value <<= 1;
		if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == 1) 
		{
            value |= 1;
        }

        if(DHT11_wait_for_level(GPIOx, GPIO_Pin, GPIO_PIN_SET, DHT11_TIMEOUT_US) < 0) return 0xFF; // timeout
    }
    return value;
}

uint8_t DHT11_handle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, DHT11_Typedef *dht_value)
{
	DHT11_send_start(GPIOx, GPIO_Pin);
	if(DHT11_check_response(GPIOx, GPIO_Pin) != 1) return -1;
	dht_value->int_RH = DHT11_read_byte(GPIOx, GPIO_Pin);
	dht_value->dec_RH = DHT11_read_byte(GPIOx, GPIO_Pin);
	dht_value->int_Temp = DHT11_read_byte(GPIOx, GPIO_Pin);
	dht_value->dec_Temp = DHT11_read_byte(GPIOx, GPIO_Pin);
	dht_value->check_sum = DHT11_read_byte(GPIOx, GPIO_Pin);
	if (dht_value->check_sum != (dht_value->int_RH + dht_value->dec_RH +
                     dht_value->int_Temp + dht_value->dec_Temp))
        return -2; // checksum error

    return 0; // success
}

void DHT11_init(TIM_HandleTypeDef *dht_htim)
{
	htim = dht_htim;
	htim->Init.Prescaler = 63;
	htim->Init.Period = 65535;
	HAL_TIM_Base_Init(htim);
    HAL_TIM_Base_Start(htim);
}

