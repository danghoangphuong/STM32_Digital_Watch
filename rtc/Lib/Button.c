#include "Button.h"

__weak void btnpress_callback(Button_TypeDef *Buttonx) //callback when press
{

}

__weak void short_press_callback(Button_TypeDef *Buttonx) // callback when press and release
{
	
}	

__weak void long_press_callback(Button_TypeDef *Buttonx) // callback when long press
{

}

__weak void btn_release_callback()
{

}

void Btn_handle(Button_TypeDef *Buttonx)
{
	//=======================noise handle=============================
	uint8_t sta = HAL_GPIO_ReadPin(Buttonx->GPIOx, Buttonx->GPIO_Pin); //current_stt == 1 = init_status
	if(sta != Buttonx->btn_filter) //push current_stt == 0 != filter_stt
	{
		Buttonx->btn_filter = sta; // init_stt = 0
		Buttonx->is_debouncing = 1;
		Buttonx->time_debounce = HAL_GetTick(); // get time
	}
	
	if(Buttonx->is_debouncing && (HAL_GetTick() - Buttonx->time_debounce >= 15)) //if stt change & current time - last time >=15
	{
		Buttonx->btn_current = Buttonx->btn_filter;
		Buttonx->is_debouncing = 0; 
	}
	
	//==============================push/ release handle====================
	if(Buttonx->btn_current != Buttonx->btn_last)
	{
		  if(Buttonx->btn_current == 0)
		  {
				Buttonx->is_press_timeout = 1;
				btnpress_callback(Buttonx);
				Buttonx->time_start_press = HAL_GetTick();
		  }
		  else
		  {
			if(HAL_GetTick() - Buttonx->time_start_press <= 1000)
			{
				short_press_callback(Buttonx);
			}	
			
			btn_release_callback();
			Buttonx->is_press_timeout = 0;
		  }
		  Buttonx->btn_last = Buttonx->btn_current;
	}
	
	if(Buttonx->is_press_timeout && (HAL_GetTick() - Buttonx->time_start_press >= 3000))
	{
		long_press_callback(Buttonx);
		Buttonx->is_press_timeout = 0;
	}
}

void Button_init(Button_TypeDef *Buttonx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	Buttonx->btn_current = 1;
	Buttonx->btn_filter = 1;
	Buttonx->btn_last = 1;
	Buttonx->GPIOx = GPIOx;
	Buttonx->GPIO_Pin = GPIO_Pin;
}

