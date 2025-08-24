/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId Temp_Humid_TaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void Start_Temp_Humid_Task(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
osThreadId Time_TaskHandle;
void Start_Time_Task(void const * argument);

osThreadId Button_TaskHandle;
void Start_Button_Task(void const * argument);

osMutexId i2cMutex_handle;

#include "ssd1306.h"
#include "fonts.h"
#include "Button.h"
#include "DHT11.h"
#include "stdio.h"
#include "stdarg.h"

#define RTC_ADD (0x68<<1)

uint8_t Dec_to_BCD(uint8_t num);
uint8_t BCD_to_Dec(uint8_t num);
void RTC_set_time_value(void);
void RTC_set_alarm_value(void);
void RTC_alarm_check(void);
void Delay_ms(uint16_t ms);
void Delay_us(uint16_t us);
void DHT_set_output(void);
void DHT_set_input(void);

const char dow_arr[][4] = {{"SUN"},{"MON"},{"TUE"},{"WED"},{"THU"},{"FRI"},{"SAT"}}; //day of week 
volatile uint8_t oled_clear_request = 0;

typedef struct{
	int8_t sec;
	int8_t min;
	int8_t hour;
	int8_t dow;
	int8_t date;
	int8_t month;
	int8_t year;
}Time_Data;

typedef enum{
	NORMAL_DISPLAY_STATE,
	CHANGE_HOUR_STATE,
	CHANGE_MIN_STATE,
	CHANGE_DATE_STATE,
	CHANGE_MONTH_STATE,
	CHANGE_YEAR_STATE,
	CHANGE_ALARM_HOUR_STATE,
	CHANGE_ALARM_MIN_STATE,
	//SET_ALARM_STATE,
	//NOT_SET_ALARM_STATE
}RTC_STATE;
RTC_STATE clock_state = NORMAL_DISPLAY_STATE;

Time_Data data_set;

Button_TypeDef btn_function;
Button_TypeDef btn_up;
Button_TypeDef btn_down;
Button_TypeDef btn_confirm;

DHT11_Typedef dht11_val;

int8_t alarm_hour = 1;
int8_t alarm_min;
char buff_uart[40];

uint8_t response;
//uint8_t is_alarm = 0; // alarm not set
//========================================GET TIME AND WRITE TIME=========================
void RTC_write(Time_Data *data)
{
	uint8_t buff_data[8];
	buff_data[0] = 0x00; //start reg address
	buff_data[1] = Dec_to_BCD(data->sec);
	buff_data[2] = Dec_to_BCD(data->min);
	buff_data[3] = Dec_to_BCD(data->hour);
	buff_data[4] = Dec_to_BCD(data->dow);
	buff_data[5] = Dec_to_BCD(data->date);
	buff_data[6] = Dec_to_BCD(data->month);
	buff_data[7] = Dec_to_BCD(data->year);
	HAL_I2C_Master_Transmit(&hi2c2, RTC_ADD, buff_data, 8, 100);
}

void RTC_read(Time_Data *data)
{
	uint8_t buff_data[7];
	uint8_t reg_address = 0;
	HAL_I2C_Master_Transmit(&hi2c2, RTC_ADD, &reg_address, 1, 100);
	HAL_I2C_Master_Receive(&hi2c2, RTC_ADD, buff_data, 7, 100);
	data->sec = BCD_to_Dec(buff_data[0]);
	data->min = BCD_to_Dec(buff_data[1]);
	data->hour = BCD_to_Dec(buff_data[2]);
	data->dow = BCD_to_Dec(buff_data[3]);
	data->date = BCD_to_Dec(buff_data[4]);
	data->month = BCD_to_Dec(buff_data[5]);
	data->year = BCD_to_Dec(buff_data[6]);
}

uint8_t Dec_to_BCD(uint8_t num)
{
	return (num/10)<<4|(num%10);
}

uint8_t BCD_to_Dec(uint8_t num)
{
	return (num>>4)*10+(num&0x0F);
}
//============================================date, month, year handle============================
uint8_t RTC_get_day_of_week(Time_Data *data)
{
	uint16_t date = data->date;
	uint16_t month = data->month;
	uint16_t year = 2000 + data->year;
	uint16_t dow = (date += month < 3 ? year-- : year-2, 23*month/9 + date + 4 + year/4 - year/100 + year/400)%7;
	return dow;
}

uint8_t RTC_check_leap_year(uint8_t year)
{
	if((year % 4) == 0 & (year % 100) != 0 || (year % 400) == 0)
	{
		return 1;
	}
	return 0;
}

uint8_t RTC_get_day_of_month(uint8_t month, uint8_t year)
{
	switch(month)
	{
		case 1: case 3: case 5: case 7: case 8: case 10: case 12:
			return 31;
			break;
		case 4: case 6: case 9: case 11:
			return 30;
			break;
		case 2:
			if(RTC_check_leap_year(year))
			{
				return 29;
			}
	}
	return 28;
}

//==================================Main screen handle==================================================
void RTC_display()
{
	uint8_t buff_hour[16];
	uint8_t buff_date[16];
	uint8_t buff_temp[16];
	uint8_t buff_humid[16];
	uint8_t buff_alarm_status[16];
	uint8_t dow = RTC_get_day_of_week(&data_set);
	
	sprintf((char*)buff_hour, "%02d:%02d:%02d", data_set.hour, data_set.min, data_set.sec);
	sprintf((char*)buff_date, "%s  %02d/%02d/20%02d", dow_arr[dow], data_set.date, 
													data_set.month, data_set.year);
	sprintf((char*)buff_temp, "Temp: %d.%d*C", dht11_val.int_Temp, dht11_val.dec_Temp);
	sprintf((char*)buff_humid, "Humid: %d%s", dht11_val.int_RH, "%RH");
	
	
	static uint32_t time_read = 0;
	if(HAL_GetTick() - time_read >= 1000)
	{
		osMutexWait(i2cMutex_handle, osWaitForever);
		//=======Display time=======================
		RTC_read(&data_set);
		SSD1306_GotoXY(10,0);
		SSD1306_Puts((char*)buff_hour, &Font_7x10, 1);
		SSD1306_GotoXY(10,15); //col; row
		SSD1306_Puts((char*)buff_date, &Font_7x10, 1);
		//=======Display temp and humid============
		SSD1306_GotoXY(10,30);
		SSD1306_Puts((char*)buff_temp, &Font_7x10, 1);
		SSD1306_GotoXY(10,45);
		SSD1306_Puts((char*)buff_humid, &Font_7x10, 1);
		
		SSD1306_UpdateScreen();
		osMutexRelease(i2cMutex_handle);
		time_read = HAL_GetTick();
	}
	
}

void RTC_handle()
{
	switch(clock_state)
	{
		case NORMAL_DISPLAY_STATE:
			RTC_display();
			RTC_alarm_check();
			break;
		case CHANGE_HOUR_STATE:
		case CHANGE_MIN_STATE:
		case CHANGE_DATE_STATE:
		case CHANGE_MONTH_STATE:
		case CHANGE_YEAR_STATE:
			RTC_set_time_value();
			break;
		case CHANGE_ALARM_HOUR_STATE:
		case CHANGE_ALARM_MIN_STATE:	
			RTC_set_alarm_value();
			break;
		default: break;
	}
} 
//===============================Setting main screen RTC params===================
void RTC_set_time_value()
{	
	static uint32_t time = 0;
	static uint8_t is_showing = 1;
	char buff_line1[16];
	char buff_line2[16];
	
	if(HAL_GetTick() - time > 300)
	{
		osMutexWait(i2cMutex_handle, osWaitForever);
		is_showing = !is_showing;
		RTC_read(&data_set);
		uint8_t dow = RTC_get_day_of_week(&data_set);
		sprintf((char*)buff_line1, "%02d:%02d:%02d", data_set.hour, data_set.min, data_set.sec);
		sprintf((char*)buff_line2, "%s  %02d/%02d/20%02d", dow_arr[dow], data_set.date, 
														   data_set.month, data_set.year);
		if(!is_showing)
		{
			switch(clock_state)
			{
				case CHANGE_HOUR_STATE:
					buff_line1[0] = buff_line1[1] = ' ';
					break;
				case CHANGE_MIN_STATE:
					buff_line1[3] = buff_line1[4] = ' ';
					break;
				case CHANGE_DATE_STATE:
					buff_line2[5] = buff_line2[6] = ' ';
					break;
				case CHANGE_MONTH_STATE:
					buff_line2[8] = buff_line2[9] = ' ';
					break;
				case CHANGE_YEAR_STATE:
					buff_line2[11] = buff_line2[12] = buff_line2[13] = buff_line2[14] = ' ';
					break;
			}
		}
		SSD1306_GotoXY(10,0);
		SSD1306_Puts((char*)buff_line1, &Font_7x10, 1);
		
		SSD1306_GotoXY(10,15); //col; row
		SSD1306_Puts((char*)buff_line2, &Font_7x10, 1);
		
		SSD1306_UpdateScreen();
		osMutexRelease(i2cMutex_handle);
		time = HAL_GetTick();
	}
}


//==================================ALARM HANDLE===================================
uint8_t alarm_status;
void RTC_alarm_check()
{
	if(alarm_hour == data_set.hour && alarm_min == data_set.min)
	{
		static uint32_t time;
		if(HAL_GetTick() - time > 300)
		{
			alarm_status = !alarm_status;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, alarm_status);
			time = HAL_GetTick();
		}
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

}

void RTC_set_alarm_value()
{
	static uint32_t time = 0;
	static uint8_t is_showing = 1;
	char buff_alarm_time[16];
	
	if(HAL_GetTick() - time > 300)
	{
		osMutexWait(i2cMutex_handle, osWaitForever);
		is_showing = !is_showing;
		RTC_read(&data_set);
		sprintf(buff_alarm_time, "%02d : %02d", alarm_hour, alarm_min);
		if(!is_showing)
		{
			switch(clock_state)
			{
				case CHANGE_ALARM_HOUR_STATE:
					buff_alarm_time[0] = buff_alarm_time[1] = ' ';
					break;
				case CHANGE_ALARM_MIN_STATE:
					buff_alarm_time[5] = buff_alarm_time[6] = ' ';
					break;
				default:
					break;
			}
		}
		SSD1306_GotoXY(0,0);
		SSD1306_Puts("ALARM CLOCK", &Font_11x18, 1);
		
		SSD1306_GotoXY(10,30); //col; row
		SSD1306_Puts(buff_alarm_time, &Font_11x18, 1);
		
		SSD1306_UpdateScreen();
		osMutexRelease(i2cMutex_handle);
		time = HAL_GetTick();
	}
}

//============================CHANGE CLOCK VALUE=======================
void RTC_change_val(int8_t *clock_val, int8_t low_limit, int8_t up_limit, int8_t plus)
{
	if(plus)
	{
		(*clock_val)++;
		if(*clock_val > up_limit)
		{
				*clock_val = low_limit;
		}
	}
	else
	{
		(*clock_val)--;
		if(*clock_val < low_limit)
		{
				*clock_val = up_limit;
		}
	}
}

void RTC_value_button_handle(int8_t plus)
{
switch(clock_state)
		{
			case CHANGE_HOUR_STATE:
			{
				osMutexWait(i2cMutex_handle, osWaitForever);
				RTC_change_val(&data_set.hour,0,23,plus);
				RTC_write(&data_set);
				osMutexRelease(i2cMutex_handle);
				break;
			}
			case CHANGE_MIN_STATE:
			{
				osMutexWait(i2cMutex_handle, osWaitForever);
				RTC_change_val(&data_set.min,0,59,plus);
				RTC_write(&data_set);
				osMutexRelease(i2cMutex_handle);
				break;
			}
			case CHANGE_DATE_STATE:
			{
				osMutexWait(i2cMutex_handle, osWaitForever);
				uint8_t day_in_mon = RTC_get_day_of_month(data_set.month, data_set.year);
				RTC_change_val(&data_set.date,1,day_in_mon,plus);
				RTC_write(&data_set);
				osMutexRelease(i2cMutex_handle);
				break;
			}
			case CHANGE_MONTH_STATE:
			{
				osMutexWait(i2cMutex_handle, osWaitForever);
				RTC_change_val(&data_set.month,1,12,plus);
				RTC_write(&data_set);
				osMutexRelease(i2cMutex_handle);
				break;
			}
			case CHANGE_YEAR_STATE:
			{	
				osMutexWait(i2cMutex_handle, osWaitForever);
				RTC_change_val(&data_set.year,0,99,plus);
				RTC_write(&data_set);
				osMutexRelease(i2cMutex_handle);
				break;
			}
			case CHANGE_ALARM_HOUR_STATE:
			{
				RTC_change_val(&alarm_hour,0,23,plus);
				break;
			}
			case CHANGE_ALARM_MIN_STATE:
			{
				RTC_change_val(&alarm_min,0,59,plus);
				break;
			}
			default:
				break;
		}
}
//============================BUTTON HANDLE===================
void btnpress_callback(Button_TypeDef *Buttonx)
{
	if(Buttonx == &btn_up)
	{
		RTC_value_button_handle(1);
	}
	else if(Buttonx == &btn_down)
	{
		RTC_value_button_handle(0);
	}
}


void short_press_callback(Button_TypeDef *Buttonx)
{
	if(Buttonx == &btn_function)
	{
		switch(clock_state)
		{
			case NORMAL_DISPLAY_STATE:
				clock_state = CHANGE_HOUR_STATE;
				break;
			case CHANGE_HOUR_STATE:
				clock_state = CHANGE_MIN_STATE;
				break;
			case CHANGE_MIN_STATE:
				clock_state = CHANGE_DATE_STATE;
				break;
			case CHANGE_DATE_STATE:
				clock_state = CHANGE_MONTH_STATE;
				break;
			case CHANGE_MONTH_STATE:
				clock_state = CHANGE_YEAR_STATE;
				break;
			case CHANGE_YEAR_STATE:
				clock_state = NORMAL_DISPLAY_STATE;
				break;
			//=======Temporary====================
			case CHANGE_ALARM_HOUR_STATE:
				clock_state = CHANGE_ALARM_MIN_STATE;
				break;
			case CHANGE_ALARM_MIN_STATE:
				osMutexWait(i2cMutex_handle, osWaitForever);
				SSD1306_Clear();
				osMutexWait(i2cMutex_handle, osWaitForever);
				clock_state = NORMAL_DISPLAY_STATE;
				break;
			default:
				break;
		}
	}
}


void long_press_callback(Button_TypeDef *Buttonx)
{	
	if(Buttonx == &btn_function)
	{
		clock_state = CHANGE_ALARM_HOUR_STATE;
		oled_clear_request = 1;
//		SSD1306_Clear();
//		SSD1306_GotoXY(0,0);
//		SSD1306_Puts("ALARM CLOCK", &Font_11x18, 1);
//		SSD1306_UpdateScreen();
	}		
}
//==============================UART PRINT==============================
void Print_uart(const char*format,...)
{
	for(uint8_t i=0; i<40; i++)
	{
		buff_uart[i] = 0;
	}
	va_list args;
	va_start(args, format);
	vsnprintf(buff_uart, sizeof(buff_uart), format, args);
	va_end(args);
	HAL_UART_Transmit(&huart1, (uint8_t*)buff_uart, 40, 1000);
}

void I2C_Scan(I2C_HandleTypeDef *hi2c) {
    char msg[64];
    for (uint8_t i = 1; i < 127; i++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i<<1), 1, 10) == HAL_OK) {
            sprintf(msg, "I2C device found at 0x%X\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
	DHT11_init(&htim2);
	//SSD1306_Init();
	Button_init(&btn_function, GPIOB, GPIO_PIN_13);
	Button_init(&btn_up, GPIOB, GPIO_PIN_14);
	Button_init(&btn_down, GPIOB, GPIO_PIN_15);
	Button_init(&btn_confirm, GPIOA, GPIO_PIN_3);
//  data_set.sec = 30;
//  data_set.min = 56;
//  data_set.hour = 16;
//  data_set.date = 12;
//  data_set.month = 8;
//  data_set.year = 25;
//  RTC_write(&data_set);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  osMutexDef(i2c_Mutex);
  i2cMutex_handle = osMutexCreate(osMutex(i2c_Mutex));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Temp_Humid_Task */
  osThreadDef(Temp_Humid_Task, Start_Temp_Humid_Task, osPriorityNormal, 0, 128);
  Temp_Humid_TaskHandle = osThreadCreate(osThread(Temp_Humid_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(Time_Task, Start_Time_Task, 1, 0, 128);
  Time_TaskHandle = osThreadCreate(osThread(Time_Task), NULL);
  
  osThreadDef(Button_Task, Start_Button_Task, 1, 0, 128);
  Button_TaskHandle = osThreadCreate(osThread(Button_Task), NULL);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Start_Time_Task(void const * argument)
{
	for(;;)
	{
		if(oled_clear_request == 1)
		{
			osMutexWait(i2cMutex_handle, osWaitForever);
			SSD1306_Clear();
			SSD1306_UpdateScreen();
			osMutexRelease(i2cMutex_handle);
			
			oled_clear_request = 0;
		}
		RTC_handle();
		I2C_Scan(&hi2c1);
		osDelay(1000);
		
	} 
}

void Start_Button_Task(void const * argument)
{
	for(;;)
	{
		Btn_handle(&btn_function);	
		Btn_handle(&btn_down);	
		Btn_handle(&btn_up);
		Btn_handle(&btn_confirm);
		osDelay(20);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_Temp_Humid_Task */
/**
  * @brief  Function implementing the Temp_Humid_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Temp_Humid_Task */
void Start_Temp_Humid_Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	char msg[] = "DHT11 Task running...\r\n";
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

	uint8_t buff_temp[16];
	uint8_t buff_humid[16];
	if(DHT11_handle(GPIOB, GPIO_PIN_0, &dht11_val)==0)
	{
		sprintf((char*)buff_temp, "Temp: %d.%d*C\n", dht11_val.int_Temp, dht11_val.dec_Temp);
		sprintf((char*)buff_humid, "Humid: %d%s\n", dht11_val.int_RH, "%RH");
		Print_uart((const char*)buff_temp);
		Print_uart((const char*)buff_humid);
	}
	else
	{
		Print_uart("Read temp, humid error!");
	}
	
    osDelay(2000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
