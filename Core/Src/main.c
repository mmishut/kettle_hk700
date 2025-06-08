/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

EXTI_HandleTypeDef hexti;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

typedef struct {
	GPIO_TypeDef* port;
	int pin;
} GPIO_t;

// Temperature curve  19 .. 100 degrees celcius.
int T19_100[] = {
	 220,   // 19 C
	 232,  246,  257,  270,  280,  290,  305,  320,  340,  355,
	 370,  385,  400,  420,  440,  460,  475,  495,	 515,  530,
	 545,  575,  600,  620,  640,  660,  685,  710,  735,  775,
	 800,  830,  850,  880,  905,  935,  970, 1000, 1025, 1050,
	1080, 1120, 1140, 1172, 1188, 1220, 1265, 1300, 1320, 1360,
	1390, 1415, 1455, 1489, 1518, 1530,	1564, 1605, 1632, 1668,
	1675, 1708, 1741, 1774, 1807, 1840, 1873, 1906, 1939, 1972,
	2005, 2038, 2071, 2104, 2137, 2170, 2203, 2236, 2269, 2302,
	2335   // 100 C
};


// LED matrix
/*
 *
 *       |    A3       |   A1       |   A0       |   A7                  |
 * ------+-------------+------------+------------+-----------------------+
 *   A2  |  LED1r [bSr]| LED4 [t80] | LED7  [5m] |  LED11  [bStartBrew]  |
 * ------+-------------+------------+------------+-----------------------+
 *   F1  |  LED1w [bSw]| LED5 [t70] | LED8  [4m] |  LED12  [bSelT]       |
 * ------+-------------+------------+------------+-----------------------+
 *   F0  |  LED2 [t100]| LED6 [t60] | LED9  [3m] |                       |
 * ------+-------------+------------+------------+-----------------------+
 *   C15 |  LED3 [8m]  | LED15 [t90]| LED10 [2m] |  LED14  [bSelM]       |
 * ------+-------------+------------+------------+-----------------------+
 */

#define _bStartR     0     // LED1r
#define _t80_C       1     // LED4
#define _Brew_5m     2     // LED7
#define _bStartBrew  3     // LED11

#define _bStartW     4     // LED1w
#define _t70_C       5     // LED5
#define _Brew_4m     6     // LED8
#define _bSelectT    7     // LED12

#define _t100_C      8     // LED2
#define _t60_C       9     // LED6
#define _Brew_3m     10    // LED9

#define _Brew_8m     12    // LED3
#define _t90_C       13    // LED15
#define _Brew_2m     14    // LED10
#define _bSelectM    15    // LED14

GPIO_t LED_ROW_PINS[4] = {
		{LED_N_1r_4_7_11_GPIO_Port , LED_N_1r_4_7_11_Pin},
		{LED_N_1w_5_8_12_GPIO_Port, LED_N_1w_5_8_12_Pin},
		{LED_N_2_6_9_GPIO_Port, LED_N_2_6_9_Pin},
		{LED_N_3_10_14_15_GPIO_Port, LED_N_3_10_14_15_Pin}
};

GPIO_t LED_COL_PINS[4] = {
		{LED_P_1r_1w_2_3_GPIO_Port, LED_P_1r_1w_2_3_Pin},
	    {LED_P_4_5_6_15_GPIO_Port ,  LED_P_4_5_6_15_Pin},
	    {LED_P_7_8_9_10_GPIO_Port ,  LED_P_7_8_9_10_Pin},
	    {LED_P_11_12_14_GPIO_Port ,  LED_P_11_12_14_Pin}
};

int LED_state[4*4]; // LED matrix

int LED_Temp[5]     = {_t60_C, _t70_C, _t80_C, _t90_C, _t100_C};
int LED_BrewTime[5] = {_Brew_2m, _Brew_3m, _Brew_4m, _Brew_5m, _Brew_8m};

void ScanLED()
{
	static int col = 0;

	// light off LED column
	HAL_GPIO_WritePin(LED_COL_PINS[col].port, LED_COL_PINS[col].pin, GPIO_PIN_RESET);

	// Advance to next column
	if (++col >= 4) col = 0;

	// Update row values
	for (int row = 0; row < 4; row++) {
		HAL_GPIO_WritePin(
				LED_ROW_PINS[row].port, LED_ROW_PINS[row].pin,
				(LED_state[row * 4 + col] > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET
		);
	}

	// light on LED column
	HAL_GPIO_WritePin(LED_COL_PINS[col].port, LED_COL_PINS[col].pin, GPIO_PIN_SET);
}

void SwitchLED_ON(int i) {
	if (LED_state[i] == 0) {
		LED_state[i] = 1;
	}
}

void SwitchLED_OFF(int i) {
	if (LED_state[i] == 1) {
		LED_state[i] = 0;
	}
}

int volatile DebounceLockCntr = 0;
int volatile ButtonEvent = 0;
int volatile ButtonEventPre = 0;

void DebounceBtn()
{
	static int DebounceCntr = 0;
	if (ButtonEventPre == 1) {
		if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET) DebounceCntr++;
	}
	if (ButtonEventPre == 2) {
		if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET) DebounceCntr++;
	}
	if (ButtonEventPre == 3) {
		if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET) DebounceCntr++;
	}
	if (ButtonEventPre == 4) {
		if (HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) == GPIO_PIN_RESET) DebounceCntr++;
	}

	if (DebounceLockCntr <= 0) {
		HAL_TIM_Base_Stop_IT(&htim14);
		if (DebounceCntr > 20) {
			ButtonEvent = ButtonEventPre;
		} else {
			ButtonEvent = 0;
		}
		DebounceCntr = 0;
		ButtonEventPre = 0;
	}
	DebounceLockCntr--;
}

#define WIN_SIZE 64

int Temp_idx = 4;
int Heat_state = 0;

#define targetT_100 4
#define targetT_90 3
#define targetT_80 2
#define targetT_70 1
#define targetT_60 0

#define Tmode_change 0
#define Tmode_show 1

int Tmode_change_timer = 0;

int targetT = 4;

uint32_t __Tadc_uint_summ = 0;

float __Tadc = 0;
float __Tadc_prev = 0;
float __Tadc_diff = 0;
float __tC = 0;

// additional control variables
#define POWER_W              2000.0f
#define SPECIFIC_HEAT_WATER  4180.0f
#define TICK_SEC             0.016f
#define PLATE_HEAT_FRAC      0.1f
#define MASS_EST_TICKS       300

float water_mass = 1.0f;      // estimated water mass in kg
float prev_water_mass = 1.0f;
float plate_heat = 0.0f;      // estimated heat energy in heating plate
float tC_start = 0.0f;
float tC_prev = 0.0f;
int heating_ticks = 0;
int water_changed = 0;
float predicted_time_s = 0.0f;


int BrewTime_idx = -1;

//  Polynom coeffs
// -5.02524268e-12  x^4
//  3.13324193e-08  x^3
// -7.11481304e-05  x^2
//  1.02027224e-01  x^1
//  3.08967137e-01  x^0

float Tadc_to_C(float __tadc) {
#define C4 -5.02524268e-12
#define C3  3.13324193e-08
#define C2 -7.11481304e-05
#define C1  1.02027224e-01
#define C0  3.08967137e-01
	float tRes = C4 * __tadc;
	tRes = (tRes + C3) * __tadc;
	tRes = (tRes + C2) * __tadc;
	tRes = (tRes + C1) * __tadc;
	return  tRes + C0;
}

unsigned int tidx = 0;
int audio_clks = 0;

#if PWM_BEEP

#define PWM_SINE_LENGTH 64
int pwm_sine[PWM_SINE_LENGTH];
#define AUDIO_MAGNITUDE 64

void PwmSineGen() {
    float pi = acosf(-1);
	for (int t = 0; t < PWM_SINE_LENGTH; t++) {
		pwm_sine[t] = (int)((AUDIO_MAGNITUDE-2) * (1.0f + sinf(2.0f*pi*(float)t/(float)PWM_SINE_LENGTH)) / 2.0f) + 1;
	}
}

void BeepClk() {
	static int buzzer_val = 0;
	buzzer_val = (buzzer_val + 1) % PWM_SINE_LENGTH;
	TIM3->CCR2 = pwm_sine[buzzer_val];
	if (audio_clks-- <= 0) {
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		HAL_TIM_Base_Stop_IT(&htim15);
	}
}

#else

void BeepStart(int beep_len) {
    audio_clks = beep_len;
    HAL_TIM_Base_Start_IT(&htim15);
}

void BeepStop() {
	HAL_TIM_Base_Stop_IT(&htim15);
}

void BeepClk() {
	static int buzzer_val = 0;
	buzzer_val ^= 1;
	HAL_GPIO_WritePin(BUZZ_PIN_GPIO_Port, BUZZ_PIN_Pin, buzzer_val);
	if (audio_clks-- <= 0) {
		BeepStop();
	}
}

#endif

int _min(int a, int b) {
	return (a < b) ? a : b;
}

int _max(int a, int b) {
	return (a > b) ? a : b;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim == &htim16)
	{
		static int measure_idx = 0;
		uint32_t __Tadc_uint_curr;

        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc, 1);

        __Tadc_uint_curr = HAL_ADC_GetValue(&hadc);
        __Tadc_uint_summ += __Tadc_uint_curr;

        measure_idx++;

        if (measure_idx == WIN_SIZE)
        {
        	measure_idx = 0;
        	__Tadc = (float)__Tadc_uint_summ/(float)WIN_SIZE;
        	__tC = Tadc_to_C(__Tadc);
        	__Tadc_diff = __Tadc - __Tadc_prev;
        	__Tadc_prev = __Tadc;
                __Tadc_uint_summ = 0;

                float tC_diff = __tC - tC_prev;
                if (Heat_state == 0 && fabsf(tC_diff) > 3.0f) {
                        water_changed = 1;
                }
                tC_prev = __tC;

                if (Heat_state != 0) {
                        heating_ticks++;
                        if (heating_ticks == 1) {
                                tC_start = __tC;
                                plate_heat = 0.0f;
                        }
                        if (heating_ticks == MASS_EST_TICKS) {
                                float dt = (float)MASS_EST_TICKS * TICK_SEC;
                                float dT = __tC - tC_start;
                                if (dT > 0.5f) {
                                        prev_water_mass = water_mass;
                                        water_mass = (POWER_W * dt) / (SPECIFIC_HEAT_WATER * dT);
                                        if (fabsf(water_mass - prev_water_mass) > 0.1f) {
                                                water_changed = 1;
                                        }
                                }
                        }
                        plate_heat += POWER_W * PLATE_HEAT_FRAC * TICK_SEC;
                        if (plate_heat > POWER_W * 10.0f) {
                                plate_heat = POWER_W * 10.0f;
                        }

                        float target_temp = 60.0f + (float)targetT * 10.0f;
                        float predicted_final = __tC + plate_heat/(water_mass*SPECIFIC_HEAT_WATER);
                        float heating_rate = POWER_W / (water_mass*SPECIFIC_HEAT_WATER);
                        predicted_time_s = (target_temp - predicted_final) / heating_rate;
                        if (predicted_time_s <= 0.0f) {
                                ButtonEvent = 1;
                        }
                } else {
                        heating_ticks = 0;
                        plate_heat *= 0.98f;
                }

            SwitchLED_OFF(LED_Temp[Temp_idx]);

            if (Tmode_change_timer > 0) {
            	Tmode_change_timer--;
            	Temp_idx = targetT;
            	if (Tmode_change_timer == 0) {
          		  SwitchLED_OFF(_bSelectT);
                  Temp_idx = _min(_max(((int)__tC - 55) / 10, 0), 4);
            	}
            } else {
                Temp_idx = _min(_max(((int)__tC - 55) / 10, 0), 4);
            }

            SwitchLED_ON(LED_Temp[Temp_idx]);

			if (Heat_state != 0)
			{
				HAL_GPIO_WritePin(PWR220_GPIO_Port, PWR220_Pin, GPIO_PIN_SET);

				// c1 = min(abs((2350 - T1)./D1), 40);
				if (targetT == targetT_100) {
		        	float C = fabs((2350.0f - __Tadc)/__Tadc_diff);
					if (__Tadc >= 1000.0f && C < 20.0f) // Prediction criterion
					{
						BeepStart(2000);
						ButtonEvent = 1;
					}

				}
				if (targetT == targetT_90) {
		        	float C = fabs((2005.0f - __Tadc)/__Tadc_diff);
					if (__Tadc >= 1000.0f && C < 20.0f) // Prediction criterion
					{
						ButtonEvent = 1;
					}
				}
				if (targetT == targetT_80) {
		        	float C = fabs((1675.0f - __Tadc)/__Tadc_diff);
					if (__Tadc >= 1000.0f && C < 20.0f) // Prediction criterion
					{
						ButtonEvent = 1;
					}
				}
				if (targetT == targetT_70) {
		        	float C = fabs((1390.0f - __Tadc)/__Tadc_diff);
					if (__Tadc >= 700.0f && C < 20.0f) // Prediction criterion
					{
						ButtonEvent = 1;
					}
				}
				if (targetT == targetT_60) {
		        	float C = fabs((1080.0f - __Tadc)/__Tadc_diff);
					if (__Tadc >= 500.0f && C < 20.0f) // Prediction criterion
					{
						ButtonEvent = 1;
					}
				}
				if (__Tadc >= 2300.0f) // Hard Stop criterion
				{
					ButtonEvent = 1;
				}
			}
			else
			{
				HAL_GPIO_WritePin(PWR220_GPIO_Port, PWR220_Pin, GPIO_PIN_RESET);
			}

			//printf("%.8i: %7.2f: %7.2f: %7.3f:\n\r", tidx++, __Tadc, __tC , C);
        }
	}
	if (htim == &htim6) {
		ScanLED();
	}
	if (htim == &htim14) {
		DebounceBtn();
	}
	if (htim == &htim15) {
		BeepClk();
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (DebounceLockCntr <= 0) {
		HAL_TIM_Base_Start_IT(&htim14);
		DebounceLockCntr = 50;
		if (GPIO_Pin == SW1_Pin) ButtonEventPre = 1;
		if (GPIO_Pin == SW2_Pin) ButtonEventPre = 2;
		if (GPIO_Pin == SW3_Pin) ButtonEventPre = 3;
		if (GPIO_Pin == SW4_Pin) ButtonEventPre = 4;
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
  MX_ADC_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  SwitchLED_ON(_bStartW);
  SwitchLED_ON(LED_Temp[Temp_idx]);
  //SwitchLED_ON(LED_Temp[Temp_idx]);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (ButtonEvent == 1) { // Push heating button
		  ButtonEvent = 0;
		  Heat_state = !Heat_state;
		  SwitchLED_OFF(_bStartR);
		  SwitchLED_OFF(_bStartW);
                  if (Heat_state != 0) {
                          SwitchLED_ON(_bStartR);
                          tidx = 0;

                          heating_ticks = 0;
                          plate_heat = 0.0f;
                          tC_start = __tC;
                          tC_prev = __tC;
                          water_changed = 0;

                          SwitchLED_ON(_bSelectT);
			  SwitchLED_OFF(LED_Temp[Temp_idx]);
			  Tmode_change_timer = 5;
			  Temp_idx = targetT;
			  SwitchLED_ON(LED_Temp[Temp_idx]);
			  BeepStart(1000);

			  //printf("%.8i: %7.2f: START HEATING\n\r", tidx, __Tadc);
		  }
                  if (Heat_state == 0) {
                          SwitchLED_ON(_bStartW);
                          heating_ticks = 0;
                          predicted_time_s = 0.0f;
                          //printf("%.8i: %7.2f: STOP HEATING\n\r", tidx, __Tadc);
                  }
	  }
	  if (ButtonEvent == 2) {
		  ButtonEvent = 0;
	  }
	  if (ButtonEvent == 3) { // Push temperature select button
		  ButtonEvent = 0;
		  if (Tmode_change_timer > 0) {
			  if (++targetT == 5) targetT = 0;
		  }

		  SwitchLED_ON(_bSelectT);
		  SwitchLED_OFF(LED_Temp[Temp_idx]);
		  Tmode_change_timer = 5;
		  Temp_idx = targetT;
		  SwitchLED_ON(LED_Temp[Temp_idx]);
		  BeepStart(300);

	  }
	  if (ButtonEvent == 4) { // Push brew time select button ?
		  ButtonEvent = 0;
		  if (BrewTime_idx >= 0)
			  SwitchLED_OFF(LED_BrewTime[BrewTime_idx]);
		  if (++BrewTime_idx == 5) BrewTime_idx = -1;
		  if (BrewTime_idx >= 0)
			  SwitchLED_ON(LED_BrewTime[BrewTime_idx]);
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 48-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 200;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1000;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 6;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 15000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 50;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_N_3_10_14_15_GPIO_Port, LED_N_3_10_14_15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED_N_2_6_9_Pin|LED_N_1w_5_8_12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_P_7_8_9_10_Pin|LED_P_4_5_6_15_Pin|LED_N_1r_4_7_11_Pin|LED_P_1r_1w_2_3_Pin
                          |LED_P_11_12_14_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZ_PIN_Pin|PWR220_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW3_Pin SW4_Pin */
  GPIO_InitStruct.Pin = SW3_Pin|SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_N_3_10_14_15_Pin */
  GPIO_InitStruct.Pin = LED_N_3_10_14_15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_N_3_10_14_15_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_N_2_6_9_Pin LED_N_1w_5_8_12_Pin */
  GPIO_InitStruct.Pin = LED_N_2_6_9_Pin|LED_N_1w_5_8_12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_P_7_8_9_10_Pin LED_P_4_5_6_15_Pin LED_N_1r_4_7_11_Pin LED_P_1r_1w_2_3_Pin
                           LED_P_11_12_14_Pin */
  GPIO_InitStruct.Pin = LED_P_7_8_9_10_Pin|LED_P_4_5_6_15_Pin|LED_N_1r_4_7_11_Pin|LED_P_1r_1w_2_3_Pin
                          |LED_P_11_12_14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZ_PIN_Pin PWR220_Pin */
  GPIO_InitStruct.Pin = BUZZ_PIN_Pin|PWR220_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
