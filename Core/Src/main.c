/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "CO_app_STM32.h"
#include "OD.h"

#include "pid.h"
#include "kalman_filter.h"

#include "motor_state.h"
#include "dc_motor_ctrl.h"
#include "pill_gate_motor.h"
#include "pkg_dis_motor.h"
#include "squeezer_motor.h"
#include "conveyor_motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef struct {
	uint16_t pin;
	GPIO_TypeDef *port;
} PinConfig;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t running = 0;

CANopenNodeSTM32 canOpenNodeSTM32;

const uint8_t main_node_id = 32;
const uint8_t squ_node_id = 60;
const uint8_t con_node_id = 61;

uint16_t temperature_adc = 0;

PID_TypeDef TPID;
float temp = 0.0;
float PID_out = 0.0;
float temp_set_pt = 0.0;
const int32_t SAMPLE_TIME = 100;
const float LOW_LIMIT = 200.0f;
const float HIGH_LIMIT = 2000.0f;

const uint16_t SQU_FWD = 0x0001;
const uint16_t SQU_REV = 0x0002;
const uint16_t SQU_STOP = 0x0006;
const uint16_t SQU_RESET = 0x0007;

const uint16_t CON_FWD = 0x0001;
const uint16_t CON_REV = 0x0002;
const uint16_t CON_STOP = 0x0006;
const uint16_t CON_RESET = 0x0007;

const uint16_t valve_start_index = 0x6054;
const uint16_t RS_start_index = 0x6060;
const uint16_t PH_start_index = 0x6090;

PinConfig PH_state_pins[] = {
    {PH_X1_Pin, PH_X1_GPIO_Port},
    {PH_X2_Pin, PH_X2_GPIO_Port},
    {PH_X3_Pin, PH_X3_GPIO_Port},
    {PH_X4_Pin, PH_X4_GPIO_Port},
	{PH_X5_Pin, PH_X5_GPIO_Port},
	{PH_X6_Pin, PH_X6_GPIO_Port},
	{PH_X7_Pin, PH_X7_GPIO_Port},
	{PH_X8_Pin, PH_X8_GPIO_Port},
};

PinConfig RS_state_pins[] = {
    {RS_X1_Pin, RS_X1_GPIO_Port},
    {RS_X2_Pin, RS_X2_GPIO_Port},
    {RS_X3_Pin, RS_X3_GPIO_Port},
    {RS_X4_Pin, RS_X4_GPIO_Port},
    {RS_X5_Pin, RS_X5_GPIO_Port},
    {RS_X6_Pin, RS_X6_GPIO_Port},
    {RS_X7_Pin, RS_X7_GPIO_Port},
    {RS_X8_Pin, RS_X8_GPIO_Port}
};

PinConfig valve_state_pins[] = {
    {SV_X1_Pin, SV_X1_GPIO_Port},
    {SV_X2_Pin, SV_X2_GPIO_Port},
    {SV_X3_Pin, SV_X3_GPIO_Port},
    {SV_X4_Pin, SV_X4_GPIO_Port},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


CO_SDO_abortCode_t
read_SDO(CO_SDOclient_t* SDO_C, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t* buf, size_t bufSize,
         size_t* readSize) {
    CO_SDO_return_t SDO_ret;

    // setup client (this can be skipped, if remote device don't change)
    SDO_ret = CO_SDOclient_setup(SDO_C, CO_CAN_ID_SDO_CLI + nodeId, CO_CAN_ID_SDO_SRV + nodeId, nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // initiate upload
    SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

	// upload data
	do {
		uint32_t timeDifference_us = 10000;
		CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

		SDO_ret = CO_SDOclientUpload(SDO_C, timeDifference_us, false, &abortCode, NULL, NULL, NULL);
		if (SDO_ret < 0) {
			return abortCode;
		}

//        sleep_us(timeDifference_us);
		delay_ms(timeDifference_us / 1000);
	} while (SDO_ret > 0);

	// copy data to the user buffer (for long data function must be called several times inside the loop)
	*readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);

    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t
write_SDO(CO_SDOclient_t* SDO_C, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t* data, size_t dataSize) {
    CO_SDO_return_t SDO_ret;
    bool_t bufferPartial = false;

    // setup client (this can be skipped, if remote device is the same)
    SDO_ret = CO_SDOclient_setup(SDO_C, CO_CAN_ID_SDO_CLI + nodeId, CO_CAN_ID_SDO_SRV + nodeId, nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return -1;
    }

    // initiate download
    SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex, dataSize, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return -1;
    }

    // fill data
    size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);
    if (nWritten < dataSize) {
        bufferPartial = true;
        // If SDO Fifo buffer is too small, data can be refilled in the loop.
    }

	// download data
	do {
		uint32_t timeDifference_us = 10000;
		CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

		SDO_ret = CO_SDOclientDownload(SDO_C, timeDifference_us, false, bufferPartial, &abortCode, NULL, NULL);
		if (SDO_ret < 0) {
			return abortCode;
		}

//        sleep_us(timeDifference_us);
		delay_ms(timeDifference_us / 1000);
	} while (SDO_ret > 0);

    return CO_SDO_AB_NONE;
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
  MX_DMA_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);	// start to refresh the watchdog

  for (uint8_t i = 0; i < 6; i++) {
	  HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
	  HAL_Delay(1000);
  }

//	CANopenNodeSTM32 canOpenNodeSTM32;
	canOpenNodeSTM32.CANHandle = &hcan1;
	canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
	canOpenNodeSTM32.timerHandle = &htim1;
	canOpenNodeSTM32.desiredNodeID = main_node_id;
	canOpenNodeSTM32.baudrate = 500;
	canopen_app_init(&canOpenNodeSTM32);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &temperature_adc, 1);

	set_pid_config();

	HAL_TIM_Base_Start_IT(&htim2);	// valve control
	HAL_TIM_Base_Start_IT(&htim5);	// temperature control
	HAL_TIM_Base_Start_IT(&htim7);  // conveyor motor
	HAL_TIM_Base_Start_IT(&htim8);  // squeezer motor

	HAL_TIM_Base_Start_IT(&htim9);  // heater
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);

	HAL_TIM_Base_Start_IT(&htim10);  // package dispenser motor
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);

	HAL_TIM_Base_Start_IT(&htim11);  // pill gate motor
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);

	HAL_TIM_Base_Start_IT(&htim13);
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);  // package length motor
	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);

	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);  // roller motor
	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);


	init_pill_gate_loc();
	init_sq_loc();
	init_pkg_len_loc();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, !canOpenNodeSTM32.outStatusLEDGreen);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, !canOpenNodeSTM32.outStatusLEDRed);
		canopen_app_process();

		//HAL_Delay(1);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// TODO: control the specific action
	if (htim == canopenNodeSTM32->timerHandle) {
		htim1_cb();
	} else if (htim == (&htim2)) {
		htim2_cb();
	} else if (htim == (&htim4)) {
		htim4_cb();
	} else if (htim == (&htim5)) {
		htim5_cb();
	} else if (htim == (&htim7)) {
		htim7_cb();
	} else if (htim == (&htim8)) {
		htim8_cb();
	} else if (htim == (&htim9)) {
		htim9_cb();
	} else if (htim == (&htim10)) {
		htim10_cb();
	} else if (htim == (&htim11)) {
		htim11_cb();
	} else if (htim == (&htim12)) {
		htim12_cb();
	} else if (htim == (&htim13)) {
		htim13_cb();
	} else if (htim == (&htim14)) {
		htim14_cb();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
	// TODO: stop the specific action
	switch (GPIO_Pin) {
	case PH_X1_Pin:
		exti1_cb();
		break;
	case PH_X2_Pin:
		exti2_cb();
		break;
	case PH_X3_Pin:
		exti3_cb();
		break;
	case PH_X4_Pin:
		exti4_cb();
		break;
	case PH_X5_Pin:
		exti5_cb();
		break;
	case PH_X6_Pin:
		exti6_cb();
		break;
	case PH_X7_Pin:
		exti7_cb();
		break;
	case PH_X8_Pin:
		exti8_cb();
		break;
	}
}

void htim1_cb(void) {
	canopen_app_interrupt();
}

void htim2_cb(void) {
	control_valve(0x6050, SV_X1_GPIO_Port, SV_X1_Pin);
	control_valve(0x6051, SV_X2_GPIO_Port, SV_X2_Pin);
	control_valve(0x6052, SV_X3_GPIO_Port, SV_X3_Pin);
	control_valve(0x6053, SV_X4_GPIO_Port, SV_X4_Pin);

	uint8_t valve_pin_states = 0;

	for (size_t i = 0; i < sizeof(valve_state_pins) / sizeof(valve_state_pins[0]); i++) {
		GPIO_PinState result = !HAL_GPIO_ReadPin(valve_state_pins[i].port, valve_state_pins[i].pin);

		if (OD_set_u8(OD_find(OD, valve_start_index + i), 0x00, (uint8_t) result, false) != ODR_OK)
			show_err_LED();

		valve_pin_states |= (result << i);
	}

	if (OD_set_u8(OD_find(OD, 0x6058), 0x00, (uint8_t) valve_pin_states, false) != ODR_OK)
		show_err_LED();

	uint8_t RS_pin_states = 0;

	for (size_t i = 0; i < sizeof(RS_state_pins) / sizeof(RS_state_pins[0]); i++) {
		GPIO_PinState result = !HAL_GPIO_ReadPin(RS_state_pins[i].port, RS_state_pins[i].pin);

		if (OD_set_u8(OD_find(OD, RS_start_index + i), 0x00, (uint8_t) result, false) != ODR_OK)
			show_err_LED();

		RS_pin_states |= (result << i);
	}

	if (OD_set_u8(OD_find(OD, 0x6068), 0x00, (uint8_t) RS_pin_states, false) != ODR_OK)
		show_err_LED();

	uint16_t PH_pin_states = 0;
	for (size_t i = 0; i < sizeof(PH_state_pins) / sizeof(PH_state_pins[0]); i++) {
		GPIO_PinState result = !HAL_GPIO_ReadPin(PH_state_pins[i].port, PH_state_pins[i].pin);

		PH_pin_states |= (result << i);
	}

	if (OD_set_u16(OD_find(OD, 0x6090), 0x00, (uint16_t) RS_pin_states, false) != ODR_OK)
		show_err_LED();
}

void htim3_cb(void) {

}

void htim4_cb(void) {
	HAL_GPIO_TogglePin(LED_Default_GPIO_Port, LED_Default_Pin);
	HAL_IWDG_Refresh(&hiwdg);
	running++;
}

void htim5_cb(void) {
	uint16_t adc_sample;
	uint8_t enable_heater, enable_kalman;

	adc_sample = temperature_adc;

	if (OD_get_u8(OD_find(OD, 0x6007), 0x00, &enable_kalman, false) != ODR_OK)
		show_err_LED();

	if (enable_kalman)
		temp = get_temperature(kalman_filter(adc_sample));
	else
		temp = get_temperature(adc_sample);

	if (OD_set_u16(OD_find(OD, 0x6001), 0x00, (uint16_t) temp, false) != ODR_OK)
		show_err_LED();

	if (OD_set_u16(OD_find(OD, 0x6002), 0x00, adc_sample, false) != ODR_OK)
		show_err_LED();

	enable_heater = 0;
	if (OD_get_u8(OD_find(OD, 0x6003), 0x00, &enable_heater, false) != ODR_OK)
		show_err_LED();

	HAL_GPIO_WritePin(Heat_Enable_GPIO_Port, Heat_Enable_Pin, !enable_heater);

	if (enable_heater) {
		PID_Compute(&TPID);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, (uint16_t ) PID_out);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, (uint16_t ) PID_out);
		if (OD_set_u16(OD_find(OD, 0x6008), 0x00, (uint16_t) PID_out, false) != ODR_OK)
			show_err_LED();
	}
}

void htim6_cb(void) {

}

void htim7_cb(void) {
	motor_state_t _state = M_ERROR;
	uint8_t _dir, _ctrl;
	uint16_t _speed;
	bool_t _start_flag = 0;

	_state = get_od_con_state(OD, 0);

	switch (_state) {
	case M_IDLE:
		_ctrl = get_od_con_ctrl(OD, 0);
		_speed = get_od_con_speed(OD, 0);
		_start_flag = _ctrl > 0 && _speed > 0;

		if (_start_flag) {
			write_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, con_node_id, 0x2000, 0x1, (uint8_t*) &CON_RESET, 2);
			write_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, con_node_id, 0x2000, 0x2, (uint8_t*) &_speed, 2);

			set_od_con_state(OD, 0, M_RUNNING);
			_dir = get_od_con_dir(OD, 0);
			if (_dir == 0) {
				write_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, con_node_id, 0x2000, 0x1, (uint8_t*) &CON_FWD, 2);
			} else {
				write_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, con_node_id, 0x2000, 0x1, (uint8_t*) &CON_REV, 2);
			}
		}
		break;
	case M_RUNNING:
		_ctrl = get_od_con_ctrl(OD, 0);

		if (_ctrl == 0) {
			set_od_con_state(OD, 0, M_STOP);
		}
		break;
	case M_STOP:
		write_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, con_node_id, 0x2000, 0x1, (uint8_t*) &CON_STOP, 2);
		set_od_con_state(OD, 0, M_RESET);
		break;
	case M_RESET:
		set_od_con_mode(OD, 0, 0);
		set_od_con_dir(OD, 0, 0);
		set_od_con_ctrl(OD, 0, 0);
		set_od_con_state(OD, 0, M_IDLE);
		break;
	case M_ERROR:
		_ctrl = get_od_con_ctrl(OD, 0);

		if (_ctrl == 0) {
			set_od_con_state(OD, 0, M_STOP);
		}
		show_err_LED();
		break;
	default:
		break;
	}
}

void htim8_cb(void) {
	motor_state_t _state = M_ERROR;
	uint8_t _dir, _ctrl, _loc;
	uint16_t _speed;
	bool_t _start_flag = 0;

	_state = get_od_sq_state(OD, 0);

	switch (_state) {
	case M_IDLE:
		_ctrl = get_od_sq_ctrl(OD, 0);
		_speed = get_od_sq_speed(OD, 0);
		_start_flag = _ctrl > 0 && _speed > 0;

		if (_start_flag) {
			write_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, squ_node_id, 0x2000, 0x1, (uint8_t*) &SQU_RESET, 2);
			write_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, squ_node_id, 0x2000, 0x2, (uint8_t*) &_speed, 2);

			_dir = get_od_sq_dir(OD, 0);
			_loc = get_od_sq_loc(OD, 0);

			if (_dir == 0 && _loc == SQUEEZE_HOME_PT) {
				set_od_sq_state(OD, 0, M_RUNNING);
				write_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, squ_node_id, 0x2000, 0x1, (uint8_t*) &SQU_FWD, 2);
			} else if (_dir == 1 && _loc == SQUEEZE_PT) {
				set_od_sq_state(OD, 0, M_RUNNING);
				write_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, squ_node_id, 0x2000, 0x1, (uint8_t*) &SQU_REV, 2);
			} else {
				set_od_sq_state(OD, 0, M_RESET);
			}
		}
		break;
	case M_RUNNING:
		_ctrl = get_od_sq_ctrl(OD, 0);

		if (_ctrl == 0) {
			set_od_sq_state(OD, 0, M_STOP);
		}
		break;
	case M_BRAKE: {
		uint8_t _wait_time = get_od_sq_wait_time(OD, 0);
		_wait_time--;

		if (_wait_time == 0)
			set_od_sq_state(OD, 0, M_STOP);
		else
			set_od_sq_wait_time(OD, 0, _wait_time);
		break;
	}
	case M_STOP:
		write_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, squ_node_id, 0x2000, 0x1, (uint8_t*) &SQU_STOP, 2);
		set_od_sq_state(OD, 0, M_RESET);
		break;
	case M_RESET:
		set_od_sq_wait_time(OD, 0, 10);
		set_od_sq_mode(OD, 0, 0);
		set_od_sq_dir(OD, 0, 0);
		set_od_sq_ctrl(OD, 0, 0);
		set_od_sq_state(OD, 0, M_IDLE);
		break;
	case M_ERROR:
		_ctrl = get_od_sq_ctrl(OD, 0);

		if (_ctrl == 0) {
			set_od_sq_state(OD, 0, M_STOP);
		}
		show_err_LED();
		break;
	default:
		show_err_LED();
		break;
	}
}

void htim9_cb(void) {

}

void htim10_cb(void) {
	motor_state_t _state = M_ERROR;
	uint8_t _dir = 0;
	bool_t _start_flag = 0;

	_state = get_od_pkg_dis_state(OD, 0);
	switch (_state) {
	case M_IDLE: {
		uint8_t ctrl, rotate_pulses;

		ctrl = get_od_pkg_dis_ctrl(OD, 0);
		rotate_pulses = get_od_pkg_dis_rotate_pulses(OD, 0);

		_start_flag = ctrl > 0 && rotate_pulses > 0;
		if (_start_flag) {
			_dir = get_od_pkg_dis_rotate_dir(OD, 0);

			HAL_GPIO_WritePin(Pkg_Dis_Enable_GPIO_Port, Pkg_Dis_Enable_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Pkg_Dis_Dir_GPIO_Port, Pkg_Dis_Dir_Pin, _dir == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 50);

			set_od_pkg_dis_state(OD, 0, M_RUNNING);
		}
		break;
	}
	case M_RUNNING: {
		uint16_t curr_pulses, rotate_pulses;

		curr_pulses = get_od_pkg_dis_curr_pulses(OD, 0);
		set_od_pkg_dis_curr_pulses(OD, 0, ++curr_pulses);

		rotate_pulses = get_od_pkg_dis_rotate_pulses(OD, 0);

		if (curr_pulses >= rotate_pulses)
			set_od_pkg_dis_state(OD, 0, M_STOP);

		break;
	}
	case M_BRAKE: {
		// stepper motor do not have brake
		set_od_pkg_dis_state(OD, 0, M_STOP);
		break;
	}
	case M_STOP: {
		HAL_GPIO_WritePin(Pkg_Dis_Enable_GPIO_Port, Pkg_Dis_Enable_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);

		set_od_pkg_dis_state(OD, 0, M_RESET);
		break;
	}
	case M_RESET: {
		set_od_pkg_dis_rotate_pulses(OD, 0, 0);
		set_od_pkg_dis_curr_pulses(OD, 0, 0);
		set_od_pkg_dis_rotate_dir(OD, 0, 0);
		set_od_pkg_dis_ctrl(OD, 0, 0);
		set_od_pkg_dis_state(OD, 0, M_IDLE);
		break;
	}
	case M_ERROR:
		HAL_GPIO_WritePin(Pkg_Dis_Enable_GPIO_Port, Pkg_Dis_Enable_Pin, GPIO_PIN_RESET);
		show_err_LED();
		break;
	default:
		break;
	}
}

void htim11_cb(void) {
	motor_state_t _state = M_ERROR;
	uint8_t _dir = 0;
	bool_t _start_flag = 0;

	_state = get_od_pill_gate_state(OD, 0);

	switch (_state) {
	case M_IDLE:
		uint8_t ctrl, rotate_pulses;

		ctrl = get_od_pill_gate_ctrl(OD, 0);
		rotate_pulses = get_od_pill_gate_rotate_pulses(OD, 0);

		_start_flag = ctrl > 0 && rotate_pulses > 0;
		if (_start_flag) {
			uint8_t _loc = get_od_pill_gate_loc(OD, 0);
			_dir = get_od_pill_gate_rotate_dir(OD, 0);

			if (_dir == 1) {
				if (_loc >= PILL_GATE_PT_0 && _loc <= PILL_GATE_PT_3) {
					set_od_pill_gate_state(OD, 0, M_RUNNING);
					set_od_pill_gate_loc(OD, 0, ++_loc);

					HAL_GPIO_WritePin(Pill_Gate_Enable_GPIO_Port, Pill_Gate_Enable_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(Pill_Gate_Dir_GPIO_Port, Pill_Gate_Dir_Pin, get_open_dir(1));

					__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 20);
				} else {
					set_od_pill_gate_state(OD, 0, M_RESET);
				}
			} else if (_dir == 0) {
				if (_loc >= PILL_GATE_PT_1 && _loc <= PILL_GATE_PT_4) {
					set_od_pill_gate_state(OD, 0, M_RUNNING);
					set_od_pill_gate_loc(OD, 0, --_loc);

					HAL_GPIO_WritePin(Pill_Gate_Enable_GPIO_Port, Pill_Gate_Enable_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(Pill_Gate_Dir_GPIO_Port, Pill_Gate_Dir_Pin, get_open_dir(0));

					__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 20);
				} else {
					set_od_pill_gate_state(OD, 0, M_RESET);
				}
			}
		}
		break;
	case M_RUNNING: {
		uint16_t curr_pulses, rotate_pulses;

		curr_pulses = get_od_pill_gate_curr_pulses(OD, 0);
		set_od_pill_gate_curr_pulses(OD, 0, ++curr_pulses);

		rotate_pulses = get_od_pill_gate_rotate_pulses(OD, 0);

		if (curr_pulses >= rotate_pulses)
			set_od_pill_gate_state(OD, 0, M_STOP);

		break;
	}
	case M_BRAKE: {
		// stepper motor do not have brake
		set_od_pill_gate_state(OD, 0, M_STOP);
		break;
	}
	case M_STOP: {
		HAL_GPIO_WritePin(Pill_Gate_Enable_GPIO_Port, Pill_Gate_Enable_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
		set_od_pill_gate_state(OD, 0, M_RESET);
		break;
	}
	case M_RESET: {
		set_od_pill_gate_rotate_pulses(OD, 0, 0);
		set_od_pill_gate_curr_pulses(OD, 0, 0);
		set_od_pill_gate_rotate_dir(OD, 0, 0);
		set_od_pill_gate_ctrl(OD, 0, 0);
		set_od_pill_gate_state(OD, 0, M_IDLE);
		break;
	}
	case M_ERROR: {
		HAL_GPIO_WritePin(Pill_Gate_Enable_GPIO_Port, Pill_Gate_Enable_Pin, GPIO_PIN_RESET);
		show_err_LED();
		break;
	}
	default:
		break;
	}
}

void htim12_cb(void) {

}

void htim13_cb(void) {
	motor_state_t _state = M_ERROR;
	uint8_t _dir = 0;
	bool_t _start_flag = 0;

	dc_motor_controller(PKG_LEN_DC, OD, &_state, &_dir, &_start_flag);

	switch (_state) {
	case M_IDLE:
		if (_start_flag) {
			uint8_t _loc = get_od_pkg_len_loc(OD, 0);
			if (_dir == 1) {
				if (_loc == PKG_LEN_PT_0) {
					HAL_GPIO_WritePin(Pkg_Len_In_1_GPIO_Port, Pkg_Len_In_1_Pin, SET);
					HAL_GPIO_WritePin(Pkg_Len_In_2_GPIO_Port, Pkg_Len_In_2_Pin, RESET);
					__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 15);
				} else {
					set_od_pkg_len_state(OD, 0, M_BRAKE);
				}
			} else if (_dir == 0) {
				if (_loc == PKG_LEN_PT_1) {
					HAL_GPIO_WritePin(Pkg_Len_In_1_GPIO_Port, Pkg_Len_In_1_Pin, RESET);
					HAL_GPIO_WritePin(Pkg_Len_In_2_GPIO_Port, Pkg_Len_In_2_Pin, SET);
					__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 20);
				} else {
					set_od_pkg_len_state(OD, 0, M_BRAKE);
				}
			}
		}
		break;
	case M_BRAKE:
		HAL_GPIO_WritePin(Pkg_Len_In_1_GPIO_Port, Pkg_Len_In_1_Pin, RESET);
		HAL_GPIO_WritePin(Pkg_Len_In_2_GPIO_Port, Pkg_Len_In_2_Pin, RESET);
//		__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);
		break;
	case M_STOP:
		__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);
		break;
	case M_ERROR:
		show_err_LED();
		break;
	default:
		break;
	}
}

void htim14_cb(void) {
	motor_state_t _state = M_ERROR;
	uint8_t _dir = 0;
	bool_t _start_flag = 0;

	dc_motor_controller(ROLLER_DC, OD, &_state, &_dir, &_start_flag);

	switch (_state) {
	case M_IDLE:
		if (_start_flag) {
			if (_dir == 0) {
				HAL_GPIO_WritePin(Roller_In_1_GPIO_Port, Roller_In_1_Pin, SET);
				HAL_GPIO_WritePin(Roller_In_2_GPIO_Port, Roller_In_2_Pin, RESET);
			} else if (_dir == 1) {
				HAL_GPIO_WritePin(Roller_In_1_GPIO_Port, Roller_In_1_Pin, RESET);
				HAL_GPIO_WritePin(Roller_In_2_GPIO_Port, Roller_In_2_Pin, SET);
			}
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 100);
		}
		break;
	case M_BRAKE:
		HAL_GPIO_WritePin(Roller_In_1_GPIO_Port, Roller_In_1_Pin, RESET);
		HAL_GPIO_WritePin(Roller_In_2_GPIO_Port, Roller_In_2_Pin, RESET);
//		__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);
		break;
	case M_STOP:
		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
		break;
	case M_ERROR:
		show_err_LED();
		break;
	default:
		break;
	}
}

void exti1_cb(void) {
	if (get_od_con_state(OD, 0) == M_RUNNING && get_od_con_stop_by_ph(OD, 0) > 0) {
		set_od_con_state(OD, 0, M_STOP);
	}
}

void exti2_cb(void) {
	if (get_od_sq_state(OD, 0) == M_RUNNING) {
		set_od_sq_loc(OD, 0, SQUEEZE_PT);
		set_od_sq_state(OD, 0, M_BRAKE);
	}
}

void exti3_cb(void) {
	if (get_od_sq_state(OD, 0) == M_RUNNING) {
		set_od_sq_loc(OD, 0, SQUEEZE_HOME_PT);
		set_od_sq_state(OD, 0, M_STOP);
	}
}

void exti4_cb(void) {
	if (get_od_roller_state(OD, 0) == M_RUNNING && get_od_roller_mode(OD, 0) == TRAY) {
		uint8_t curr_step = get_od_dc_curr_step(ROLLER_DC, OD, 0);
		set_od_dc_curr_step(ROLLER_DC, OD, 0, ++curr_step);
	}
}

void exti5_cb(void) {
	if (get_od_roller_state(OD, 0) == M_RUNNING && get_od_roller_mode(OD, 0) == ROLLER_HOMING) {
		uint8_t curr_step = get_od_dc_curr_step(ROLLER_DC, OD, 0);
		set_od_dc_curr_step(ROLLER_DC, OD, 0, ++curr_step);
	}
}

void exti6_cb(void) {
	if (get_od_pill_gate_state(OD, 0) == M_RUNNING) {
		set_od_pill_gate_loc(OD, 0, PILL_GATE_PT_0);
		set_od_pill_gate_state(OD, 0, M_STOP);
	}
}

void exti7_cb(void) {
	if (get_od_pkg_len_state(OD, 0) == M_RUNNING) {
		uint8_t curr_step = get_od_dc_curr_step(PKG_LEN_DC, OD, 0);
		set_od_dc_curr_step(PKG_LEN_DC, OD, 0, ++curr_step);
		set_od_pkg_len_loc(OD, 0, PKG_LEN_PT_0);
	}
}

void exti8_cb(void) {
	if (get_od_pkg_len_state(OD, 0) == M_RUNNING) {
		uint8_t curr_step = get_od_dc_curr_step(PKG_LEN_DC, OD, 0);
		set_od_dc_curr_step(PKG_LEN_DC, OD, 0, ++curr_step);
		set_od_pkg_len_loc(OD, 0, PKG_LEN_PT_1);
	}
}

void init_pill_gate_loc(void) {
	if (HAL_GPIO_ReadPin(PH_X6_GPIO_Port, PH_X6_Pin) == GPIO_PIN_RESET)
		set_od_pill_gate_loc(OD, 0, PILL_GATE_PT_0);
}

void init_pkg_len_loc(void) {
	if (HAL_GPIO_ReadPin(PH_X7_GPIO_Port, PH_X7_Pin) == GPIO_PIN_RESET)
		set_od_pkg_len_loc(OD, 0, PKG_LEN_PT_0);
	else if (HAL_GPIO_ReadPin(PH_X8_GPIO_Port, PH_X8_Pin) == GPIO_PIN_RESET)
		set_od_pkg_len_loc(OD, 0, PKG_LEN_PT_1);
}

void init_sq_loc(void) {
	if (HAL_GPIO_ReadPin(PH_X2_GPIO_Port, PH_X2_Pin) == GPIO_PIN_RESET)
		set_od_sq_loc(OD, 0, SQUEEZE_PT);
	else if (HAL_GPIO_ReadPin(PH_X3_GPIO_Port, PH_X3_Pin) == GPIO_PIN_RESET)
		set_od_sq_loc(OD, 0, SQUEEZE_HOME_PT);
}

void control_valve(uint16_t ctrl_index, GPIO_TypeDef *port, uint16_t pin) {
	uint8_t valve_control = 0;

	if (OD_get_u8(OD_find(OD, ctrl_index), 0x00, &valve_control, false) != ODR_OK) {
		show_err_LED();
	}

	HAL_GPIO_WritePin(port, pin, !valve_control);
}

inline float get_temperature(uint16_t adc_value) {
	float val = ((3.3f * (float) adc_value / 4096.0f) - 1.25f) / 0.005f;

	return val > 0.0f ? val : 0.0f;
}

inline void show_err_LED(void) {
	HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
}

void set_pid_config(void) {
	uint16_t target_temp, K_p, K_i, K_d;

	if (OD_get_u16(OD_find(OD, 0x6000), 0x00, &target_temp, false) != ODR_OK)
		show_err_LED();
	if (OD_get_u16(OD_find(OD, 0x6004), 0x00, &K_p, false) != ODR_OK)
		show_err_LED();
	if (OD_get_u16(OD_find(OD, 0x6005), 0x00, &K_i, false) != ODR_OK)
		show_err_LED();
	if (OD_get_u16(OD_find(OD, 0x6006), 0x00, &K_d, false) != ODR_OK)
		show_err_LED();

	temp_set_pt = (float) target_temp;

	PID(&TPID, &temp, &PID_out, &temp_set_pt, (float) K_p, (float) K_i, (float) K_d, _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&TPID, SAMPLE_TIME);
	PID_SetOutputLimits(&TPID, LOW_LIMIT, HIGH_LIMIT);
}

void delay_ms(uint32_t ms) {
	volatile uint32_t count = 0;
	for (count = 0; count < ms * 1000; count++) {
		__asm("NOP");
	}
}

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
	while (1) {
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
