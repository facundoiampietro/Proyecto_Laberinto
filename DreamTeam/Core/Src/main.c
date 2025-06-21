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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h> //Nos permite usar la función sprintf
#include <string.h> //Nos permite usar la función strcat
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define norte 0 // Direcciones hacia donde puede estar mirando el autito
#define este  1
#define sur   2
#define oeste 3

#define adelante 0 // Tipos de giro que puede hacer
#define izquierda 1
#define derecha  2
#define giro_180   3

#define avance     0b01
#define retroceso  0b10
#define freno      0b00

#define v_max 63999
#define v_min 31000
#define v_media_izq 36000
#define v_media_der 38000
#define v_media 32000

#define tiempo_giro90_der 570
#define tiempo_giro90_izq 570
#define tiempo_giro180 960
#define tiempo_giro90_2 900

#define tiempo_muerto_avanzar 200
#define tiempo_muerto 300
#define tiempo_mini 150
#define tiempo_rebotes 20
#define tiempo_muerto_retroceso 100

#define cant_casilleros 16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
char mensaje[16];
const uint8_t delay = 50;
uint8_t ubicacion = 0;    //defino ubicacion (va a ser un numero entre 0 y 15)
uint8_t orientacion_actual = norte;
uint8_t orientacion_futura = norte;
uint8_t casilla_n = 4;   // la casilla a la cual hay q ir
uint8_t giro;
uint8_t peso[cant_casilleros];
uint8_t pared[cant_casilleros];
uint8_t contador_casillas = 0;
uint8_t prueba;
uint8_t contador_giros = 0;
uint16_t sensor_izq_min = 32700;
uint16_t sensor_der_min = 32700;
uint16_t sensor_izq_max = 0;
uint16_t sensor_der_max = 0;
uint16_t margen_i;
uint16_t margen_d;

uint8_t camino_solucion[32] = { 0 };
uint8_t girando = 0;
volatile uint32_t tiempo_inicio = 0;
volatile uint8_t solicitud_pared = 0;
volatile uint8_t filtrado_pared = 0;

volatile uint32_t tiempo_inicio_2 = 0;
volatile uint8_t solicitud_linea = 0;
volatile uint8_t filtrado_linea = 0;

uint16_t dma_buffer[64];

volatile uint16_t sensor_izq_avg;
volatile uint16_t sensor_der_avg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
uint8_t obtener_orientacion_futura(uint8_t casilla_actual, uint8_t casilla_n);
uint8_t obtenerGiro(uint8_t orientacion_actual, uint8_t orientacion_futura);
uint8_t act_ubicacion(uint8_t ubicacion, uint8_t orientacion_actual);
void act_pesos(uint8_t *pared, uint8_t *peso);
uint8_t act_pared(uint8_t *pared, uint8_t ubicacion, uint8_t orientacion_actual);
uint8_t calculo_minimo_peso(uint8_t *peso, uint8_t *pared, uint8_t ubicacion, uint8_t orientacion_actual);

bool verificar_sensor(void);

void de_reversa_mami(void);
void correccion_avanzar(void);
void avanzar(void);
void apagar_izquierda(void);
void apagar_derecha(void);
void setMotorIzquierdo(uint8_t modo);
void setMotorDerecho(uint8_t modo);
void ejecutarGiro(uint8_t giro);
void prueba_avanzar(void);
void prueba_rapida(void);
void prueba_casilla_n(void);
void programa_principal(void);
void prueba_post_relleno(void);
void error(void);
void mini_retroceso(void);
void ajuste_automatico(void);
void mini_avance(void);
void de_reversa_mami(void);
void envio_ubicacion(uint8_t ubicacion,uint8_t casilla_n);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) dma_buffer, 64);

	peso[15] = 0;  //inicialización de pesos
	peso[11] = peso[14] = 1;
	peso[7] = peso[10] = peso[13] = 2;
	peso[3] = peso[6] = peso[9] = peso[12] = 3;
	peso[2] = peso[5] = peso[8] = 4;
	peso[1] = peso[4] = 5;
	peso[0] = 6;

	//paredes de cada casilla (8=pared en norte, 4=pared en este, 2=pared en sur, 1=pared en oeste)
	pared[0] = 6; //se suma sur y este
	pared[1] = pared[2] = 2; //solo sur
	pared[3] = 3;  //se suma sur y oeste
	pared[7] = pared[11] = 1; // solo oeste
	pared[15] = 9;  //se suma norte y oeste
	pared[14] = pared[13] = 8; //solo norte
	pared[12] = 12; //se suma mprte y este
	pared[4] = pared[8] = 4; //solo este
	pared[5] = pared[6] = pared[9] = pared[10] = 0;

	girando = 0;

	HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_RESET); // INICIALIZACION EN AVANZAR
	HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Inicio de la modulación PWM, rueda izquierda
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // Inicio de la modulación PWM, rueda derecha
	TIM3->CCR3 = v_media_izq; // rueda a velocidad media (condigurable)
	TIM3->CCR4 = v_media_der; // rueda a velocidad media

	HAL_GPIO_WritePin(led_verde_GPIO_Port, led_verde_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(led_naranja_GPIO_Port, led_naranja_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(led_rojo_GPIO_Port, led_rojo_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(led_azul_GPIO_Port, led_azul_Pin, GPIO_PIN_RESET);

	prueba = 6; //Aca se elige que programa queremos que se realice
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_WritePin(led_verde_GPIO_Port, led_verde_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_naranja_GPIO_Port, led_naranja_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_rojo_GPIO_Port, led_rojo_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_azul_GPIO_Port, led_azul_Pin, GPIO_PIN_RESET);
		switch (prueba) {

		case 0:
			prueba_avanzar();
			break;

		case 1:

			break;

		case 4:
			programa_principal();
			break;

		case 5:
			ajuste_automatico();
			break;

		case 6:
			sensor_izq_min = 32700;
			sensor_der_min = 32700;
			sensor_izq_max = 0;
			sensor_der_max = 0;
			prueba = 5;
		case 10:
			TIM3->CCR3 = 0;
			TIM3->CCR4 = 0;
			HAL_GPIO_WritePin(led_verde_GPIO_Port, led_verde_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(led_naranja_GPIO_Port, led_naranja_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(led_rojo_GPIO_Port, led_rojo_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(led_azul_GPIO_Port, led_azul_Pin, GPIO_PIN_SET);
			break;
		case 11:
			contador_casillas = contador_casillas - 1;
			prueba = 12;
			break;
		case 12:
			de_reversa_mami();
			break;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 63999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, m0_izquierda_Pin|m1_izquierda_Pin|m0_derecha_Pin|m1_derecha_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, led_verde_Pin|led_naranja_Pin|led_rojo_Pin|led_azul_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : boton_Pin */
  GPIO_InitStruct.Pin = boton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(boton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : m0_izquierda_Pin m1_izquierda_Pin m0_derecha_Pin m1_derecha_Pin */
  GPIO_InitStruct.Pin = m0_izquierda_Pin|m1_izquierda_Pin|m0_derecha_Pin|m1_derecha_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : led_verde_Pin led_naranja_Pin led_rojo_Pin led_azul_Pin */
  GPIO_InitStruct.Pin = led_verde_Pin|led_naranja_Pin|led_rojo_Pin|led_azul_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor_frontal_Pin sensor_linea_Pin */
  GPIO_InitStruct.Pin = sensor_frontal_Pin|sensor_linea_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void de_reversa_mami(void) {//codigo para ir de la casilla 15 a la 0... muy chiche

	if (verificar_sensor()) { //cambio de casilla
			contador_giros = 0;
			contador_casillas = contador_casillas - 1;
			ubicacion = act_ubicacion(ubicacion, orientacion_actual);
			casilla_n = calculo_minimo_peso(peso, pared, ubicacion, orientacion_actual); //calcula la casilla a la que hay q ir
			orientacion_futura = obtener_orientacion_futura(ubicacion, casilla_n); //obtiene a la orientacion a la que hay que ir con la ubicacion actual y casilla n
			giro = obtenerGiro(orientacion_actual, orientacion_futura); //con la orientacion futura (orientación q quiero) y la orientacion actual que giro debo realizar
			orientacion_actual = orientacion_futura;  //actualizo la orientación
			ejecutarGiro(giro); //giro y me voy del if
		}
	if (ubicacion == 0)
		prueba = 10;
}
void ajuste_automatico(void) {
	if ((sensor_der_min == 0) || (sensor_izq_min == 0)) {
		sensor_der_min = 32000;
		sensor_izq_min = 32000;
	}
	if (sensor_der_min > sensor_der_avg) {
		sensor_der_min = sensor_der_avg;
	}
	if (sensor_izq_min > sensor_izq_avg) {
		sensor_izq_min = sensor_izq_avg;
	}
	if (sensor_der_max < sensor_der_avg) {
		sensor_der_max = sensor_der_avg;
	}
	if (sensor_izq_max < sensor_izq_avg) {
		sensor_izq_max = sensor_izq_avg;
	}
	if (HAL_GPIO_ReadPin(boton_GPIO_Port, boton_Pin) == GPIO_PIN_SET) {
		HAL_Delay(40);
		if (HAL_GPIO_ReadPin(boton_GPIO_Port, boton_Pin) == GPIO_PIN_SET) {
			margen_d = ((sensor_der_max * 0.4) + (sensor_der_min * 0.6));
			margen_i = ((sensor_izq_max * 0.4) + (sensor_izq_min * 0.6));
			prueba = 4;
		}
	}
}

void prueba_rapida(void) {
	correccion_avanzar(); //codigo sencillo para configurar los margenes del ADC y verificacion de las ruedas y pilas
	ejecutarGiro(izquierda);
	correccion_avanzar();
	while (1)
		;
}
void prueba_avanzar(void) {
//	correccion_avanzar(); //codigo sencillo para configurar los margenes del ADC y verificacion de las ruedas y pilas
	ejecutarGiro(izquierda);
	ejecutarGiro(adelante);

	while (1)
		;
}

bool verificar_sensor(void) {
	static GPIO_PinState ultima_lectura_valida = GPIO_PIN_SET;
	GPIO_PinState lectura1, lectura2;
	bool pedido = false;
	lectura1 = HAL_GPIO_ReadPin(sensor_linea_GPIO_Port, sensor_linea_Pin);
	if (lectura1 != ultima_lectura_valida) {
		HAL_Delay(tiempo_rebotes);
		lectura2 = HAL_GPIO_ReadPin(sensor_linea_GPIO_Port, sensor_linea_Pin);
		if (lectura2 == lectura1)
			ultima_lectura_valida = lectura2;
		if (ultima_lectura_valida == GPIO_PIN_RESET)
			pedido = true;
	}
	return pedido;
}

void programa_principal(void) {
	correccion_avanzar();

	if (verificar_sensor()) { //cambio de casilla
			contador_giros = 0;
			contador_casillas = contador_casillas + 1;
			ubicacion = act_ubicacion(ubicacion, orientacion_actual);
			envio_ubicacion(ubicacion, casilla_n);
			casilla_n = calculo_minimo_peso(peso, pared, ubicacion, orientacion_actual); //calcula la casilla a la que hay q ir
			orientacion_futura = obtener_orientacion_futura(ubicacion, casilla_n); //obtiene a la orientacion a la que hay que ir con la ubicacion actual y casilla n
			giro = obtenerGiro(orientacion_actual, orientacion_futura); //con la orientacion futura (orientación q quiero) y la orientacion actual que giro debo realizar
			orientacion_actual = orientacion_futura;  //actualizo la orientación
			ejecutarGiro(giro); //giro y me voy del if
		}
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(sensor_frontal_GPIO_Port, sensor_frontal_Pin)) {
		HAL_Delay(tiempo_rebotes);
		if (HAL_GPIO_ReadPin(sensor_frontal_GPIO_Port, sensor_frontal_Pin) == GPIO_PIN_RESET) {
			act_pared(pared, ubicacion, orientacion_actual); //primero actualiza la pared encontrada
			act_pesos(pared, peso);  //luego actualiza el peso
			casilla_n = calculo_minimo_peso(peso, pared, ubicacion, orientacion_actual); //calcula la casilla a la que hay q ir
			orientacion_futura = obtener_orientacion_futura(ubicacion, casilla_n); //obtiene a la orientacion a la que hay que ir con la ubicacion actual y casilla n
			giro = obtenerGiro(orientacion_actual, orientacion_futura); //con la orientacion futura (orientación q quiero) y la orientacion actual que giro debo realizar
			orientacion_actual = orientacion_futura;  //actualizo la orientación
			if (contador_giros > 0) {
				mini_retroceso();
			}
			ejecutarGiro(giro); //giro y me voy del if
		}
	}
	if (ubicacion == 15) {
		prueba = 10;
	}
}
void error(void) {      //CUANDO HAY ERROR RETROCEDE INFINITAMENTE
	HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_RESET);
	TIM3->CCR3 = v_media_izq; // rueda a velocidad media (condigurable)
	TIM3->CCR4 = v_media_der; // rueda a velocidad media
	while (1)
		;
}

uint8_t obtener_orientacion_futura(uint8_t ubicacion, uint8_t casilla_n) { // Devuelve la dirección hacia donde hay que ir según la diferencia entre casillas

	if (casilla_n == ubicacion + 1)
		return oeste;
	if (casilla_n == ubicacion - 1)
		return este;
	if (casilla_n == ubicacion + 4)
		return norte;
	if (casilla_n == ubicacion - 4)
		return sur;

	return 100; // Movimiento no válido (no adyacente o fuera del tablero)
}

uint8_t obtenerGiro(uint8_t orientacion_actual, uint8_t orientacion_futura) { // Calcula el giro que debe hacer el autito para pasar de su orientación actual a la deseada
	int diferencia = (orientacion_futura - orientacion_actual + 4) % 4; //el %4 se queda con el resto de la divsion por 4

	switch (diferencia) {
	case 0:
		return adelante;
	case 1:
		return derecha;
	case 2:
		return giro_180;
	case 3:
		return izquierda;
	default:
		return 100; // Error
	}
}
uint8_t act_ubicacion(uint8_t ubicacion, uint8_t orientacion_actual) {

	switch (orientacion_actual) {
	case 0:
		return ubicacion + 4;
	case 1:
		return ubicacion - 1;
	case 2:
		return ubicacion - 4;
	case 3:
		return ubicacion + 1;
	default:
		return 100; // Error
	}
}

void promediar(uint16_t *buffer) {
	uint32_t sensor_izq_sum = 0, sensor_der_sum = 0; // Inicializamos variables para acumular la suma de las muestras
	for (int i = 0; i < 16; ++i) { // Ralizamos la suma incrementando el puntero a las muestras
		sensor_izq_sum += buffer[0];
		sensor_der_sum += buffer[1]; //hago la suma y la guardo (eso significa el +=)
		buffer += 2;  //desplazo el puntero 2
	}
	sensor_izq_avg = sensor_izq_sum / 16; //divido 16 porq son 16 muestras
	sensor_der_avg = sensor_der_sum / 16;
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) { // Rutina de antención a la interrupción de buffer a mitad
// Promediamos la primera mitad del buffer (el primer bloque de tamaño mínimo)
	promediar(&dma_buffer[0]);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) { // Rutina de antención a la interrupción de buffer a tope
// Promediamos la segunda mitad del buffer (el segundo bloque de tamaño mínimo)
	promediar(&dma_buffer[32]);
}
void correccion_avanzar(void) {
	// corrección para el sensor izquierdo
	if ((sensor_izq_avg < margen_i) && (margen_d < sensor_der_avg)) {
		apagar_derecha();  // apagar motor derecho
	} else if ((margen_i < sensor_izq_avg) && (sensor_der_avg < margen_d)) { // avanzar con ambos motores
		apagar_izquierda();  //apaga motor izquierdo
	} else {
		avanzar();
	}
}
void avanzar(void) {
	HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_SET);
	TIM3->CCR3 = v_media_izq; // rueda a velocidad media (condigurable)
	TIM3->CCR4 = v_media_der; // rueda a velocidad media
}

void apagar_izquierda(void) {
	HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_RESET);
	TIM3->CCR3 = v_min; // rueda a velocidad media (condigurable)
	TIM3->CCR4 = 0; // rueda a velocidad media
}

void apagar_derecha(void) {
	HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_SET);
	TIM3->CCR3 = 0; // rueda a velocidad media (condigurable)
	TIM3->CCR4 = v_min; // rueda a velocidad media
}

void mini_retroceso(void) {
	setMotorIzquierdo(retroceso);
	setMotorDerecho(retroceso);
	HAL_Delay(tiempo_muerto_retroceso);
}
void mini_avance(void) {
	setMotorIzquierdo(avance);
	setMotorDerecho(avance);
	HAL_Delay(tiempo_muerto);
}

void ejecutarGiro(uint8_t giro) {
	switch (giro) {
	case adelante:
		mini_avance(); //este es para q siga recto y no corrija mal
		break;
	case derecha:
		if (contador_giros == 0) {
			contador_giros = contador_giros + 1;
			setMotorIzquierdo(avance);
			setMotorDerecho(avance);
			HAL_Delay(tiempo_muerto_avanzar);
			girando = 1;
			setMotorIzquierdo(avance);
			setMotorDerecho(retroceso);
			HAL_Delay(tiempo_giro90_der);
			mini_avance();
			HAL_Delay(tiempo_muerto);
			HAL_Delay(tiempo_muerto);
		} else {
			contador_giros = contador_giros + 1;
			mini_retroceso();
			girando = 1;
			setMotorIzquierdo(avance);
			setMotorDerecho(retroceso);
			HAL_Delay(tiempo_giro90_2);
			mini_avance();
		}
		break;
	case izquierda:
		if (contador_giros == 0) {
			contador_giros = contador_giros + 1;
			setMotorIzquierdo(avance);
			setMotorDerecho(avance);
			HAL_Delay(tiempo_muerto_avanzar);
			girando = 1;
			setMotorIzquierdo(retroceso);
			setMotorDerecho(avance);
			HAL_Delay(tiempo_giro90_izq);
			mini_avance();
			HAL_Delay(tiempo_muerto);
			HAL_Delay(tiempo_muerto);
		} else {
			contador_giros = contador_giros + 1;
			HAL_Delay(tiempo_muerto_avanzar);
			girando = 1;
			setMotorIzquierdo(retroceso);
			setMotorDerecho(avance);
			HAL_Delay(tiempo_giro90_2);
			mini_avance();
		}
		break;

	case giro_180:
		contador_giros = contador_giros + 1;
		girando = 1;
		setMotorIzquierdo(avance);
		setMotorDerecho(retroceso);
		HAL_Delay(tiempo_giro180);
		mini_avance();
		break;

	}
}

void setMotorIzquierdo(uint8_t modo) {

	TIM3->CCR3 = v_media; // rueda a velocidad media

	switch (modo) {
	case avance:
		HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_SET);
		break;
	case retroceso:
		HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_RESET);
		break;
	}
}

void setMotorDerecho(uint8_t modo) {

	TIM3->CCR4 = v_media; // rueda a velocidad media

	switch (modo) {
	case avance:
		HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_SET);
		break;
	case retroceso:
		HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_RESET);
		break;

	}
}

uint8_t act_pared(uint8_t *pared, uint8_t ubicacion, uint8_t orientacion_actual) { // este CODIGO ES SUPONIENDO Q YA SE DETECTO LA PARED

	// actualizamos el valor de pared según la orientación
	switch (orientacion_actual) { //SE PONE 0X08 PORQ ES HEXADECIMAL, SI NO LO PONES ESTA EN OTRA BASE, ME HIZO RE CONFUNDIR
	case norte: // TAMBIEN USO EL |= PARA Q SI DETECTA UNA PARED YA INICIALIZADA NO LA SUME Y ACUMULE UN CARRY , SI NO Q HAGA LA OR
		pared[ubicacion] |= 0x08;  // suma 8 (1000 en binario)
		break;
	case este:
		pared[ubicacion] |= 0x04;  // suma 4 (0100 en binario)
		break;
	case sur:
		pared[ubicacion] |= 0x02;  // suma 2 (0010 en binario)
		break;
	case oeste:
		pared[ubicacion] |= 0x01;  // suma 1 (0001 en binario)
		break;
	default:
		return 0; // Orientación no válida
	}
	return pared[ubicacion];  // devolvés el valor actualizado
}

void act_pesos(uint8_t *pared, uint8_t *peso) {
	uint8_t minimo_peso_vecino;
	for (int j = 0; j < 15; j++) {
		for (int i = 0; i < cant_casilleros - 1; i++) {
			minimo_peso_vecino = 100;
			if (((i + 4 < cant_casilleros) && (pared[i] & 0x08) == 0)) { //mirar la el vecino de arriba si el bit 3 es 0 y si i es menor a 12 (es decir q no tiene pared superior) PORQUE SI NO NO PUEDE CALCULAR EL VECINO DE ARRIBA PORQ SERIA I+4 Y SI I ES 12 O MAS, I+4 VA A DAR 16 O MAS, Q NO EXISTE
				if (peso[i + 4] < minimo_peso_vecino)
					minimo_peso_vecino = peso[i + 4];
			}

			if (((!(i == 3 || i == 7 || i == 11 || i == 15)) && (pared[i] & 0x01) == 0)) { //ideam mirar el vecino derecha si el bit 2 es 0 y si el numero es distinto a 3 7 11 y 15 PORQ EN ESE CASO NO TIENE VECINO A LA derecha (LIMITE DEL MAPA)
				if (peso[i + 1] < minimo_peso_vecino)
					minimo_peso_vecino = peso[i + 1];
			}

			if (((i >= 4) && (pared[i] & 0x02) == 0)) { //ideam al primero, mira el vecino de abajo en el caso de q i sea mayor o igual a 4
				if (peso[i - 4] < minimo_peso_vecino)
					minimo_peso_vecino = peso[i - 4];
			}

			if (((pared[i] & 0x04) == 0) && (!(i == 0 || i == 4 || i == 8 || i == 12))) { //ideam al dos
				if (peso[i - 1] < minimo_peso_vecino)
					minimo_peso_vecino = peso[i - 1];
			}

			peso[i] = minimo_peso_vecino + 1;
		}
	}
}

uint8_t calculo_minimo_peso(uint8_t *peso, uint8_t *pared, uint8_t ubicacion, uint8_t orientacion_actual) {
	uint8_t minimo_peso = 15;
	if (ubicacion == 15){
		casilla_n = 15;
		return casilla_n;
	}
	else{

	switch (orientacion_actual) {
	case norte:
		if (((peso[ubicacion + 4] < minimo_peso) && ((pared[ubicacion] & 0x08) == 0) && (ubicacion + 4 < cant_casilleros))) {
			minimo_peso = peso[ubicacion + 4];
			casilla_n = ubicacion + 4;
		}
		if (((peso[ubicacion + 1] < minimo_peso) && ((pared[ubicacion] & 0x01) == 0) && !(ubicacion == 3 || ubicacion == 7 || ubicacion == 11 || ubicacion == 15))) { // el signo de admiracion niega y convierte en booleana ubicacion
			minimo_peso = peso[ubicacion + 1];
			casilla_n = ubicacion + 1;
		}
		if (((peso[ubicacion - 1] < minimo_peso) && ((pared[ubicacion] & 0x04) == 0) && !(ubicacion == 3 || ubicacion == 7 || ubicacion == 11 || ubicacion == 15))) {
			minimo_peso = peso[ubicacion - 1];
			casilla_n = ubicacion - 1;
		}
		if (((peso[ubicacion - 4] < minimo_peso) && ((pared[ubicacion] & 0x02) == 0) && (4 <= ubicacion))) {
			minimo_peso = peso[ubicacion - 4];
			casilla_n = ubicacion - 4;
		}
		return casilla_n;
		break;
	case sur:
		if (((peso[ubicacion - 4] < minimo_peso) && ((pared[ubicacion] & 0x02) == 0) && (4 <= ubicacion))) {
			minimo_peso = peso[ubicacion - 4];
			casilla_n = ubicacion - 4;
		}

		if (((peso[ubicacion + 1] < minimo_peso) && ((pared[ubicacion] & 0x01) == 0) && !(ubicacion == 3 || ubicacion == 7 || ubicacion == 11 || ubicacion == 15))) { // el signo de admiracion niega y convierte en booleana ubicacion
			minimo_peso = peso[ubicacion + 1];
			casilla_n = ubicacion + 1;
		}
		if (((peso[ubicacion - 1] < minimo_peso) && ((pared[ubicacion] & 0x04) == 0) && !(ubicacion == 3 || ubicacion == 7 || ubicacion == 11 || ubicacion == 15))) {
			minimo_peso = peso[ubicacion - 1];
			casilla_n = ubicacion - 1;
		}

		if (((peso[ubicacion + 4] < minimo_peso) && ((pared[ubicacion] & 0x08) == 0) && (ubicacion + 4 < cant_casilleros))) {
			minimo_peso = peso[ubicacion + 4];
			casilla_n = ubicacion + 4;
		}
		return casilla_n;
		break;
	case oeste:
		if (((peso[ubicacion + 1] < minimo_peso) && ((pared[ubicacion] & 0x01) == 0) && !(ubicacion == 3 || ubicacion == 7 || ubicacion == 11 || ubicacion == 15))) { // el signo de admiracion niega y convierte en booleana ubicacion
			minimo_peso = peso[ubicacion + 1];
			casilla_n = ubicacion + 1;
		}
		if (((peso[ubicacion + 4] < minimo_peso) && ((pared[ubicacion] & 0x08) == 0) && (ubicacion + 4 < cant_casilleros))) {
			minimo_peso = peso[ubicacion + 4];
			casilla_n = ubicacion + 4;
		}

		if (((peso[ubicacion - 4] < minimo_peso) && ((pared[ubicacion] & 0x02) == 0) && (4 <= ubicacion))) {
			minimo_peso = peso[ubicacion - 4];
			casilla_n = ubicacion - 4;
		}

		if (((peso[ubicacion - 1] < minimo_peso) && ((pared[ubicacion] & 0x04) == 0) && !(ubicacion == 3 || ubicacion == 7 || ubicacion == 11 || ubicacion == 15))) {
			minimo_peso = peso[ubicacion - 1];
			casilla_n = ubicacion - 1;
		}
		return casilla_n;
		break;
	case este:
		if (((peso[ubicacion - 1] < minimo_peso) && ((pared[ubicacion] & 0x04) == 0) && !(ubicacion == 3 || ubicacion == 7 || ubicacion == 11 || ubicacion == 15))) {
			minimo_peso = peso[ubicacion - 1];
			casilla_n = ubicacion - 1;
		}
		if (((peso[ubicacion + 4] < minimo_peso) && ((pared[ubicacion] & 0x08) == 0) && (ubicacion + 4 < cant_casilleros))) {
			minimo_peso = peso[ubicacion + 4];
			casilla_n = ubicacion + 4;
		}
		if (((peso[ubicacion - 4] < minimo_peso) && ((pared[ubicacion] & 0x02) == 0) && (4 <= ubicacion))) {
			minimo_peso = peso[ubicacion - 4];
			casilla_n = ubicacion - 4;
		}
		if (((peso[ubicacion + 1] < minimo_peso) && ((pared[ubicacion] & 0x01) == 0) && !(ubicacion == 3 || ubicacion == 7 || ubicacion == 11 || ubicacion == 15))) { // el signo de admiracion niega y convierte en booleana ubicacion
			minimo_peso = peso[ubicacion + 1];
			casilla_n = ubicacion + 1;
		}
		return casilla_n;
		break;
	default:
		return 100;
	}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

}

void envio_ubicacion(uint8_t ubicacion,uint8_t casilla_n) {
	if (casilla_n == ubicacion){
		sprintf(mensaje, "Ubicacion: %d", ubicacion);
		strcat(mensaje, "\r\n");
		HAL_UART_Transmit(&huart5, (uint8_t*) mensaje, sizeof(mensaje), delay);
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
