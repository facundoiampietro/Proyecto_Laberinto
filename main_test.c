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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NORTE 0 // Direcciones hacia donde puede estar mirando el autito
#define ESTE  1
#define SUR   2
#define OESTE 3

#define ADELANTE 0 // Tipos de giro que puede hacer
#define IZQUIERDA 1
#define DERECHA  2
#define GIRO_180   3

#define margen 500 // adcs

#define AVANCE     0b01
#define RETROCESO  0b10
#define FRENO      0b00

#define v_max 63999
#define v_media 32000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
// Los valores asignados definen el estado inicial del robot, i.e casilla cero, mirando al norte. De esta forma ira hacia casilla 4
uint8_t ubicacion = 0;    //defino ubicacion (va a ser un numero entre 0 y 15)
uint8_t orientacion_actual = NORTE;
uint8_t orientacion_n = NORTE; 
uint8_t casilla_n = 4;   // la casilla a la cual hay q ir
uint8_t giro;
uint8_t peso[16];
uint8_t pared[16];

//Defino para poder probar el código por partes
uint8_t modo_test = 1;     // 1: avance, 2: giro, 3: línea
uint8_t contador = 0;      // para contar líneas detectadas


/* USER CODE BEGIN PV */
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
/* USER CODE BEGIN PFP */
uint8_t obtener_orientacion_N(uint8_t casilla_actual, uint8_t casilla_n); //Devuelve la orientacion (NORTE, ESTE, SUR, OESTE) que hay que tomar para ir desde la casilla_actual a la casilla_n.
uint8_t obtenerGiro(uint8_t orientacion_actual, uint8_t orientacion_n); //Devuelve que tipo de giro hay que hacer para pasar de la orientacion actual a la deseada.
uint8_t actualizo_ubicacion(uint8_t ubicacion, uint8_t orientacion_n); //Donde estoy en funcion de la orientacion y donde estaba ubicado
uint8_t metodo_de_llenado(uint8_t ubicacion, uint8_t orientacion_n, bool verificacion_pared);
void correccion_avanzar(void);
void avanzar(void);
void apagar_derecha(void);
void apagar_izquierda(void);
void setMotorIzquierdo(uint8_t modo);
void setMotorDerecho(uint8_t modo);
void ejecutarGiro(uint8_t giro);
bool verificar_sensor(void);
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
  /* USER CODE BEGIN 2 */

	peso[15] = 0;  //inicialización de pesos (distancia a la meta)
	peso[11] = peso[14] = 1;
	peso[7] = peso[10] = peso[13] = 2;
	peso[3] = peso[6] = peso[9] = peso[12] = 3;
	peso[2] = peso[5] = peso[8] = 4;
	peso[1] = peso[4] = 5;
	peso[0] = 6;

	//paredes de cada casilla (8=1000=pared en Norte, 4=0100=pared en Este, 2=0010=pared en Sur, 1=0001=pared en Oeste)
	pared[0] = 6; //se suma sur y este
	pared[1] = pared[2] = 2; //solo sur
	pared[3] = 3;  //se suma sur y oeste
	pared[7] = pared[11] = 1; // solo oeste
	pared[15] = 9;  //se suma norte y oeste
	pared[14] = pared[13] = 8; //solo norte
	pared[12] = 12; //se suma mprte y este
	pared[4] = pared[8] = 4; //solo este
	pared[5] = pared[6] = pared[9] = pared[10] = 0;

	HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_RESET); // INICIALIZACION EN AVANZAR
	HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Inicio de la modulación PWM, rueda izquierda
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // Inicio de la modulación PWM, rueda derecha
	TIM4->CCR3 = v_media; // rueda a velocidad media (condigurable)
	TIM4->CCR4 = v_media; // rueda a velocidad media

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */
		avanzar();
		if (!verificar_sensor()) {//cambio de casilla
			ubicacion = actualizo_ubicacion(ubicacion, orientacion_actual);
			//aca estaria el metodo de llenado q me da casilla_n
			orientacion_n = obtener_orientacion_N(ubicacion, casilla_n);
			giro = obtenerGiro(orientacion_actual, orientacion_n);
			orientacion_actual = orientacion_n;  //actualizo la orientacion
			ejecutarGiro(giro);
		}

		//faltaria el if de la pared

  /* Este es el modo de "prueba" por partes
    switch (modo_test) {
    case 1:
      correccion_avanzar();
      HAL_Delay(50);
      break;
    case 2:
      ejecutarGiro(DERECHA);
      HAL_Delay(1000);
      break;
    case 3:
      if (verificar_sensor()) {
        contador++;
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);
        HAL_Delay(300);
      }
      break;
  }
  */
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, m0_izquierda_Pin|m1_izquierda_Pin|m0_derecha_Pin|m1_derecha_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : m0_izquierda_Pin m1_izquierda_Pin m0_derecha_Pin m1_derecha_Pin */
  GPIO_InitStruct.Pin = m0_izquierda_Pin|m1_izquierda_Pin|m0_derecha_Pin|m1_derecha_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor_frontal_Pin sensor_linea_Pin */
  GPIO_InitStruct.Pin = sensor_frontal_Pin|sensor_linea_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint8_t obtener_orientacion_N(uint8_t ubicacion, uint8_t casilla_n) { // Devuelve la dirección hacia donde hay que ir según la diferencia entre casillas
	if (casilla_n == ubicacion + 1)
		return OESTE;
	if (casilla_n == ubicacion - 1)
		return ESTE;
	if (casilla_n == ubicacion + 4)
		return NORTE;
	if (casilla_n == ubicacion - 4)
		return SUR;

	return 100; // Movimiento no válido (no adyacente o fuera del tablero)
}

uint8_t obtenerGiro(uint8_t orientacion_actual, uint8_t orientacion_n) { // Calcula el giro que debe hacer el autito para pasar de su orientación actual a la deseada
	int diferencia = (orientacion_n - orientacion_actual + 4) % 4; //el %4 se queda con el resto de la divsion por 4

	switch (diferencia) {
	case 0:
		return ADELANTE;
	case 1:
		return DERECHA;
	case 2:
		return GIRO_180;
	case 3:
		return IZQUIERDA;
	default:
		return 100; // Error
	}
}
uint8_t actualizo_ubicacion(uint8_t ubicacion, uint8_t orientacion_n) {

	switch (orientacion_n) {
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
// Dividimos las sumas para obtener el promedio
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
	if (sensor_izq_avg < margen) {
		apagar_derecha();  // apagar motor derecho
	} else {
		avanzar();         // avanzar con ambos motores
	}

	// corrección para el sensor derecho
	if (sensor_der_avg < margen) {
		apagar_izquierda();  // apagar motor izquierdo
	} else {
		avanzar();           // avanzar con ambos motores
	}
}

void avanzar(void) {
	HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_SET);
	TIM4->CCR3 = v_media; // rueda a velocidad media (condigurable)
	TIM4->CCR4 = v_media; // rueda a velocidad media
}

void apagar_derecha(void) {
	HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_RESET); //pongo la derecha trabada
	TIM4->CCR3 = v_media; // rueda a velocidad media (condigurable)
	TIM4->CCR4 = 0; // rueda a velocidad media
}

void apagar_izquierda(void) {
	HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_RESET);//pongo la izquierda trabada
	HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_SET);
	TIM4->CCR3 = 0; // rueda a velocidad media (condigurable)
	TIM4->CCR4 = v_media ; // rueda a velocidad media
}
void ejecutarGiro(uint8_t giro) {
	switch (giro) {
	case ADELANTE:
		setMotorIzquierdo(AVANCE);
		setMotorDerecho(AVANCE);
		//delay
		break;

	case DERECHA:
		setMotorIzquierdo(AVANCE);
		setMotorDerecho(RETROCESO);
		//delay
		break;

	case IZQUIERDA:
		setMotorIzquierdo(RETROCESO);
		setMotorDerecho(AVANCE);
		//delay
		break;

	case GIRO_180:
		setMotorIzquierdo(AVANCE);
		setMotorDerecho(RETROCESO);
		// delay mas largo
		break;

	}
}

void setMotorIzquierdo(uint8_t modo) {

	TIM4->CCR3 = v_max ; // rueda a velocidad MAX

	switch (modo) {
	case AVANCE:
		HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin,	GPIO_PIN_SET);
		break;

	case RETROCESO:
		HAL_GPIO_WritePin(m1_izquierda_GPIO_Port, m1_izquierda_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(m0_izquierda_GPIO_Port, m0_izquierda_Pin, GPIO_PIN_RESET);
		break;
	}
}

void setMotorDerecho(uint8_t modo) {

	TIM4->CCR4 = v_max ; // rueda a velocidad MAX

	switch (modo) {
	case AVANCE:
		HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_SET);
		break;

	case RETROCESO:
		HAL_GPIO_WritePin(m1_derecha_GPIO_Port, m1_derecha_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(m0_derecha_GPIO_Port, m0_derecha_Pin, GPIO_PIN_RESET);
		break;

	}
}

bool verificar_sensor(void){
static GPIO_PinState ultima_lectura_valida = GPIO_PIN_SET;
// Se crean variables para lecturas intermedias
GPIO_PinState lectura1, lectura2;
// Se crea una variable booleana para indicar si hay un pedido
bool pedido = false; // No hay pedido hasta que se pulsa el botón
// Se lee el estado del botón
lectura1 = HAL_GPIO_ReadPin(sensor_linea_GPIO_Port, sensor_linea_Pin);
// Si hubo un cambio
if (lectura1 != ultima_lectura_valida){
// Se espera un tiempo para filtrar los rebotes
HAL_Delay(20); // Retardo de 20 milisegundos
// Se lee nuevamente el estado del botón
lectura2 = HAL_GPIO_ReadPin(sensor_linea_GPIO_Port, sensor_linea_Pin);
// Si ambas lecturas son iguales, se considera una lectura válida
if(lectura2 == lectura1)
ultima_lectura_valida = lectura2;
// Si el botón pasó de liberado a pulsado (1-->0), hubo un pedido de cambio de estado
if (ultima_lectura_valida == GPIO_PIN_RESET)
pedido = true;
}
return pedido;
}

uint8_t metodo_de_llenado(uint8_t ubicacion, uint8_t orientacion_n,
		bool verificacion_pared);

//...

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
