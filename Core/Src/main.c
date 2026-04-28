/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include <stdlib.h>
#include "NanoEdgeAI.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DMA 0
#define TIMER_INTERRUPT 1

#define CAPTURE_MODE 0
#define CLASSIFY_MODE 1
#define CRONOMETER_MODE 2

// MODE ya no se define con #define fijo — se selecciona en runtime desde el menu
#define DAC_MODE TIMER_INTERRUPT
#define ADC_MODE TIMER_INTERRUPT

// Valores para DAC y ADC:
#define ADC_SAMP_FREQ 100
#define DAC_SIGNAL_FREQ_INICIAL 4
#define PRESCALER 199
#define CLK 4000000
// NIVEL_RUIDO se configura en runtime desde el menu (opcion 4) — ver variable global nivel_ruido

#define N_FREQS 4
#define PERIOD_TIMER_ADC  (CLK / ((PRESCALER + 1) * ADC_SAMP_FREQ)) - 1

/*----------------------------------------------------------------------------*/
// ESTO SIRVE PARA QUE FUNCIONE EL SCANF!!!
#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif


GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;

  __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);

  HAL_UART_Receive(&hcom_uart[COM1], &ch, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&hcom_uart[COM1], &ch, 1, HAL_MAX_DELAY);

  return ch;
}
int _read(int file, char *ptr, int len)
{
  (void)file;

  uint8_t ch;
  HAL_UART_Receive(&hcom_uart[COM1], &ch, 1, HAL_MAX_DELAY);

  *ptr = ch;
  return 1;
}

/*----------------------------------------------------------------------------*/


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
DMA_NodeTypeDef Node_GPDMA1_Channel1;
DMA_QListTypeDef List_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel1;

DAC_HandleTypeDef hdac1;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */

const uint32_t signal_sen[128]      = {2048, 2148, 2248, 2348, 2447, 2545, 2642, 2737, 2831, 2923, 3012, 3100, 3185, 3267, 3346, 3422,3495, 3564, 3630, 3692, 3750, 3803, 3853, 3898, 3939, 3975, 4006, 4033, 4055, 4072, 4085, 4092,	4095, 4092, 4085, 4072, 4055, 4033, 4006, 3975,	3939, 3898, 3853, 3803, 3750, 3692, 3630, 3564,	3495, 3422, 3346, 3267, 3185, 3100, 3012, 2923,	2831, 2737, 2642, 2545, 2447, 2348, 2248, 2148,	2048, 1947, 1847, 1747, 1648, 1550, 1453, 1358,	1264, 1172, 1083,  995,  910,  828,  749,  673,	 600,  531,  465,  403,  345,  292,  242,  197,	 156,  120,   89,   62,   40,   23,   10,    3,	   1,    3,   10,   23,   40,   62,   89,  120, 156,  197,  242,  292,  345,  403,  465,  531,	 600,  673,  749,  828,  910,  995, 1083, 1172,	1264, 1358, 1453, 1550, 1648, 1747, 1847, 1947};
const uint32_t signal_cuadrada[128]   = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095	};
const uint32_t signal_triangular[128] = {1,33,65,97,129,161,193,225,257,289,321,353,385,417,449,481,513,545,577,609,641,673,705,737,769,801,833,865,897,929,961,993,1025,1057,1089,1121,1153,1185,1217,1249,1281,1313,1345,1377,1409,1441,1473,1505,1537,1569,1601,1633,1665,1697,1729,1761,1793,1825,1857,1889,1921,1953,1985,2017,2049,2081,2113,2145,2177,2209,2241,2273,2305,2337,2369,2401,2433,2465,2497,2529,2561,2593,2625,2657,2689,2721,2753,2785,2817,2849,2881,2913,2945,2977,3009,3041,3073,3105,3137,3169,3201,3233,3265,3297,3329,3361,3393,3425,3457,3489,3521,3553,3585,3617,3649,3681,3713,3745,3777,3809,3841,3873,3905,3937,3969,4001,4033,4065};

const uint32_t *signals[3] = { signal_sen, signal_cuadrada, signal_triangular };
volatile uint8_t signal_idx = 0; // 0, 1 o 2

volatile uint32_t boton_press_tick = 0;
volatile uint8_t boton_presionado = 0;

// Modo de operacion seleccionado en runtime por menu UART
volatile uint8_t modo_actual = 255; // 255 = sin modo seleccionado

// Nivel de ruido configurable desde el menu (0 a 4095). Usada en ISR de TIM2.
volatile uint32_t nivel_ruido = 100;


uint32_t adc_buf[256];
uint32_t tiempo;
uint32_t temp_flag;
uint8_t buffer_actual;
uint8_t buffer_completo;
int32_t buffers_restantes = 0; // Para modo captura: cuantos buffers quedan por adquirir


const uint32_t frecuencias_dac[] = {1, 2, 8, 16};
volatile uint8_t freq_idx = 0; // arranca en 4Hz (índice 1)

// Periodo del timer del dac considerando 128 Muestras x ciclo
int getTIMER_DAC ( int signal_freq )
{
	return (CLK / ((PRESCALER + 1) * signal_freq * 128)) - 1;
}


int getRandom (int amplitud)
{
	if(amplitud == 0){return 0;}
	uint32_t rnd;
	HAL_RNG_GenerateRandomNumber(&hrng, &rnd);
	return (rnd % amplitud) - amplitud/2;
}





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPDMA1_Init(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_ICACHE_Init(void);
static void MX_TIM15_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */

static char * convertir_a_tiempo(int s);
static void menu_principal(void);
static uint8_t uart_getchar_timeout(uint32_t timeout_ms);

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

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPDMA1_Init();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_ICACHE_Init();
  MX_TIM15_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  neai_classification_init();

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  printf("\n\n\r --------- Comienzo de programa --------- \n\n\r");


  /* SECCION DAC CONFIGURADO POR INTERRUPCIONES O POR DMA */
  #if DAC_MODE == DMA
    // DAC: Configurado por DMA
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t*)signals[signal_idx], 128, DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(&htim2);

  #elif DAC_MODE == TIMER_INTERRUPT
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim2);
  #endif

  /* SECCION ADC CONFIGURADO POR INTERRUPCIONES O POR DMA */
  buffer_actual = 0;
  buffer_completo = 0;

  #if ADC_MODE == DMA
	// ADC: configurado por DMA
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 256);
	HAL_TIM_Base_Start(&htim15);

  #elif ADC_MODE == TIMER_INTERRUPT
	HAL_TIM_Base_Start_IT(&htim15);
  #endif

  /* TIMER CONFIGURADO POR INTERRUPCIONES */
  tiempo = 0;
  temp_flag = 0;
  HAL_TIM_Base_Start_IT(&htim4);

  // Mostrar menu y esperar seleccion de modo
  menu_principal();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	/* MODO DE CAPTURA DE DATOS... Para entrenar NanoEdgeAI */
	if(modo_actual == CAPTURE_MODE)
	{
		if(buffer_completo == 1){

		    uint32_t buffer_copia[128];
		    uint32_t *ptr = (buffer_actual == 1) ? &adc_buf[0] : &adc_buf[128];
		    memcpy(buffer_copia, ptr, 128 * sizeof(uint32_t));
		    buffer_completo = 0;
		    char *t = convertir_a_tiempo(tiempo);

		    printf("\r\n { \"ts\":%s , \"Muestras\": ",t);
			for(int i =0; i < 128;i++){
				printf("%lu, ", buffer_copia[i]);
			}
			printf("}\n");

			// Descontar buffer y salir al menu si terminamos
			buffers_restantes--;
			if(buffers_restantes == 0){
				printf("\r\n--- Adquisicion completa. Volviendo al menu ---\r\n");
				menu_principal();
			}
		}
	}

	/* MODO DE CLASIFICACION */
	else if(modo_actual == CLASSIFY_MODE)
	{
		// Verificar si llego una tecla (no bloqueante, timeout 0)
		uint8_t tecla = uart_getchar_timeout(0);
		if(tecla == 'q' || tecla == 'Q'){
			printf("\r\n--- Clasificacion detenida. Volviendo al menu ---\r\n");
			menu_principal();
		}

		if(buffer_completo == 1){
			int id_class;
			float similarities[neai_get_number_of_classes()];
			float input_float[128];

			if(buffer_actual == 1)
			{
				for(int i = 0; i < 128; i++)
				{
				    input_float[i] = (float)adc_buf[i];
				}

			}else{
				for(int i = 0; i < 128; i++)
				{
					input_float[i] = (float)adc_buf[i+128];
				}
			}
			char *t = convertir_a_tiempo(tiempo);
			neai_classification(input_float,similarities,&id_class);
			printf("\r%s: Señal clasificada como %s con %d%% de confianza  [q=salir]     ", t, neai_get_class_name(id_class), (int)(similarities[id_class] * 100));
			fflush(stdout);
			buffer_completo = 0;
		}
	}

	/* CRONOMETRO */
	else if(modo_actual == CRONOMETER_MODE)
	{
		// Verificar si llego una tecla (no bloqueante, timeout 0)
		uint8_t tecla = uart_getchar_timeout(0);
		if(tecla == 'q' || tecla == 'Q'){
			printf("\r\n--- Cronometro detenido. Volviendo al menu ---\r\n");
			menu_principal();
		}

		if(temp_flag == 1){
			char *t = convertir_a_tiempo(tiempo);
			printf("\r%s  [q=salir]                                                        ", t);
			fflush(stdout);
			temp_flag=0;
		}
	}


	// Detección de pulsación corta vs larga
	if(boton_presionado && !BSP_PB_GetState(BUTTON_USER))
	{
		boton_presionado = 0;
		uint32_t duracion = HAL_GetTick() - boton_press_tick;

		if(duracion > 1000)
		{
			// Pulsacion larga cambia forma de onda

			signal_idx = (signal_idx + 1) % 3;
			#if DAC_MODE == DMA
				HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t*)signals[signal_idx], 128, DAC_ALIGN_12B_R);
			#endif
		}
		else
		{
			// Pulsacion corta cambia frecuencia
			freq_idx = (freq_idx + 1) % N_FREQS;

			// Cambio de frecuencia el timer del DAC:
			HAL_TIM_Base_Stop_IT(&htim2);
			__HAL_TIM_SET_AUTORELOAD(&htim2, getTIMER_DAC(frecuencias_dac[freq_idx]));
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			HAL_TIM_Base_Start_IT(&htim2);
		}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T15_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_5CYCLE;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	#if ADC_MODE == TIMER_INTERRUPT
		hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		if (HAL_ADC_Init(&hadc1) != HAL_OK)
		{
		  Error_Handler();
		}
	#endif

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};
  DAC_AutonomousModeConfTypeDef sAutonomousMode = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Autonomous Mode
  */
  sAutonomousMode.AutonomousModeState = DAC_AUTONOMOUS_MODE_DISABLE;
  if (HAL_DACEx_SetConfigAutonomousMode(&hdac1, &sAutonomousMode) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  __HAL_TIM_SET_AUTORELOAD(&htim2, getTIMER_DAC(frecuencias_dac[freq_idx]));

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim15.Init.Prescaler = 199;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 199;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  // Seteo la frecuencia de muestreo del ADC. Lo hago aca para poder cambiarla por SW
  __HAL_TIM_SET_AUTORELOAD(&htim15, PERIOD_TIMER_ADC);

  /* USER CODE END TIM15_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PG7 PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    buffer_completo = 1;
    buffer_actual = 1; // primera mitad lista
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    buffer_completo = 1;
    buffer_actual = 0; // segunda mitad lista
}

void BSP_PB_Callback(Button_TypeDef Button)
{
    if(Button == BUTTON_USER)
        {
            boton_press_tick = HAL_GetTick();
            boton_presionado = 1;
        }

}

static char* convertir_a_tiempo (int s)
{
	int segundos = s % 60;
	int minutos = (s/60) % 60;
	int horas = ((s/60)/60) % 24;

	static char tiempo_formateado[9];
	sprintf(tiempo_formateado,"%02d:%02d:%02d",horas,minutos,segundos);
	return tiempo_formateado;

}

// Recibe un caracter por UART con timeout en ms. Retorna 0 si no llego nada.
static uint8_t uart_getchar_timeout(uint32_t timeout_ms)
{
    uint8_t ch = 0;
    HAL_StatusTypeDef status = HAL_UART_Receive(&hcom_uart[COM1], &ch, 1, timeout_ms);
    if(status == HAL_OK){
        return ch;
    }
    return 0;
}

// Menu principal bloqueante — sale cuando el usuario selecciona un modo valido
static void menu_principal(void)
{
    modo_actual = 255; // sin modo
    buffer_completo = 0; // descartar buffers pendientes

    while(1)
    {
        printf("\r\n");
        printf("========================================\r\n");
        printf("   MENU PRINCIPAL\r\n");
        printf("========================================\r\n");
        printf("  1 - Adquisicion (captura de datos)\r\n");
        printf("  2 - Clasificacion\r\n");
        printf("  3 - Cronometro\r\n");
        printf("  4 - Configurar nivel de ruido (actual: %lu)\r\n", nivel_ruido);
        printf("========================================\r\n");
        printf("Seleccione una opcion: ");
        fflush(stdout);

        // Esperar tecla (bloqueante, esta bien porque estamos en el menu)
        uint8_t opcion = uart_getchar_timeout(HAL_MAX_DELAY);
        printf("%c\r\n", opcion); // echo

        if(opcion == '1')
        {
            // Pedir cantidad de buffers a adquirir
            printf("\r\nCantidad de buffers a adquirir (1-999): ");
            fflush(stdout);

            char input[8];
            int n = 0;
            while(n < 7)
            {
                uint8_t c = uart_getchar_timeout(HAL_MAX_DELAY);
                if(c == '\r' || c == '\n') break;
                if(c == 8 || c == 127) { // backspace
                    if(n > 0){ n--; printf("\b \b"); fflush(stdout); }
                    continue;
                }
                if(c >= '0' && c <= '9'){
                    input[n++] = c;
                    printf("%c", c); fflush(stdout);
                }
            }
            input[n] = '\0';
            printf("\r\n");

            int cantidad = atoi(input);
            if(cantidad <= 0){
                printf("Cantidad invalida. Intente de nuevo.\r\n");
                continue;
            }

            buffers_restantes = cantidad;
            buffer_completo = 0;
            printf("Adquiriendo %d buffers... (iniciando)\r\n", cantidad);
            modo_actual = CAPTURE_MODE;
            return;
        }
        else if(opcion == '2')
        {
            printf("\r\nModo Clasificacion activo. Presione 'q' para salir.\r\n");
            buffer_completo = 0;
            modo_actual = CLASSIFY_MODE;
            return;
        }
        else if(opcion == '3')
        {
            printf("\r\nModo Cronometro activo. Presione 'q' para salir.\r\n");
            temp_flag = 0;
            modo_actual = CRONOMETER_MODE;
            return;
        }
        else if(opcion == '4')
        {
            printf("\r\nNivel de ruido actual: %lu (0 = sin ruido, 4095 = maximo)\r\n", nivel_ruido);
            printf("Nuevo valor (0-4095): ");
            fflush(stdout);

            char input[8];
            int n = 0;
            while(n < 7)
            {
                uint8_t c = uart_getchar_timeout(HAL_MAX_DELAY);
                if(c == '\r' || c == '\n') break;
                if(c == 8 || c == 127) { // backspace
                    if(n > 0){ n--; printf("\b \b"); fflush(stdout); }
                    continue;
                }
                if(c >= '0' && c <= '9'){
                    input[n++] = c;
                    printf("%c", c); fflush(stdout);
                }
            }
            input[n] = '\0';
            printf("\r\n");

            if(n == 0){
                printf("Sin cambios.\r\n");
            } else {
                int valor = atoi(input);
                if(valor < 0 || valor > 4095){
                    printf("Valor invalido. Debe estar entre 0 y 4095.\r\n");
                } else {
                    nivel_ruido = (uint32_t)valor;
                    printf("Nivel de ruido actualizado a %lu.\r\n", nivel_ruido);
                }
            }
            // Volver a mostrar el menu sin salir del while
            continue;
        }
        else
        {
            printf("Opcion no valida.\r\n");
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  if(htim->Instance == TIM2)
  {
	// Este timer controla el  DAC
	// Si esta configurado por DMA en realidad la interrupcion no hace nada

	#if DAC_MODE == TIMER_INTERRUPT
	  static uint8_t idx = 0;

	  int32_t numero = (int32_t)signals[signal_idx][idx] + (int32_t)getRandom(nivel_ruido);
	  uint32_t numero_saturacion = (numero>4095) ? 4095 : ((numero<0)? 0 : numero );

	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,numero_saturacion);
	  idx++;
	  if(idx >= 128) idx = 0;
    #endif
  }

  if(htim->Instance == TIM4)
  {
	  // Este timer es para llevar el cronometro interrumpe una vez por segundo
	  tiempo++;
	  temp_flag=1;
  }
  if(htim->Instance == TIM15)
  {
	  // Este timer controla el DMA del ADC

	#if ADC_MODE == TIMER_INTERRUPT

	  static uint16_t idx = 0;
	  /* ADC */
	  HAL_ADC_Start(&hadc1);		// Iniciar conversión
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Esperar a que termine
	  uint32_t raw = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);
	  adc_buf[idx] = raw;
	  idx++;
	  if(idx == 128)
	  {
		  buffer_actual = 1;
		  buffer_completo = 1;
	  }
	  else if (idx == 256)
	  {
		  buffer_actual = 0;
		  buffer_completo = 1;
		  idx = 0;
	  }

	#endif

  }


  /* USER CODE END Callback 1 */
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
#ifdef USE_FULL_ASSERT
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
