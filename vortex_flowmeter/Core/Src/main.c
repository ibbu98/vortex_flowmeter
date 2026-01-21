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
/* ---------------------------------------------------------------------------- 
--                                                                           --
--              ECEN 5803 Mastering Embedded System Architecture             --
--                  Project 1 Module 6                                       --
--                Microcontroller Firmware                                   --
--                                                                           --
-------------------------------------------------------------------------------
--
--  Designed for:  University of Colorado at Boulder
--            by:  Ibrahim Muadh & Varun Shah
-- 
-- Version: 1.0
-- Date of current revision:  2025-10-12  
-- Target Microcontroller: ST STM32F401RE 
-- Tools used:  STMCubeMX Configurator
--              STMCubeIDE and GCC Compiler
--              Keil MDK IDE and Keil Version 6 ARM Clang compiler
--              ST Nucleo STM32F401RE Board
-- -- 
-- Functional Description: Create a bare metal Cyclic Executive (Super-Loop)
--                          to calculate the flow given voretex frequency
--                          samples from the ADC. Do the following tasks in
--                          the while(1) loop:
--                          1.  Send UART Messages to the PC (Debug monitor)
--                              Include Flow, Frequency, and Temperature info
--                          2.  Print Flow, Frequency, and Temperature info to
--                              the LCD over a SPI port
--                          3.  Read the ADC for flow information
--                          4.  Read the ADC temperature sensor for temperature
--                              and also the reference voltage as a check
--                          5.  Generate a 4-20 mA output on a PWM channel with
--                              rate proportional to flow
--                          6.  Generate a pulse output using a PWM channel with
--                              rate propotional to vortex frequency
--                          7.  Blink the LED at a rate proportional to vortex
--                              frequency
--                          8.  Read the DS1631 temperature sensor on I2C 
--                              (optional)
--
-- System initialization has been created using STMCubeMX. Your task is to
--  complete the code where indicated                         
--
--   Caution:  Make sure ST-LINK is the chosen debugger with either IDE	
--             Be sure to set optimization to -O0	
--             If using STM32CubeIDE, be sure to define STM32CUBE
--             If using KEIL, do not define STM32CUBE
--             This compiler switch necessary to control printf
--	GOOD LUCK!
--  
--
--      Copyright (c) 2025 University of Colorado  All rights reserved.
--
*/                  
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "freq_detect.h"
#include "adc_data.h"
float freq_hz = 0.0f;
float flow_gpm_val = 0.0f;
float temp_c = 0.0f;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef ENABLE_SPI
#include "nhd_0216hz.h"
#endif
#include <stdio.h>

#include "stm32f4xx.h"                  // Device header

#define MAIN
#include "shared.h"
#undef MAIN
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ADC_HandleTypeDef hadc1;

//#ifdef ENABLE_I2C
I2C_HandleTypeDef hi2c1;
//#endif

#ifdef ENABLE_SPI
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim1;
#endif

TIM_HandleTypeDef htim3;

#ifdef ENABLE_UART
UART_HandleTypeDef huart2;
#endif

#ifdef ENABLE_I2C
//I2C address of temperature sensor DS1631
const int temp_addr = 0x90;

// Array storing the Start Convert T command and the Read Temperature command.
char cmd[] = {0x51, 0xAA};

// Array to store the 16-bit temperature data read from the sensor.
char read_temp[2];
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
#ifdef ENABLE_I2C
static void MX_I2C1_Init(void);
#endif
#ifdef ENABLE_SPI
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
#endif
static void MX_TIM3_Init(void);
#ifdef ENABLE_UART
static void MX_USART2_UART_Init(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef ENABLE_UART
#ifdef STM32CUBE
// Redirect printf to UART
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
#else
// Redirect printf to UART
int stdout_putchar(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
#endif
#endif

volatile uint32_t timeTicks = 0;                            /* counts  timeTicks */
/*----------------------------------------------------------------------------
 * SysTick_Handler:
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
	timer0();
    timeTicks++;
	if (timeTicks > 9)
	{
	   HAL_IncTick();  // Adjust HAL_IncTick to 1 ms for all HAL driver timing
	   timeTicks = 0;
	}
}


/*----------------------------------------------------------------------------
 * main: Create Scheduler, Debug Monitor and Blink LED
 *----------------------------------------------------------------------------*/
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
  /* Configure the system clock */
  SystemClock_Config();
/****************      ECEN 5803 add code as indicated   **********************/
               //  Add code to call timer0 function every 100 uS
	
  SysTick_Config(SystemCoreClock / 10000 );/* SysTick now 100 usec interrupts */
	                                         // was 1 s
//  HAL_SuspendTick (void )
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();

  /* Start PWM on TIM3 CH2 (PB5) and CH3 (PB0) */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  /* Set a safe initial duty on both channels */
  __HAL_TIM_SET_AUTORELOAD(&htim3, 4000);     // period
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);



  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
  MX_ADC1_Init();
#ifdef ENABLE_I2C
	MX_I2C1_Init();
#endif
#ifdef ENABLE_SPI
	MX_SPI2_Init();
	MX_TIM1_Init();
#endif
  MX_TIM3_Init();
#ifdef ENABLE_UART
	MX_USART2_UART_Init();
#endif
#ifdef ENABLE_SPI
	HAL_TIM_Base_Start(&htim1); //start counter in free running mode
	init_spi();// initializes the SPI interface
	init_lcd();// initializes the LCD display
	print_lcd("Welcome to");// printing the string to the LCD
	set_cursor(0,1); // moving the cursor to the second line of the LCD
	print_lcd("5803-MESA!!");// printing another string to the LCD
#endif    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  
#ifdef ENABLE_I2C
/****************      ECEN 5803 add code as indicated   **********************/
//  Add code here to read the DS1631 temperature sensor if desired (not require) 
//  as a comparison to the ADC temp sensor




#endif



/*************  Initialization  *************/	


 // Print the initial banner
#ifdef ENABLE_UART
    printf("\r\nHello World!\n\n\r");
#endif
    uint32_t  count = 0;   

// initialize serial buffer pointers
   rx_in_ptr =  rx_buf; /* pointer to the receive in data */
   rx_out_ptr = rx_buf; /* pointer to the receive out data*/
   tx_in_ptr =  tx_buf; /* pointer to the transmit in data*/
   tx_out_ptr = tx_buf; /* pointer to the transmit out */	
   

/****************      ECEN 5803 add code as indicated   **********************/
    // uncomment this section after adding monitor code.  
   /* send a starting message to the terminal  */                    
   
   UART_direct_msg_put((const unsigned char *)"\r\nSystem Reset\r\nCode ver. ");
   UART_direct_msg_put((const unsigned char *) CODE_VERSION );
   UART_direct_msg_put((const unsigned char *)"\r\n");
   UART_direct_msg_put( (const unsigned char *)COPYRIGHT );
   UART_direct_msg_put((const unsigned char *)"\r\n");

   set_display_mode();         
                                
/****************      ECEN 5803 add code as indicated   **********************/
//                        Start ADC
    
 
    while(1) 
	{


    serial();            // Polls the serial port
    chk_UART_msg();     // checks for a serial port message received
    monitor();           // Sends serial port output messages depending
                         //  on commands received and display mode

		
/****************      ECEN 5803 add code as indicated   **********************/
//  readADC()

    if (adc_flag) {
        adc_flag = 0;


        uint16_t raw16 = adc_sample();          // from adc_data.c
        uint16_t raw12 = raw16 >> 4;            // keep top 12 bits
        float sample   = ((float)raw12 - 2048.0f) / 2048.0f;

        /* Feed one sample at 10 kHz to estimator */
        freq_hz = freq_estimate(sample);
    }



    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t adc_temp = HAL_ADC_GetValue(&hadc1);

    // Convert ADC to °C using STM32 internal-sensor formula
    float temp_c = (adc_temp * 3.3f / 4095.0f - 0.76f) / 0.0025f + 25.0f;

    // --- 4. Calculate flow in GPM ---
    float flow_gpm = flow_calculate(freq_hz, temp_c);
    flow_gpm_val = flow_gpm;

    // --- 5. PWM outputs ---
    //     Channel 2: proportional to flow (4–20 mA simulation)
    //     Channel 3: proportional to vortex frequency pulse output
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)(flow_gpm * 10));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)(freq_hz * 2));

    // --- 6. LCD Display ---
    #ifdef ENABLE_SPI
    set_cursor(0, 0);
    char line1[16];
    snprintf(line1, sizeof(line1), "F:%.1fHz", freq_hz);
    print_lcd(line1);

    set_cursor(0, 1);
    char line2[16];
    snprintf(line2, sizeof(line2), "Q:%.1fGPM", flow_gpm);
    print_lcd(line2);
    #endif

    // --- 7. UART terminal output ---
    printf("Freq:%d Hz, Flow:%d GPM, Temp:%d C\r\n",
           (int)freq_hz, (int)flow_gpm, (int)temp_c);


    // --- 8. LED blink rate proportional to vortex frequency ---
    if (freq_hz > 0)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Nucleo green LED
        HAL_Delay((uint32_t)(500.0f / (freq_hz / 500.0f + 1.0f))); // adaptive blink
    }




//   calculate ADC temperature
/*  Temperature (in �C) = {(VSENSE � V25) / Avg_Slope} + 25
Where:  � V25 = VSENSE value for 25� C
        � Avg_Slope = average slope of the temperature vs. VSENSE curve 
        (given in mV/�C or �V/�C)
Refer to the datasheet�s electrical characteristics section for the actual 
values of V25 and Avg_Slope.
*/


//  calculate flow()


//  4-20 output ()    // use PWM channel output proportional pulse rate to flow

//  Pulse output()   // use PWM channel output propotional pulse rate to frequency

//  LCD_Display()   // use the SPI port to send flow number

//  Blink the LED at a rate proportional to vortex frequency using timer0 code
//   to control the blinking rate

//  Write your code here for any additional tasks


      }
    count++;                  // counts the number of times through the loop				


		HAL_Delay(1000);
	
  
  
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  // Rank 1: PA1 (ADC_CHANNEL_1)
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  // Rank 2: internal temperature sensor
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;   // longer for temp sensor
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  // Rank 3: internal VREFINT
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_SS_GPIO_Port, SPI2_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_SS_Pin */
  GPIO_InitStruct.Pin = SPI2_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_SS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Configure PA1 (ADC1_IN1) as Analog, no pull */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
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
