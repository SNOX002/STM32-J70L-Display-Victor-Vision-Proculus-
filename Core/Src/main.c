/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED(x) HAL_GPIO_WritePin(GPIOA, LED_Pin, x)
#define CS_I2C(x)       HAL_GPIO_WritePin(CS_I2C_GPIO_Port, CS_I2C_Pin, x) // Pino de teste
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
UART_HandleTypeDef *huart = &huart3;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t page = 1;
int16_t axis[3];
float accX, accY;

uint8_t Buf=0, Buffer[2];
uint8_t erro, i;

#include "Display_J70L.h"
#include <Perfilografo.h>

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DSP_MSG()
{
	if(*PtrByteCount == 0)
		return;

	HAL_Delay(5);	//1 caracter no BaudRate(115200) demora 50Us
	HAL_UART_DMAStop(&huart3);
	memcpy(DSP_Buffer, RXBuffer+3, *PtrByteCount);
	/* ******************************************
	 * Funções portada da antiga DSP_GetCMD()
	 */
	memset(RXBuffer, 0, 4);
	uint16_t VP = Get_VP(PtrDSP_Buffer);
	/******************************************/
//	DSP_GetCMD();
	perfilografo(VP);			
	/*Essa é a função personalizada que fiz para esse projeto em especifico. 
	*Ela Recebe como parâmetro o VP encontrado na Mensagem(MSG) do Display, toda a lógica funciona em cima disso,
	*os VPs são endereços de memória do Display e podem ser Botões, registradores ou slots de Strings
	*/
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
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
 HAL_UART_Receive_DMA(huart, RXBuffer, 60);		// Inicia o DMA e é fundamental para o funcionamento
 init_LIS();	//Inicia o acelerometro do meu projeto
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  accX = read_accelX();
//	  accY = read_accelY();
//	  Read_Data(axis);
	
	  DSP_MSG();		//Fica verificando se recebeu algo do display
	  /*
	  *Deixe sempre essa função executando a todo momento, pode ser dentro de uma interrupção
	  *Construi meu código com lógica de estado de máquina então deixei dentro do looping
	  * É possível utilizar o Callback de TX da Uart, se fizer desta forma me chama para coversarmos
	  * fiquei curioso para ver como seria o código com callback.
	  */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/**************************** Só coisas padroes daqui para baixo, tirando o exemplo de callback de Erro na UART que está comentado*/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart3)
//	{
//	}
//}
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
//    if (huart == &huart3) {
//        // Chamado quando ocorre um erro na comunicação UART
//        if (huart->ErrorCode & HAL_UART_ERROR_PE) {
//            // Erro de paridade
//        }
//        if (huart->ErrorCode & HAL_UART_ERROR_NE) {
//            // Erro de frame
//        	while(1);
//        }
//        if (huart->ErrorCode & HAL_UART_ERROR_FE) {
//            // Erro de frame
////        	while(1);
//        	erro++;
//        }
//        if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
//        	HAL_Delay(1000);
//
//        }
//        if (huart->ErrorCode & HAL_UART_ERROR_DMA) {
//            // Erro de DMA
//        }
//        // ... outros erros possíveis ...
//
//        // Você pode tomar medidas apropriadas para lidar com os erros aqui
//    }
//}
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
