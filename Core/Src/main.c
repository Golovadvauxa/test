/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_ADDR (0x27 << 1)       // адрес дисплея, сдвинутый на 1 бит влево (HAL работает с I2C-адресами, сдвинутыми на 1 бит влево)

#define PIN_RS    (1 << 0)         // если на ножке 0, данные воспринимаются как команда, если 1 - как символы для вывода
#define PIN_EN    (1 << 2)         // бит, по изменению сост. которого считывается информация
#define BACKLIGHT (1 << 3)         // управление подсветкой

#define LCD_DELAY_MS 5             // пауза перед высвечиванием символа

#define PIN_OUTPUT 0
#define PIN_INPUT 1
#define PIN_SET 1
#define PIN_RESET 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
uint8_t mcp_read[8] = { 0x0041, 0x0043, 0x0045, 0x0047, 0x0049, 0x004B, 0x004D,
		0x004F };

uint8_t mcp_write[8] = { 0x0040, 0x0042, 0x0044, 0x0046, 0x0048, 0x004A, 0x004C,
		0x004E };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void I2C_send(uint8_t data, uint8_t flags) {
	HAL_StatusTypeDef res;
	// бесконечный цикл
	for (;;) {
		// проверяем, готово ли устройство по адресу lcd_addr для связи
		res = HAL_I2C_IsDeviceReady(&hi2c1, LCD_ADDR, 1, HAL_MAX_DELAY);
		// если да, то выходим из бесконечного цикла
		if (res == HAL_OK)
			break;
	}
	// операция �? с 1111 0000 приводит к обнулению бит с 0 по 3, остаются биты с 4 по 7
	uint8_t up = data & 0xF0;
	// то же самое, но data сдвигается на 4 бита влево
	uint8_t lo = (data << 4) & 0xF0;

	uint8_t data_arr[4];
	// 4-7 биты содержат информацию, биты 0-3 настраивают работу дисплея
	data_arr[0] = up | flags | BACKLIGHT | PIN_EN;
	// дублирование сигнала, на выводе Е в этот раз 0
	data_arr[1] = up | flags | BACKLIGHT;
	data_arr[2] = lo | flags | BACKLIGHT | PIN_EN;
	data_arr[3] = lo | flags | BACKLIGHT;

	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_arr, sizeof(data_arr),
	HAL_MAX_DELAY);
	HAL_Delay(LCD_DELAY_MS);
}

void LCD_SendString(char *str) {
	// *char по сути является строкой
	// пока строчка не закончится
	while (*str) {
		// передача первого символа строки
		I2C_send((uint8_t) (*str), 1);
		// сдвиг строки налево на 1 символ
		str++;
	}
}

static uint8_t I2C1_MCP23017_GET_REG(uint8_t reg_addr, uint8_t mcp_num) {
	uint8_t i2c_wbyte = reg_addr;
	uint8_t i2c1_rbyte;
	HAL_I2C_Master_Transmit(&hi2c1, mcp_write[mcp_num], &i2c_wbyte, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, mcp_read[mcp_num], &i2c1_rbyte, 1, 1000);
	return i2c1_rbyte;
}

static uint8_t I2C2_MCP23017_GET_REG(uint8_t reg_addr, uint8_t mcp_num) {
	uint8_t i2c_wbyte = reg_addr;
	uint8_t i2c2_rbyte;
	HAL_I2C_Master_Transmit(&hi2c2, mcp_write[mcp_num], &i2c_wbyte, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c2, mcp_read[mcp_num], &i2c2_rbyte, 1, 1000);
	return i2c2_rbyte;
}

static void I2C1_MCP23017_SET_REG(uint8_t reg_addr, uint8_t reg_val,
		uint8_t mcp_num) {
	uint8_t i2c_wbuf[2];
	i2c_wbuf[0] = reg_addr;
	i2c_wbuf[1] = reg_val;
	HAL_I2C_Master_Transmit(&hi2c1, mcp_write[mcp_num], i2c_wbuf, 2, 1000);
}
static void I2C2_MCP23017_SET_REG(uint8_t reg_addr, uint8_t reg_val,
		uint8_t mcp_num) {
	uint8_t i2c_wbuf[2];
	i2c_wbuf[0] = reg_addr;
	i2c_wbuf[1] = reg_val;
	HAL_I2C_Master_Transmit(&hi2c2, mcp_write[mcp_num], i2c_wbuf, 2, 1000);
}

static void I2C_MCP23017_SET_PIN_MODE(uint8_t pin_num, uint8_t pin_mode) { // pin_mode = 0 ... OUTPUT
// pin_mode = 1 ... INPUT
	uint8_t temp_pin_num = pin_num % 16;

	uint8_t bit_num = pin_num % 8;
	uint8_t bit_mask = 0x01 << bit_num;
	uint8_t reg_addr = 0;
	uint8_t reg_state;
	uint8_t reg_pin_pullup;
	if (temp_pin_num < 8){
		reg_addr = 0x00;
	reg_pin_pullup = 0x0C;
	}
	else{
		reg_addr = 0x01;
	reg_pin_pullup = 0x0D;
	}
	if (pin_num < 128)
		reg_state = I2C1_MCP23017_GET_REG(reg_addr, pin_num / 16);
	else
		reg_state = I2C2_MCP23017_GET_REG(reg_addr, (pin_num - 128) / 16);
	if (pin_mode == PIN_OUTPUT)
		reg_state = reg_state & (~bit_mask);

	else
		reg_state = reg_state | bit_mask;
	if (pin_num < 128)
		I2C1_MCP23017_SET_REG(reg_addr, reg_state, pin_num / 16);
	else
		I2C2_MCP23017_SET_REG(reg_addr, reg_state, (pin_num - 128) / 16);
}

static void I2C_MCP23017_SET_PIN_STATE(uint8_t pin_num, uint8_t pin_state) {
	uint8_t temp_pin_num = pin_num % 16;
	uint8_t bit_num = pin_num % 8;
	uint8_t bit_mask = 0x01 << bit_num;
	uint8_t reg_addr = 0;
	uint8_t reg_state;
	if (temp_pin_num < 8)
		reg_addr = 0x12;


	else
		reg_addr = 0x13;


	if (pin_num < 128)
		reg_state = I2C1_MCP23017_GET_REG(reg_addr, pin_num / 16);
	else
		reg_state = I2C2_MCP23017_GET_REG(reg_addr, (pin_num - 128) / 16);

	if (pin_state == PIN_SET)
		reg_state = reg_state | bit_mask;
	else
		reg_state = reg_state & (~bit_mask);

	if (pin_num < 128)
		I2C1_MCP23017_SET_REG(reg_addr, reg_state, pin_num / 16);
	else
		I2C2_MCP23017_SET_REG(reg_addr, reg_state, (pin_num - 128) / 16);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	/* USER CODE BEGIN 2 */
	I2C_send(0b00110000, 0);   // 8ми битный интерфейс
	I2C_send(0b00000010, 0);   // установка курсора в начале строки
	I2C_send(0b00001100, 0);   // нормальный режим работы, выкл курсор
	I2C_send(0b00000001, 0);   // очистка дисплея

	LCD_SendString("Hello");
	I2C_send(0b11000000, 0);   // перевод строки
	LCD_SendString("     WORLD!!!");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	I2C_MCP23017_SET_PIN_MODE(64, PIN_OUTPUT);
	I2C_MCP23017_SET_PIN_MODE(223, PIN_OUTPUT);

	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		I2C_MCP23017_SET_PIN_STATE(64, PIN_SET);
		I2C_MCP23017_SET_PIN_STATE(223, PIN_RESET);
		HAL_Delay(500);
		I2C_MCP23017_SET_PIN_STATE(64, PIN_RESET);
		I2C_MCP23017_SET_PIN_STATE(223, PIN_SET);
		HAL_Delay(500);
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_I2C2_Init(void) {

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
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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

