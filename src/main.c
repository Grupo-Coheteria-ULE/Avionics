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
#include <string.h>
#include "osal.h"
#include "sd_spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  DIAG_SD_IDLE = 0,
  DIAG_SD_SPI_SLOW,
  DIAG_SD_ATTACH,
  DIAG_SD_INIT,
  DIAG_SD_SPI_FAST,
  DIAG_SD_BACKUP_READ,
  DIAG_SD_TEST_WRITE,
  DIAG_SD_TEST_READ,
  DIAG_SD_COMPARE,
  DIAG_SD_RESTORE,
  DIAG_SD_DONE,
  DIAG_SD_FAIL
} diag_sd_step_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SD_DIAG_TIMEOUT_MS          2000u
#define SD_DIAG_TEST_LBA_DEFAULT    2048u
#define SD_DIAG_PATTERN_SEED        0x5Au
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static osal_spi_t g_spi_sd;
static sd_spi_t   g_sd;

static uint8_t g_sd_backup[SD_SPI_SECTOR_SIZE];
static uint8_t g_sd_tx[SD_SPI_SECTOR_SIZE];
static uint8_t g_sd_rx[SD_SPI_SECTOR_SIZE];

volatile diag_sd_step_t g_diag_step = DIAG_SD_IDLE;
volatile uint8_t g_diag_done = 0u;
volatile uint8_t g_diag_ok = 0u;

volatile sd_spi_status_t g_sd_init_status = SD_SPI_ERR;
volatile sd_spi_status_t g_sd_backup_status = SD_SPI_ERR;
volatile sd_spi_status_t g_sd_write_status = SD_SPI_ERR;
volatile sd_spi_status_t g_sd_read_status = SD_SPI_ERR;
volatile sd_spi_status_t g_sd_restore_status = SD_SPI_ERR;

volatile uint32_t g_sd_sector_count = 0u;
volatile sd_spi_card_t g_sd_card_type = SD_SPI_CARD_UNKNOWN;
volatile uint32_t g_sd_test_lba = 0u;
volatile uint32_t g_sd_mismatch_index = 0xFFFFFFFFu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void sd_cs_select(void *user);
static void sd_cs_deselect(void *user);
static HAL_StatusTypeDef spi1_set_prescaler(uint32_t prescaler);
static void sd_diag_fill_pattern(uint8_t *buf, uint32_t len, uint8_t seed);
static uint32_t sd_diag_find_mismatch(const uint8_t *a, const uint8_t *b, uint32_t len);
static void sd_diag_run(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void sd_cs_select(void *user)
{
  (void)user;
  HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_RESET);
}

static void sd_cs_deselect(void *user)
{
  (void)user;
  HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef spi1_set_prescaler(uint32_t prescaler)
{
  (void)HAL_SPI_DeInit(&hspi1);
  hspi1.Init.BaudRatePrescaler = prescaler;
  return HAL_SPI_Init(&hspi1);
}

static void sd_diag_fill_pattern(uint8_t *buf, uint32_t len, uint8_t seed)
{
  uint32_t i;

  if (buf == NULL)
  {
    return;
  }

  for (i = 0u; i < len; i++)
  {
    buf[i] = (uint8_t)(seed + (uint8_t)i + (uint8_t)(i >> 3));
  }
}

static uint32_t sd_diag_find_mismatch(const uint8_t *a, const uint8_t *b, uint32_t len)
{
  uint32_t i;

  if ((a == NULL) || (b == NULL))
  {
    return 0u;
  }

  for (i = 0u; i < len; i++)
  {
    if (a[i] != b[i])
    {
      return i;
    }
  }

  return 0xFFFFFFFFu;
}

static void sd_diag_run(void)
{
  g_diag_done = 0u;
  g_diag_ok = 0u;
  g_diag_step = DIAG_SD_IDLE;

  g_sd_init_status = SD_SPI_ERR;
  g_sd_backup_status = SD_SPI_ERR;
  g_sd_write_status = SD_SPI_ERR;
  g_sd_read_status = SD_SPI_ERR;
  g_sd_restore_status = SD_SPI_ERR;

  g_sd_sector_count = 0u;
  g_sd_card_type = SD_SPI_CARD_UNKNOWN;
  g_sd_test_lba = 0u;
  g_sd_mismatch_index = 0xFFFFFFFFu;

  memset(g_sd_backup, 0, sizeof(g_sd_backup));
  memset(g_sd_tx, 0, sizeof(g_sd_tx));
  memset(g_sd_rx, 0, sizeof(g_sd_rx));

  /* Dejar ambos dispositivos deseleccionados */
  HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);
  HAL_Delay(20);

  osal_spi_init(&g_spi_sd,
                &hspi1,
                SD_DIAG_TIMEOUT_MS,
                sd_cs_select,
                sd_cs_deselect,
                NULL);

  /* La SD debe arrancar a baja velocidad */
  g_diag_step = DIAG_SD_SPI_SLOW;
  if (spi1_set_prescaler(SPI_BAUDRATEPRESCALER_256) != HAL_OK)
  {
    g_diag_step = DIAG_SD_FAIL;
    g_diag_done = 1u;
    return;
  }

  g_diag_step = DIAG_SD_ATTACH;
  sd_spi_attach(&g_sd, &g_spi_sd, SD_DIAG_TIMEOUT_MS);

  g_diag_step = DIAG_SD_INIT;
  g_sd_init_status = sd_spi_init(&g_sd);
  if (g_sd_init_status != SD_SPI_OK)
  {
    g_diag_step = DIAG_SD_FAIL;
    g_diag_done = 1u;
    return;
  }

  g_sd_sector_count = sd_spi_get_sector_count(&g_sd);
  g_sd_card_type = sd_spi_get_card_type(&g_sd);

  if (g_sd_sector_count < 2u)
  {
    g_diag_step = DIAG_SD_FAIL;
    g_diag_done = 1u;
    return;
  }

  /* Subimos velocidad tras inicializar correctamente */
  g_diag_step = DIAG_SD_SPI_FAST;
  if (spi1_set_prescaler(SPI_BAUDRATEPRESCALER_4) != HAL_OK)
  {
    g_diag_step = DIAG_SD_FAIL;
    g_diag_done = 1u;
    return;
  }

  g_sd_test_lba = (g_sd_sector_count > SD_DIAG_TEST_LBA_DEFAULT)
                ? SD_DIAG_TEST_LBA_DEFAULT
                : (g_sd_sector_count - 1u);

  /* Leer sector original */
  g_diag_step = DIAG_SD_BACKUP_READ;
  g_sd_backup_status = sd_spi_read(&g_sd, g_sd_test_lba, g_sd_backup, 1u);
  if (g_sd_backup_status != SD_SPI_OK)
  {
    g_diag_step = DIAG_SD_FAIL;
    g_diag_done = 1u;
    return;
  }

  /* Escribir patrón */
  sd_diag_fill_pattern(g_sd_tx, sizeof(g_sd_tx), SD_DIAG_PATTERN_SEED);

  g_diag_step = DIAG_SD_TEST_WRITE;
  g_sd_write_status = sd_spi_write(&g_sd, g_sd_test_lba, g_sd_tx, 1u);
  if (g_sd_write_status != SD_SPI_OK)
  {
    g_diag_step = DIAG_SD_FAIL;
    g_diag_done = 1u;
    return;
  }

  /* Leer de vuelta */
  g_diag_step = DIAG_SD_TEST_READ;
  g_sd_read_status = sd_spi_read(&g_sd, g_sd_test_lba, g_sd_rx, 1u);
  if (g_sd_read_status != SD_SPI_OK)
  {
    g_diag_step = DIAG_SD_FAIL;
    g_diag_done = 1u;
    return;
  }

  /* Comparar */
  g_diag_step = DIAG_SD_COMPARE;
  g_sd_mismatch_index = sd_diag_find_mismatch(g_sd_tx, g_sd_rx, SD_SPI_SECTOR_SIZE);

  /* Restaurar sector original */
  g_diag_step = DIAG_SD_RESTORE;
  g_sd_restore_status = sd_spi_write(&g_sd, g_sd_test_lba, g_sd_backup, 1u);

  if ((g_sd_mismatch_index == 0xFFFFFFFFu) && (g_sd_restore_status == SD_SPI_OK))
  {
    g_diag_ok = 1u;
    g_diag_step = DIAG_SD_DONE;
  }
  else
  {
    g_diag_step = DIAG_SD_FAIL;
  }

  g_diag_done = 1u;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  sd_diag_run();
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CS_SD_Pin */
  GPIO_InitStruct.Pin = CS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_SD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_IMU_Pin */
  GPIO_InitStruct.Pin = CS_IMU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_IMU_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
