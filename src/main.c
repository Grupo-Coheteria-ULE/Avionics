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
#include "config.h"
#include "osal.h"
#include "sd_spi.h"
#include "ff.h"
#include "imu.h"
#include "ms5837.h"
#include "csv_logger.h"
#include "altitude.h"
#include "apogee.h"
#ifdef CONFIG_CDC
#include "usbd_core.h"
#include "usbd_cdc.h"
extern USBD_DescriptorsTypeDef FS_Desc;
extern uint8_t CDC_Interface_Init(void);
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    SD_INIT_SPI_SLOW,
    SD_INIT_ATTACH,
    SD_INIT_CARD,
    SD_INIT_SPI_FAST,
    SD_INIT_DONE,
    SD_INIT_FAIL
} sd_init_step_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SD_INIT_TIMEOUT_MS          2000u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
osal_spi_t g_spi_sd;
sd_spi_t   g_sd;

static FIL    g_csv_file;
static FATFS  g_fatfs;
static char   g_csv_buf[CSV_LINE_MAX_LEN];

#ifdef CONFIG_CDC
USBD_HandleTypeDef hUsbDeviceFS;
#endif

static osal_i2c_t g_i2c_baro;
static ms5837_t   g_baro;

static altitude_ctx_t g_alt;
static apogee_ctx_t  g_apogee;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void sd_cs_select(void *user);
static void sd_cs_deselect(void *user);
static HAL_StatusTypeDef spi1_set_prescaler(uint32_t prescaler);
static sd_spi_status_t sd_init(void);
static void sdlog_setup(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void debug_msg(const char *msg)
{
#ifdef CONFIG_CDC
    CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
    HAL_Delay(50);
#else
    (void)msg;
#endif
}

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

static sd_spi_status_t sd_init(void)
{
    HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);
    HAL_Delay(20);

    osal_spi_init(&g_spi_sd,
                  &hspi1,
                  SD_INIT_TIMEOUT_MS,
                  sd_cs_select,
                  sd_cs_deselect,
                  NULL);

    /* SD must start at low speed */
    if (spi1_set_prescaler(SPI_BAUDRATEPRESCALER_256) != HAL_OK)
        return SD_SPI_ERR;

    sd_spi_attach(&g_sd, &g_spi_sd, SD_INIT_TIMEOUT_MS);

    sd_spi_status_t st = sd_spi_init(&g_sd);
    if (st != SD_SPI_OK)
        return st;

    /* Switch to high speed after init */
    if (spi1_set_prescaler(SPI_BAUDRATEPRESCALER_4) != HAL_OK)
        return SD_SPI_ERR;

    return SD_SPI_OK;
}

static void sdlog_setup(void)
{
    FRESULT fr;
    char filename[16];
    UINT bw;

    /* Mount filesystem */
    fr = f_mount(&g_fatfs, "", 1);
    if (fr != FR_OK)
        return;

    /* Find next available filename: datos_1.csv, datos_2.csv, ... */
    for (uint16_t n = 1; n <= 999; n++)
    {
        snprintf(filename, sizeof(filename), "datos_%u.csv", n);
        fr = f_open(&g_csv_file, filename, FA_READ);
        if (fr == FR_NO_FILE)
            break; /* File doesn't exist, use this name */
        f_close(&g_csv_file);
    }

    /* Open file for writing */
    fr = f_open(&g_csv_file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK)
        return;

    /* Write CSV header */
    const char *header = "t;ax;ay;az;wx;wy;wz;T;p\r\n";
    f_write(&g_csv_file, header, strlen(header), &bw);
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
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */

    /* --- USB CDC Device --- */
#ifdef CONFIG_CDC
    USBD_Init(&hUsbDeviceFS, &FS_Desc, 0);
    USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
    CDC_Interface_Init();
    USBD_Start(&hUsbDeviceFS);
    HAL_Delay(3000);
    debug_msg("=== AVI Flight Computer ===\r\n");
#endif

    /* --- SD Card --- */
    sd_spi_status_t sd_st = sd_init();

    /* --- IMU (LSM6DSO via SPI) --- */
    /* Full SPI peripheral reset via RCC to clear stuck BSY flag */
    HAL_SPI_DeInit(&hspi1);
    __HAL_RCC_SPI1_FORCE_RESET();
    HAL_Delay(1);
    __HAL_RCC_SPI1_RELEASE_RESET();
    HAL_Delay(1);
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    HAL_SPI_Init(&hspi1);
    HAL_Delay(10);

    int imu_st = imu_init();
    if (imu_st == 0)
        spi1_set_prescaler(SPI_BAUDRATEPRESCALER_4);

    /* --- IMU Calibration --- */
#ifdef CONFIG_IMU_CALIBRATE
    if (imu_st == 0)
    {
        imu_calibrate(1000);
    }
#endif

    /* --- Barometer (MS5837 via I2C) --- */
    osal_i2c_init(&g_i2c_baro, &hi2c1, 500);
    int baro_st = ms5837_init(&g_baro, &g_i2c_baro);

    /* --- Altitude and Apogee init --- */
    if (baro_st == 0) {
        ms5837_data_t baro;
        if (ms5837_read(&g_baro, &baro) == 0) {
            altitude_init(&g_alt, baro.pressure);
            apogee_init(&g_apogee);
        }
    }

    /* --- Debug status messages --- */
#ifdef CONFIG_CDC
    if (sd_st == SD_SPI_OK)
        debug_msg("[SD]   OK\r\n");
    else
        debug_msg("[SD]   FAIL\r\n");
    if (imu_st == 0)
        debug_msg("[IMU]  OK (LSM6DSO)\r\n");
    else
        debug_msg("[IMU]  FAIL\r\n");
    if (baro_st == 0)
        debug_msg("[BARO] OK (MS5837)\r\n");
    else
        debug_msg("[BARO] FAIL\r\n");
#endif

    /* --- Serial debug init --- */
    csv_debug_init();

    /* --- SD Logger (FatFs) --- */
    if (sd_st == SD_SPI_OK)
    {
        sdlog_setup();
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        csv_sample_t sample;
        memset(&sample, 0, sizeof(sample));

        sample.timestamp_ms = (uint32_t)((uint64_t)HAL_GetTick() * 1000); /* microseconds */

        /* Read IMU (LSM6DSO) — ax,ay,az in g, wx,wy,wz in dps, T in deg C */
        if (imu_st == 0)
        {
            imu_read(&sample.imu);
        }

        /* Read barometer (MS5837) — p in Pa, T in deg C */
        if (baro_st == 0)
        {
            ms5837_data_t baro;
            if (ms5837_read(&g_baro, &baro) == 0)
            {
                sample.temperature = baro.temperature;
                sample.pressure = baro.pressure;

                /* Update altitude and velocity */
                uint32_t now_ms = sample.timestamp_ms / 1000;
                int32_t vel = altitude_update(&g_alt, baro.pressure, now_ms);
                int32_t alt = altitude_get(&g_alt);

                /* Check apogee detection */
                if (apogee_update(&g_apogee, 20, alt, vel)) {
                    debug_msg("Paracaidas desplegado!\r\n");
                }
            }
        }

        /* Format CSV line: t;ax;ay;az;wx;wy;wz;T;p */
        int len = csv_format_line(&sample, g_csv_buf);

        /* Send over USB serial */
        csv_debug_print(&sample);

        /* Log to SD (FatFs) */
        if ((sd_st == SD_SPI_OK) && (len > 0))
        {
            UINT bw;
            f_write(&g_csv_file, g_csv_buf, (UINT)len, &bw);
            f_sync(&g_csv_file);
        }

        /* ~50 Hz data rate */
        HAL_Delay(20);
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

    /** Initializes the RCC Oscillators
    *  HSE 25 MHz -> PLL:
    *    PLLM = 25, PLLN = 384, PLLP = 4 -> VCO = 384 MHz, SYSCLK = 96 MHz
    *    PLLQ = 8 -> USB OTG = 48 MHz
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 384;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    *  SYSCLK = 96 MHz, AHB = 96 MHz, APB1 = 48 MHz, APB2 = 96 MHz
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
    huart2.Init.BaudRate = CONFIG_SERIAL_BAUDRATE;
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
