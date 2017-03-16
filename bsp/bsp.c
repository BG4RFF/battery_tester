/* begin: bsp.c ----- */
/* created 3/16/2017 by Denis Krasutski ----------------------------------- */

/* ----- Includes ----------------------------------------------------------- */
#include "bsp.h"
#include "pins.h"
#include "stm32f1xx_hal.h"

/* ----- Types -------------------------------------------------------------- */
/* ----- Settings ----------------------------------------------------------- */
/* ----- Global variables --------------------------------------------------- */
/* ----- Local variables ---------------------------------------------------- */

static ADC_HandleTypeDef hadc1 = {0};
static SPI_HandleTypeDef hspi1 = {0};
static UART_HandleTypeDef huart1 = {0};

void (*bsp_charger_flag_cb)(void) = NULL;

/* ----- Local functions ---------------------------------------------------- */


/* =====> Implementation ---------------------------------------------------- */

void HAL_MspInit(void) {

    __HAL_RCC_AFIO_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    /* BusFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    /* UsageFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    /* SVCall_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    /* DebugMonitor_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    /**NOJTAG: JTAG-DP Disabled and SW-DP Enabled
    */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

}

/* -------------------------------------------------------------------------- */

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {

    GPIO_InitTypeDef GPIO_InitStruct;
    if(hadc->Instance==ADC1) {
        /* Peripheral clock enable */
        __HAL_RCC_ADC1_CLK_ENABLE();

        GPIO_InitStruct.Pin = ADC_VBAT_PIN | ADC_VIN_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/* -------------------------------------------------------------------------- */

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {

    if(hadc->Instance==ADC1) {
        /* Peripheral clock disable */
        __HAL_RCC_ADC1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOB, ADC_VBAT_PIN | ADC_VIN_PIN);
    }
}

/* -------------------------------------------------------------------------- */

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {

    GPIO_InitTypeDef GPIO_InitStruct;
    if(hspi->Instance==SPI1) {

        /* Peripheral clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();

        /**SPI1 GPIO Configuration
        PA5     ------> SPI1_SCK
        PA6     ------> SPI1_MISO
        PA7     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/* -------------------------------------------------------------------------- */

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {

    if(hspi->Instance==SPI1) {
        /* Peripheral clock disable */
        __HAL_RCC_SPI1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    }
}

/* -------------------------------------------------------------------------- */

void HAL_UART_MspInit(UART_HandleTypeDef* huart) {

    GPIO_InitTypeDef GPIO_InitStruct;
    if(huart->Instance==USART1) {

        /* Peripheral clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitStruct.Pin = UART1_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(UART1_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = UART1_RX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(UART1_RX_PORT, &GPIO_InitStruct);
    }

}

/* -------------------------------------------------------------------------- */

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {

    if(huart->Instance==USART1) {
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();

        HAL_GPIO_DeInit(UART1_TX_PORT, UART1_TX_PIN);
        HAL_GPIO_DeInit(UART1_RX_PORT, UART1_RX_PIN);
    }
}

/* -------------------------------------------------------------------------- */

static void _gpio_init(void) {

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, DB00_PIN|DB01_PIN|DB02_PIN|DB03_PIN
                      |DB04_PIN|DB05_PIN|DB06_PIN|DB07_PIN
                          |LCD_RS_PIN|CS_LCD_PIN|LCD_WR_PIN|LCD_RD_PIN
                              |LCD_BL_EN_PIN, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LED1_PIN|LED2_PIN|CS_TS_PIN|CHARGER_SW_SEL_PIN, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LED3_PIN|DB10_PIN|DB11_PIN|DB12_PIN
                      |DB13_PIN|DB14_PIN|DB15_PIN|RELAY_CONTROL_PIN
                          |CS_F_PIN|CS_SD_PIN|DB08_PIN|DB09_PIN, GPIO_PIN_RESET);

    /*Configure GPIO pin : INT_TS_PIN */
    GPIO_InitStruct.Pin = INT_TS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(INT_TS_GPIO_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : DB00_PIN DB01_PIN DB02_PIN DB03_PIN
    DB04_PIN DB05_PIN DB06_PIN DB07_PIN
    LCD_RS_PIN CS_LCD_PIN LCD_WR_PIN LCD_RD_PIN
    LCD_BL_EN_PIN */
    GPIO_InitStruct.Pin = DB00_PIN|DB01_PIN|DB02_PIN|DB03_PIN
        |DB04_PIN|DB05_PIN|DB06_PIN|DB07_PIN
            |LCD_RS_PIN|CS_LCD_PIN|LCD_WR_PIN|LCD_RD_PIN
                |LCD_BL_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : BUTTON_1_PIN BUTTON_2_PIN */
    GPIO_InitStruct.Pin = BUTTON_1_PIN|BUTTON_2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LED1_PIN LED2_PIN CS_TS_PIN CHARGER_SW_SEL_PIN */
    GPIO_InitStruct.Pin = LED1_PIN|LED2_PIN|CS_TS_PIN|CHARGER_SW_SEL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LED3_PIN DB10_PIN DB11_PIN DB12_PIN
    DB13_PIN DB14_PIN DB15_PIN
    CS_F_PIN CS_SD_PIN DB08_PIN DB09_PIN */
    GPIO_InitStruct.Pin = LED3_PIN|DB10_PIN|DB11_PIN|DB12_PIN
        |DB13_PIN|DB14_PIN|DB15_PIN
            |CS_F_PIN|CS_SD_PIN|DB08_PIN|DB09_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_PORT, RELAY_CONTROL_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = RELAY_CONTROL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RELAY_CONTROL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CHARGER_STATUS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CHARGER_STATUS_PORT, &GPIO_InitStruct);

    __HAL_GPIO_EXTI_CLEAR_IT(CHARGER_STATUS_PIN);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* -------------------------------------------------------------------------- */

static void _clock_init(void) {

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        //Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
        |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        //    Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        //    Error_Handler();
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* -------------------------------------------------------------------------- */

static void _spi_init(void) {

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
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        //    Error_Handler();
    }
}

/* -------------------------------------------------------------------------- */

static void _uart_init(void) {

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 921600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        //    Error_Handler();
    }

}

/* -------------------------------------------------------------------------- */

static void _adc_init(void) {

    ADC_ChannelConfTypeDef sConfig;

    /**Common config
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        //    Error_Handler();
    }

    /**Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        //    Error_Handler();
    }

}

/* -------------------------------------------------------------------------- */

void bsp_init(void) {

    HAL_Init();

    _clock_init();

    _gpio_init();

    _uart_init();
    _spi_init();
    _adc_init();
}

/* -------------------------------------------------------------------------- */

void bsp_switch_relay(const bsp_relay_swith_t select) {

    if(select == RELAY_CHARGE) {
        HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_PORT, RELAY_CONTROL_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_PORT, RELAY_CONTROL_PIN, GPIO_PIN_RESET);
    }
}

/* -------------------------------------------------------------------------- */

void bsp_debug_write(const uint8_t *data, uint32_t size) {

    HAL_UART_Transmit(&huart1, (uint8_t*)data, size, 1000);
}

/* -------------------------------------------------------------------------- */


void bsp_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

/* -------------------------------------------------------------------------- */


uint32_t bsp_get_tick_ms(void) {
    return HAL_GetTick();
}

/* -------------------------------------------------------------------------- */


void bsp_disable_irq(void) {
    __disable_irq();
}

/* -------------------------------------------------------------------------- */


void bsp_enable_irq(void) {
    __enable_irq();
}

/* -------------------------------------------------------------------------- */

bool bsp_is_charger_flag_high(void) {

    if(HAL_GPIO_ReadPin(CHARGER_STATUS_PORT, CHARGER_STATUS_PIN) == GPIO_PIN_SET) {

        return true;
    }

    return false;
}

/* -------------------------------------------------------------------------- */

bool bsp_is_charger_swire_pin_high(void) {

    if(HAL_GPIO_ReadPin(CHARGER_SW_SEL_PORT, CHARGER_SW_SEL_PIN) == GPIO_PIN_SET) {
        return true;
    }
    return false;
}

/* -------------------------------------------------------------------------- */

void bsp_togle_charger_swire_pin(void) {

  HAL_GPIO_TogglePin(CHARGER_SW_SEL_PORT, CHARGER_SW_SEL_PIN);
}

/* -------------------------------------------------------------------------- */

void bsp_register_charger_status_cb(void (*cb)(void)) {
    if(cb == NULL) {
        bsp_charger_flag_cb = NULL;

    } else {
        bsp_charger_flag_cb = cb;
    }
}

/* end: bsp.c ----- */

