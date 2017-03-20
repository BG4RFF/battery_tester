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
static TIM_HandleTypeDef htim2 = {0};
DMA_HandleTypeDef hdma_adc1;

void (*bsp_charger_flag_cb)(void) = NULL;
void (*bsp_timer1_cb)(void) = NULL;
static bool _adc_complete = false;

#define ADC_CHANNEL_COUNT       (2U)

const uint32_t ADC_TIMEOUT_MS = 100U;
uint16_t volts[ADC_CHANNEL_COUNT];

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
        __HAL_RCC_DMA1_CLK_ENABLE();

        GPIO_InitStruct.Pin = ADC_VBAT_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(ADC_VBAT_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = ADC_VIN_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(ADC_VIN_PORT, &GPIO_InitStruct);


        RCC_PeriphCLKInitTypeDef  PeriphClkInit;

        /* Configure ADCx clock prescaler */
        /* Caution: On STM32F1, ADC clock frequency max is 14MHz (refer to device   */
        /*          datasheet).                                                     */
        /*          Therefore, ADC clock prescaler must be configured in function   */
        /*          of ADC clock source frequency to remain below this maximum      */
        /*          frequency.                                                      */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
        PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);



        /* Peripheral DMA init*/

        hdma_adc1.Instance = DMA1_Channel1;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;

		HAL_DMA_DeInit(&hdma_adc1);
        HAL_DMA_Init(&hdma_adc1);

        __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);

        /* System interrupt init*/
        HAL_NVIC_SetPriority(ADC1_2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

        /* DMA interrupt init */
        HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    }
}

/* -------------------------------------------------------------------------- */

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {

    if(hadc->Instance==ADC1) {
        /* Peripheral clock disable */
        __HAL_RCC_ADC1_CLK_DISABLE();

        HAL_GPIO_DeInit(ADC_VBAT_PORT, ADC_VBAT_PIN);
        HAL_GPIO_DeInit(ADC_VIN_PORT, ADC_VIN_PIN);
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

/*----------------------------------------------------------------------------*/

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {

    if(htim_base->Instance==TIM2) {
        /* Peripheral clock enable */
        __TIM2_CLK_ENABLE();
    }
}

/*----------------------------------------------------------------------------*/

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {

    if(htim_base->Instance==TIM2) {
        /* Peripheral clock disable */
        __TIM2_CLK_DISABLE();
    }
}
/* -------------------------------------------------------------------------- */

static void _timer_init(void) {

    htim2.Instance = TIM2;
    htim2.Init.Period            = 100;
    htim2.Init.Prescaler         = 72;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.RepetitionCounter = 0;

    HAL_TIM_Base_Init(&htim2);

    NVIC_SetPriority(TIM2_IRQn, 6);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/* -------------------------------------------------------------------------- */
static void _lcd_bus_init(bool is_output) {

    GPIO_InitTypeDef GPIO_InitStruct;

    if(is_output) {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    } else {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    }

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = DB08_PIN|DB09_PIN|DB10_PIN|DB11_PIN|DB12_PIN|DB13_PIN|DB14_PIN|DB15_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DB00_PIN|DB01_PIN|DB02_PIN|DB03_PIN|DB04_PIN|DB05_PIN|DB06_PIN|DB07_PIN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

static void _gpio_init(void) {

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIOA->BRR = 0xFFFF;
    GPIOB->BRR = 0xFFFF;
    GPIOC->BRR = 0xFFFF;
    GPIOD->BRR = 0xFFFF;


    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    /* conf OD pins */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;

    HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = RELAY_CONTROL_PIN;
    HAL_GPIO_Init(RELAY_CONTROL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED1_PIN;
    HAL_GPIO_Init(LED1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED2_PIN;
    HAL_GPIO_Init(LED2_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED3_PIN;
    HAL_GPIO_Init(LED3_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CHARGER_EN_PIN;
    HAL_GPIO_Init(CHARGER_EN_PORT, &GPIO_InitStruct);

    /* conf PP pins */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pin = CHARGER_SW_SEL_PIN;
    HAL_GPIO_Init(CHARGER_SW_SEL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_BL_EN_PIN;
    HAL_GPIO_Init(LCD_BL_EN_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_RS_PIN;
    HAL_GPIO_Init(LCD_RS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_RD_PIN;
    HAL_GPIO_Init(LCD_RD_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_WR_PIN;
    HAL_GPIO_Init(LCD_WR_PORT, &GPIO_InitStruct);


    HAL_GPIO_WritePin(CS_LCD_PORT, CS_LCD_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = CS_LCD_PIN;
    HAL_GPIO_Init(CS_LCD_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(CS_TS_PORT, CS_TS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = CS_TS_PIN;
    HAL_GPIO_Init(CS_TS_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(CS_F_PORT, CS_F_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = CS_F_PIN;
    HAL_GPIO_Init(CS_F_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(CS_SD_PORT, CS_SD_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = CS_SD_PIN;
    HAL_GPIO_Init(CS_SD_PORT, &GPIO_InitStruct);


    _lcd_bus_init(true);


    /* conf INPUT pins */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = BUTTON1_PIN;
    HAL_GPIO_Init(BUTTON1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON2_PIN;
    HAL_GPIO_Init(BUTTON2_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = INT_TS_PIN;
    HAL_GPIO_Init(INT_TS_PORT, &GPIO_InitStruct);

    /* conf INPUT IT pins */
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;

    GPIO_InitStruct.Pin = CHARGER_STATUS_PIN;
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
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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

    /**Common config */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = ENABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = ADC_CHANNEL_COUNT;
    hadc1.Init.NbrOfDiscConversion = 1;

    HAL_ADC_Init(&hadc1);

    /**Configure Regular Channel */

    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);


	HAL_ADCEx_Calibration_Start(&hadc1);


    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)volts, ADC_CHANNEL_COUNT);
}

/* -------------------------------------------------------------------------- */

void bsp_init(void) {

    HAL_Init();

    _clock_init();

    _gpio_init();
    _uart_init();
    _spi_init();
    _adc_init();
    _timer_init();
}

/* -------------------------------------------------------------------------- */

void bsp_switch_relay(const bsp_relay_swith_t select) {

    if(select == RELAY_DISCHARGE_OFF) {
        HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(RELAY_CONTROL_PORT, RELAY_CONTROL_PIN, GPIO_PIN_RESET);
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

void bsp_charger_en_set_high(void) {

    HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */

void bsp_charger_en_set_low(void) {

    HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_RESET);
}

/* -------------------------------------------------------------------------- */

void bsp_toggle_charger_swire_pin(void) {

    HAL_GPIO_TogglePin(CHARGER_SW_SEL_PORT, CHARGER_SW_SEL_PIN);
}

/* -------------------------------------------------------------------------- */

void bsp_register_charger_status_cb(void (*cb)(void)) {

    bsp_charger_flag_cb = cb;
}

/* -------------------------------------------------------------------------- */

void bsp_charger_timer_set_period(uint32_t us, void (*cb)(void)) {

    bsp_timer1_cb = cb;

    __HAL_TIM_SET_AUTORELOAD(&htim2, us);
    HAL_TIM_Base_Start_IT(&htim2);

}

/* -------------------------------------------------------------------------- */

void bsp_charger_timer_disable(void) {

    HAL_TIM_Base_Stop_IT(&htim2);

    bsp_timer1_cb = NULL;
}

/* -------------------------------------------------------------------------- */

void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef * hadc) {

	_adc_complete = true;
}

/* -------------------------------------------------------------------------- */

uint32_t bsp_get_voltage(bsp_voltage_source_t source) {


    _adc_complete = false;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS);

    uint32_t time = bsp_get_tick_ms();
    while(_adc_complete == false) {
        if((bsp_get_tick_ms() - time) > ADC_TIMEOUT_MS) {
            break;
        }
    }

    float k = 3300.f / 4096.f;

    return (uint32_t)((float)volts[source] * k * 2.f);
}

/* -------------------------------------------------------------------------- */

void bsp_lcd_write_reg(uint32_t reg, uint16_t data) {

    CS_LCD_PORT->BRR = CS_LCD_PIN;
    LCD_RS_PORT->BRR = LCD_RS_PIN;
	LCD_WR_PORT->BRR = LCD_WR_PIN;

    GPIOC->ODR = ((GPIOC->ODR&0xFF00)|(reg&0x00FF));
    GPIOB->ODR = ((GPIOB->ODR&0x00FF)|(reg&0xFF00));

	LCD_WR_PORT->BSRR = LCD_WR_PIN;

    /* Write 16-bit Reg */
	LCD_RS_PORT->BSRR = LCD_RS_PIN;
	LCD_WR_PORT->BRR = LCD_WR_PIN;

    GPIOC->ODR = ((GPIOC->ODR&0xFF00)|(data&0x00FF));
    GPIOB->ODR = ((GPIOB->ODR&0x00FF)|(data&0xFF00));

	LCD_WR_PORT->BSRR = LCD_WR_PIN;
	CS_LCD_PORT->BSRR = CS_LCD_PIN;
}

/* -------------------------------------------------------------------------- */

uint16_t bsp_lcd_read_reg(uint32_t reg) {

    uint16_t data = 0;

	/* Write 16-bit Index (then Read Reg) */
	CS_LCD_PORT->BRR = CS_LCD_PIN;
	LCD_RS_PORT->BRR = LCD_RS_PIN;
	LCD_WR_PORT->BRR = LCD_WR_PIN;

    GPIOC->ODR = ((GPIOC->ODR&0xFF00)|(reg&0x00FF));
    GPIOB->ODR = ((GPIOB->ODR&0x00FF)|(reg&0xFF00));

	LCD_WR_PORT->BSRR = LCD_WR_PIN;

    _lcd_bus_init(false);

	/* Read 16-bit Reg */
	LCD_RS_PORT->BSRR = LCD_RS_PIN;
	LCD_RD_PORT->BRR = LCD_RD_PIN;
	LCD_RD_PORT->BSRR = LCD_RD_PIN;


	data = GPIOB->IDR & 0xFF00;
    data |= GPIOC->IDR & 0x00FF;

    _lcd_bus_init(true);

	CS_LCD_PORT->BSRR = CS_LCD_PIN;

	return data;
}

/* -------------------------------------------------------------------------- */

uint16_t LCD_ReadSta(void) {

    uint16_t data;

    _lcd_bus_init(false);
	/* Write 16-bit Index, then Write Reg */
	LCD_RS_PORT->BSRR = LCD_RS_PIN;
    LCD_RD_PORT->BRR = LCD_RD_PIN;
	LCD_RD_PORT->BSRR = LCD_RD_PIN;
	data = (GPIOB->IDR&0xFF00);
    data |= (GPIOC->IDR&0x00FF);

    CS_LCD_PORT->BSRR = CS_LCD_PIN;

    _lcd_bus_init(true);

	return data;
}

/* -------------------------------------------------------------------------- */

void bsp_lcd_write_command(uint16_t data) {

    CS_LCD_PORT->BRR = CS_LCD_PIN;
	LCD_RS_PORT->BRR = LCD_RS_PIN;
	LCD_WR_PORT->BRR = LCD_WR_PIN;

    GPIOC->ODR = (GPIOC->ODR&0xFF00)|(data&0x00FF);
    GPIOB->ODR = (GPIOB->ODR&0x00FF)|(data&0xFF00);

	LCD_WR_PORT->BSRR = LCD_WR_PIN;
	CS_LCD_PORT->BSRR = CS_LCD_PIN;
}

/* -------------------------------------------------------------------------- */

void bsp_lcd_write_prepare(void){

    /* Write 16-bit Index, then Write Reg */
	CS_LCD_PORT->BRR = CS_LCD_PIN;
	LCD_RS_PORT->BRR = LCD_RS_PIN;
	LCD_WR_PORT->BRR = LCD_WR_PIN;

    const uint32_t reg = 34;
    GPIOC->ODR = (GPIOC->ODR&0xFF00)|(reg&0x00FF);
    GPIOB->ODR = (GPIOB->ODR&0x00FF)|(reg&0xFF00);

	LCD_WR_PORT->BSRR = LCD_WR_PIN;
	CS_LCD_PORT->BSRR = CS_LCD_PIN;

}

/* -------------------------------------------------------------------------- */

void bsp_lcd_write_ram(uint16_t data) {

	CS_LCD_PORT->BRR = CS_LCD_PIN;
	LCD_RS_PORT->BSRR = LCD_RS_PIN;
	LCD_WR_PORT->BRR = LCD_WR_PIN;

    GPIOC->ODR = (GPIOC->ODR&0xFF00)|(data&0x00FF);
    GPIOB->ODR = (GPIOB->ODR&0x00FF)|(data&0xFF00);

	LCD_WR_PORT->BSRR = LCD_WR_PIN;
	CS_LCD_PORT->BSRR = CS_LCD_PIN;
}

/* -------------------------------------------------------------------------- */

void bsp_lcd_backlight_enable(bsp_backlight_control_t control) {

    if(control == LCD_BACKLIGHT_DISABLE) {
        LCD_BL_EN_PORT->BRR = LCD_BL_EN_PIN;
    } else {
        LCD_BL_EN_PORT->BSRR = LCD_BL_EN_PIN;
    }
}

/* -------------------------------------------------------------------------- */

void bsp_cs_touch_set_low(void) {

    CS_TS_PORT->BRR = CS_TS_PIN;
}

/* -------------------------------------------------------------------------- */

void bsp_cs_touch_set_high(void) {

    CS_TS_PORT->BSRR = CS_TS_PIN;
}

/* -------------------------------------------------------------------------- */

void bsp_touch_wr_rd(uint8_t *wr_data, uint8_t *rd_data, uint32_t size) {
    HAL_SPI_TransmitReceive(&hspi1, wr_data, rd_data, size, 1000);
}

/* -------------------------------------------------------------------------- */

bool bsp_is_button1_pressed(void) {

    if(BUTTON1_PORT->IDR & BUTTON1_PIN) {
        return false;
    }

    return true;
}

/* -------------------------------------------------------------------------- */

bool bsp_is_button2_pressed(void) {

    if(BUTTON2_PORT->IDR & BUTTON2_PIN) {
        return false;
    }

    return true;
}

/* -------------------------------------------------------------------------- */

void bsp_led1_enable(bsp_led_control_t control) {

    if(control == LED_ENABLE) {
        LED1_PORT->BRR = LED1_PIN;
    } else {
        LED1_PORT->BSRR = LED1_PIN;
    }
}

/* -------------------------------------------------------------------------- */

void bsp_led2_enable(bsp_led_control_t control) {

    if(control == LED_ENABLE) {
        LED2_PORT->BRR = LED2_PIN;
    } else {
        LED2_PORT->BSRR = LED2_PIN;
    }
}

/* -------------------------------------------------------------------------- */

void bsp_led3_enable(bsp_led_control_t control) {

    if(control == LED_ENABLE) {
        LED3_PORT->BRR = LED3_PIN;
    } else {
        LED3_PORT->BSRR = LED3_PIN;
    }
}

/* end: bsp.c ----- */