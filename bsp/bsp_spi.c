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
static SPI_HandleTypeDef hspi1 = {0};

#define PIN_SET(PORT, PIN)      (PORT)->BSRR = (PIN)
#define PIN_RESET(PORT, PIN)    (PORT)->BRR = (PIN)

/* ----- Local functions ---------------------------------------------------- */


/* =====> Implementation ---------------------------------------------------- */

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

__weak void bsp_spi1_lock(void) {

}

/* -------------------------------------------------------------------------- */

__weak void bsp_spi1_unlock(void) {

}

/* -------------------------------------------------------------------------- */


void spi_init(void) {

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        //    Error_Handler();
    }
}

/* -------------------------------------------------------------------------- */


void bsp_cs_touch_set_low(void) {
    bsp_spi1_lock();
    PIN_RESET(CS_TS_PORT, CS_TS_PIN);
}

/* -------------------------------------------------------------------------- */

void bsp_cs_touch_set_high(void) {

    PIN_SET(CS_TS_PORT, CS_TS_PIN);
    bsp_spi1_unlock();
}

/* -------------------------------------------------------------------------- */

bool bsp_is_touch_irq_active(void) {

    if(INT_TS_PORT->IDR & INT_TS_PIN) {
        return false;
    }

    return true;
}

/* -------------------------------------------------------------------------- */
void bsp_spi1_wr_rd(uint8_t *wr_data, uint8_t *rd_data, uint32_t size) {
    HAL_SPI_TransmitReceive(&hspi1, wr_data, rd_data, size, 1000);
}

/* -------------------------------------------------------------------------- */

void bsp_cs_sd_set_low(void) {

    if(CS_SD_PORT->IDR & CS_SD_PIN) {

        bsp_spi1_lock();
        PIN_RESET(CS_SD_PORT, CS_SD_PIN);
    }
}

/* -------------------------------------------------------------------------- */

void bsp_cs_sd_set_high(void) {

    PIN_SET(CS_SD_PORT, CS_SD_PIN);
    bsp_spi1_unlock();
}

/* -------------------------------------------------------------------------- */


/* end: bsp.c ----- */