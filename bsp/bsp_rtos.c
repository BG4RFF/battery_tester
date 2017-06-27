#include <stddef.h>
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "bsp_rtos.h"

SemaphoreHandle_t semaphore_spi1;
SemaphoreHandle_t semaphore_uart1;

void bsp_rtos_init(void) {

    semaphore_spi1 = xSemaphoreCreateBinary();
    xSemaphoreGive(semaphore_spi1);

    if(semaphore_spi1 == NULL) {
        //TODO signal
    }
    vQueueAddToRegistry(semaphore_spi1,"spi1");

    semaphore_uart1 = xSemaphoreCreateBinary();
    xSemaphoreGive(semaphore_uart1);

    if(semaphore_uart1 == NULL) {
        //TODO signal
    }
    vQueueAddToRegistry(semaphore_uart1,"uart1");
}

/* -------------------------------------------------------------------------- */

void bsp_spi1_lock(void) {
    xSemaphoreTake(semaphore_spi1, portMAX_DELAY);
}

/* -------------------------------------------------------------------------- */

void bsp_spi1_unlock(void) {
    xSemaphoreGive(semaphore_spi1);
}


/* -------------------------------------------------------------------------- */

void bsp_uart1_lock(void) {
    xSemaphoreTake(semaphore_uart1, portMAX_DELAY);
}

/* -------------------------------------------------------------------------- */

void bsp_uart1_unlock(void) {
    xSemaphoreGive(semaphore_uart1);
}

/* -------------------------------------------------------------------------- */
