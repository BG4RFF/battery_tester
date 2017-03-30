#include "logger.h"

#include "bsp.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

/* ===== SETTINGS =========================================================== */
#define _LOGGER_LOCK()
#define _LOGGER_UNLOCK()

/* ===== TYPEDEFS =========================================================== */


/* ===== LOCAL ============================================================== */
static char _log_buffer[MAX_MESSAGE_LENGTH];
static void(*_transport)(const uint8_t *data, uint32_t size) = NULL;

/* -------------------------------------------------------------------------- */

/* ===== IMPLEMENTATION ===================================================== */

void logger_init(void(*write_fn)(const uint8_t *data, uint32_t size)) {

    _transport = write_fn;
}

/* -------------------------------------------------------------------------- */

void logger( const char* format, ...) {

    if(_transport == NULL) {
        return;
    }

    va_list args;
    va_start(args, format);

    _LOGGER_LOCK();

    vsnprintf(_log_buffer, MAX_MESSAGE_LENGTH, format, args);

    va_end(args);

    _transport((uint8_t*)_log_buffer, strlen(_log_buffer));

    _LOGGER_UNLOCK();
}

/* -------------------------------------------------------------------------- */
