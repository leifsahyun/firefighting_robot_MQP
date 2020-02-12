// Copyright (c) Acconeer AB, 2016-2019
// All rights reserved

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_log.h"

#include "acc_definitions.h"
#include "acc_device_os.h"
#include "acc_hal_definitions.h"
#include "acc_log_integration.h"


#define LOG_BUFFER_MAX_SIZE 220


static char            log_buffer[LOG_BUFFER_MAX_SIZE];
static acc_app_integration_mutex_t  mutex           = NULL;
static acc_log_level_t log_level_limit = ACC_LOG_LEVEL_INFO;


/**
 * @brief Perform any init of log module
 */
static void acc_log_init(void)
{
	static bool init_done = false;

	if (init_done)
	{
		return;
	}

	mutex = acc_os_mutex_create();

	assert(mutex != NULL);

	init_done = true;
}


void acc_log(acc_log_level_t level, const char *module, const char *format, ...)
{
	va_list ap;

	if (level > log_level_limit)
	{
		return;
	}

	acc_log_init();

	va_start(ap, format);

	acc_os_mutex_lock(mutex);

	int ret = vsnprintf(log_buffer, LOG_BUFFER_MAX_SIZE, format, ap);
	if (ret >= LOG_BUFFER_MAX_SIZE)
	{
		log_buffer[LOG_BUFFER_MAX_SIZE - 4] = '.';
		log_buffer[LOG_BUFFER_MAX_SIZE - 3] = '.';
		log_buffer[LOG_BUFFER_MAX_SIZE - 2] = '.';
		log_buffer[LOG_BUFFER_MAX_SIZE - 1] = 0;
	}

	acc_log_integration(level, module, log_buffer);

	acc_os_mutex_unlock(mutex);

	va_end(ap);
}
