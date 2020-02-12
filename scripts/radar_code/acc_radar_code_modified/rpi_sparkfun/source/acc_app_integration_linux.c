// Copyright (c) Acconeer AB, 2019
// All rights reserved

#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif

#include "acc_app_integration.h"

#include <errno.h>
#include <time.h>

void acc_app_integration_sleep_us(uint32_t time_usec)
{
	int             ret = 0;
	struct timespec ts;
	struct timespec remain;

	if (time_usec == 0)
	{
		time_usec = 1;
	}

	ts.tv_sec  = time_usec / 1000000;
	ts.tv_nsec = (time_usec % 1000000) * 1000;

	do {
		ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &remain);
		ts = remain;
	} while (ret == EINTR);
}


void acc_app_integration_sleep_ms(uint32_t time_msec)
{
	acc_app_integration_sleep_us(time_msec * 1000);
}
