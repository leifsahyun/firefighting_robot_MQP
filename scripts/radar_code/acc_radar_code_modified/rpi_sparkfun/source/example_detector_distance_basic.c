// Copyright (c) Acconeer AB, 2018-2019
// All rights reserved

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_definitions.h"
#include "acc_detector_distance_basic.h"
#include "acc_driver_hal.h"
#include "acc_hal_definitions.h"
#include "acc_rss.h"
#include "acc_version.h"


/**
 * @brief Example that shows how to use the distance basic detector
 *
 * This is an example on how the envelope service can be used.
 * The example executes as follows:
 *   - Activate Radar System Software (RSS)
 *   - Create a distance basic detector
 *   - Get the result and print it 10 times
 *   - Destroy the distance basic detector
 *   - Deactivate Radar System Software (RSS)
 */


static bool acc_example_detector_distance_basic(void);


int main(void)
{
	if (!acc_driver_hal_init())
	{
		return EXIT_FAILURE;
	}

	if (!acc_example_detector_distance_basic())
	{
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}


bool acc_example_detector_distance_basic(void)
{
	printf("Acconeer software version %s\n", acc_version_get());

	acc_hal_t hal = acc_driver_hal_get_implementation();

	hal.log.log_level = ACC_LOG_LEVEL_ERROR;

	if (!acc_rss_activate(&hal))
	{
		return false;
	}

	acc_sensor_id_t sensor_id = 1;
	float           start_m   = 0.2f;
	float           length_m  = 0.5f;

	acc_detector_distance_basic_handle_t handle = acc_detector_distance_basic_create(sensor_id, start_m, length_m);
	if (handle == NULL)
	{
		fprintf(stderr, "acc_detector_distance_basic_create() failed\n");
		acc_rss_deactivate();
		return false;
	}

	acc_detector_distance_basic_reflection_t reflection;

	const int iterations = 10;

	for (int i = 0; i < iterations; i++)
	{
		reflection = acc_detector_distance_basic_get_reflection(handle);

		printf("%d mm (%u)\n", (int)(reflection.distance * 1000.0f), (unsigned int)reflection.amplitude);
	}

	acc_detector_distance_basic_destroy(&handle);

	acc_rss_deactivate();

	return true;
}
