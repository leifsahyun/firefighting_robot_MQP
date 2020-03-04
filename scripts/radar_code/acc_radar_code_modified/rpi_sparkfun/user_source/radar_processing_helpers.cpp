#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_driver_hal.h"
#include "acc_hal_definitions.h"
#include "acc_rss.h"
#include "acc_service.h"
#include "acc_service_envelope.h"

#include "acc_version.h"
#include "radar_processing_helpers.h"

/**
 * Helper functions to setup and operate the Acconeer radar for FRED
 * Author: Leif Sahyun
 */

RadarHelper::acc_service_configuration_t service_envelope_setup(void)
{
	if (!acc_driver_hal_init())
	{
		return NULL;
	}

	printf("Acconeer software version %s\n", acc_version_get());

	acc_hal_t hal = acc_driver_hal_get_implementation();

	if (!acc_rss_activate(&hal))
	{
		fprintf(stderr, "acc_rss_activate() failed\n");
		return NULL;
	}

	acc_service_configuration_t envelope_configuration = acc_service_envelope_configuration_create();

	if (envelope_configuration == NULL)
	{
		fprintf(stderr, "acc_service_envelope_configuration_create() failed\n");
		acc_rss_deactivate();
		return NULL;
	}

	float start_m  = 0.2f;
	float length_m = 0.8f;

	acc_service_requested_start_set(envelope_configuration, start_m);
	acc_service_requested_length_set(envelope_configuration, length_m);

	//set the service profile to medium wavelets
	acc_service_profile_set(envelope_configuration, ACC_SERVICE_PROFILE_2);
/**
	if (!execute_envelope(envelope_configuration))
	{
		service_envelope_takedown(envelope_configuration);
		return false;
	}
*/
	return envelope_configuration;
}

bool RadarHelper::service_envelope_takedown(acc_service_configuration_t envelope_configuration)
{
	acc_service_envelope_configuration_destroy(&envelope_configuration);
	acc_rss_deactivate();
	return true;
}

double RadarHelper::execute_envelope(acc_service_configuration_t envelope_configuration)
{
	acc_service_handle_t handle = acc_service_create(envelope_configuration);

	if (handle == NULL)
	{
		fprintf(stderr, "acc_service_create() failed\n");
		return -1;
	}

	acc_service_envelope_metadata_t envelope_metadata;
	acc_service_envelope_get_metadata(handle, &envelope_metadata);

	printf("Start: %d mm\n", (int)(envelope_metadata.start_m * 1000.0f));
	printf("Length: %u mm\n", (unsigned int)(envelope_metadata.length_m * 1000.0f));
	printf("Data length: %u\n", (unsigned int)(envelope_metadata.data_length));

	uint16_t data[envelope_metadata.data_length];

	acc_service_envelope_result_info_t result_info;

	if (!acc_service_activate(handle))
	{
		fprintf(stderr, "acc_service_activate() failed\n");
		acc_service_destroy(&handle);
		return -1;
	}

	bool success = true;

	success = acc_service_envelope_get_next(handle, data, envelope_metadata.data_length, &result_info);

	if (!success)
	{
		fprintf(stderr, "acc_service_envelope_get_next() failed\n");
		return -1;
	}

	//Get a distance measurement
	uint16_t peak_index = 0;
	int peak_radiance = 0;
	for(uint16_t i=0; i<envelope_metadata.data_length; i++)
	{
		if(data[i]>peak_radiance)
		{
			peak_radiance = data[i];
			peak_index = i;
		}
	}
	double measured_distance = envelope_metadata.start_m + peak_index*envelope_metadata.step_length_m;
	printf("\n");
	printf("Total number of values recorded: %u\n", envelope_metadata.data_length);
	printf("Peak radiance: %d\n", peak_radiance);
	printf("Distance to peak: %f\n", measured_distance);

	//bool deactivated = acc_service_deactivate(handle);

	acc_service_destroy(&handle);

	return measured_distance;
}

double RadarHelper::sample_average_dist(acc_service_configuration_t envelope_configuration, int sample_size)
{
	double cum_dist = 0;
	for(int i=0; i<sample_size; i++)
	{
		cum_dist += execute_envelope(envelope_configuration);
		service_envelope_takedown(envelope_configuration);
	}
	return cum_dist/sample_size;
}

