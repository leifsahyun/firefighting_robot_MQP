// Original example from Acconeer, modified by Leif Sahyun

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
 * Uses the radar helper functions to take a single reading of the radar and print the distance measured
 * Author: Leif Sahyun
 */


int main(void)
{
	acc_driver_hal_init();
	acc_service_configuration_t config = service_envelope_setup();
	double dist = execute_envelope(config);
	service_envelope_takedown(config);
	
	printf("Distance to peak radiance: %f\n", dist);

	return EXIT_SUCCESS;
}



