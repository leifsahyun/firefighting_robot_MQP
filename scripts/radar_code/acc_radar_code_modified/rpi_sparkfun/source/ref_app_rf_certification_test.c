// Copyright (c) Acconeer AB, 2019
// All rights reserved

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_driver_hal.h"
#include "acc_hal_definitions.h"
#include "acc_rss.h"
#include "acc_rf_certification_test.h"
#include "acc_version.h"


static acc_hal_t hal;


static bool acc_ref_app_rf_certification_test(void);


int main(void)
{
	if (!acc_driver_hal_init())
	{
		return EXIT_FAILURE;
	}

	if (!acc_ref_app_rf_certification_test())
	{
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}


bool acc_ref_app_rf_certification_test(void)
{
	printf("Acconeer software version %s\n", acc_version_get());

	hal = acc_driver_hal_get_implementation();

	if (!acc_rss_activate(&hal))
	{
		fprintf(stderr, "Failed to activate RSS\n");
		return false;
	}

	bool success = acc_rf_certification_test(0, 0);

	acc_rss_deactivate();

    return success;
}
