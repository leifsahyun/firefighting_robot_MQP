#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_service.h"
#include "acc_service_envelope.h"

acc_service_configuration_t service_envelope_setup();
bool service_envelope_takedown(acc_service_configuration_t);
double execute_envelope(acc_service_configuration_t);
double sample_average_dist(acc_service_configuration_t, int);
