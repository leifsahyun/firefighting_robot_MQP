// Copyright (c) Acconeer AB, 2018-2019
// All rights reserved

#ifndef ACC_DRIVER_HAL_H_
#define ACC_DRIVER_HAL_H_

#include "acc_definitions.h"
#include "acc_hal_definitions.h"


/**
 * @brief Initialize hal driver
 *
 * @return True if initialization is successful
 */
bool acc_driver_hal_init(void);


/**
 * @brief Get hal implementation reference
 */
acc_hal_t acc_driver_hal_get_implementation(void);


#endif
