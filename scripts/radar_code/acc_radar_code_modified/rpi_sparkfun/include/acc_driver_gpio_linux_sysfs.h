// Copyright (c) Acconeer AB, 2016-2019
// All rights reserved

#ifndef ACC_DRIVER_GPIO_LINUX_SYSFS_H_
#define ACC_DRIVER_GPIO_LINUX_SYSFS_H_

#include <stdbool.h>
#include <stdint.h>

#include "acc_app_integration.h"
#include "acc_device_gpio.h"


/**
 * @brief GPIO pin direction
 */
typedef enum
{
	GPIO_DIR_IN,
	GPIO_DIR_OUT,
	GPIO_DIR_UNKNOWN
} gpio_dir_enum_t;
typedef uint32_t gpio_dir_t;

/**
 * @brief GPIO pin information
 */
typedef struct
{
	bool                        is_open;
	uint_fast8_t                pin;
	int                         dir_fd;
	int                         value_fd;
	gpio_dir_t                  dir;
	int_fast8_t                 value;
	uint_fast8_t                pull;
	acc_app_integration_mutex_t mutex;
	acc_device_gpio_isr_t       isr;
	void                        *thread_handle;
} gpio_t;


/**
 * @brief Request driver to register with appropriate device(s)
 *
 * @param pin_count The maximum number of pins supported
 * @param[in] gpio_mem Memory to be used be GPIO driver
 */
void acc_driver_gpio_linux_sysfs_register(uint_fast16_t pin_count, gpio_t *gpio_mem);


#endif
