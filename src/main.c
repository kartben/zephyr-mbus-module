/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <stdio.h>

#include <mbus.h>

void main(void)
{
	printf("Hello World!\n");
	mbus_init();
}
