/******************************************************************************
  * @file           : meit.h
  * @brief          : Header file for MEIT System API
  *****************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Roman Vaněk
  * All rights reserved.
  *
  * Dept. of Radio Electronics, Brno University of Technology, Czechia
  *
  *****************************************************************************
  */

#ifndef MEIT_MEIT_H_
#define MEIT_MEIT_H_

#include "main.h"

/** @addtogroup Multiplexer module structure definition
 */
typedef struct
{
  uint8_t initialized;
  uint8_t address;
  uint16_t gpio;

} Mux_TypeDef;

/** @addtogroup system function definition
 */
void meit_gpio_write(const uint8_t address, const uint16_t value);

#endif