/*
  *****************************************************************************
  * @file           : meit.c
  * @brief          : MEIT System API
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

#include "meit.h"
#include "gpio.h"

void meit_gpio_write(const uint8_t address, const uint16_t value)
{
  /* temporary code, only used for two CS pin of specific probe selection */
  switch (value)
  {
  case 1:
    // probe number 0
    HAL_GPIO_WritePin(SPIP_CS1_GPIO_Port, SPIP_CS1_Pin, 0);
    break;
  
  case 2:
    // probe number 1
    HAL_GPIO_WritePin(SPIP_CS2_GPIO_Port, SPIP_CS2_Pin, 0);
    break;
  
  default:
    HAL_GPIO_WritePin(SPIP_CS1_GPIO_Port, SPIP_CS1_Pin, 1);
    HAL_GPIO_WritePin(SPIP_CS2_GPIO_Port, SPIP_CS2_Pin, 1);
    break;
  }
}