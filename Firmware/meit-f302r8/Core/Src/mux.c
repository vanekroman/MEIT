/*
  *****************************************************************************
  * @file           : mux.h
  * @brief          : MEIT Multiplexer module API
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

#include "main.h"

#include "config.h"

#include "meit.h"
#include "mux.h"

void mux_init(Mux_TypeDef* mux, const uint8_t address)
{
  if (mux == NULL) goto catch_null_mux;
  
  mux->address = address;
  mux->gpio = 0;
  mux->initialized = 1;

  return;
  
catch_null_mux:
  log("ERROR: Define mux before initialization.\n");
}


void mux_select(Mux_TypeDef* mux, const ProbeMode_Type mode, const uint8_t number, const ProbeModeStatus_Type status)
{
  if (mux == NULL) goto catch_null_mux;

  /*  temporary code, only used for measurement mode swithing */
  if (mode == PROBE_MODE_MEASUREMENT)
  {
    // shift/assing number to GPIOs located on module
    if (status == PROBE_MODE_SET)
      mux->gpio |= ((uint16_t)1 << number);
    else // PROBE_MODE_RESET
      mux->gpio &= ~((uint16_t)1 << number);

    // setting speficic bits of mux gpio expader
    meit_gpio_write(mux->address, mux->gpio);
  }

  return;

catch_null_mux:
  log("ERROR: Define mux before initialization.\n");
}