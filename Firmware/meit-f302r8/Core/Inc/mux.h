/******************************************************************************
  * @file           : meit_mux.h
  * @brief          : Header file for MEIT Multiplexer module
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
#ifndef MEIT_MUX_H_
#define MEIT_MUX_H_

/** @addtogroup MEIT MODULE API
 */

#include "main.h"
#include "meit.h"
#include "probe.h"

/** @addtogroup Modules system address 
 */
#define MEIT_MUX_A_ADDRESS 0


void mux_init(Mux_TypeDef* mux, const uint8_t address);
void mux_select(Mux_TypeDef* mux, const ProbeMode_Type mode, const uint8_t number, const ProbeModeStatus_Type status);

#endif