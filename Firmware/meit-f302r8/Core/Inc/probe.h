/******************************************************************************
  * @file           : probe.h
  * @brief          : Header file for MEIT probe API
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
#ifndef PROBE_H_
#define PROBE_H_

/** @addtogroup MODULE PROBE
 */
#include "main.h"
#include "meit.h"

/** @addtogroup Active probe
 */
/** @addtogroup PGA280 interface commands
 */
#define PGA280_CMD_WRITE           0b01100000
#define PGA280_CMD_READ            0b10000000

/** @addtogroup PGA280 register default values
 */
#define PGA280_DEFAULT_GPIO_MODE   0b01111111
#define PGA280_GPIO_LED_RESET      0b00000111
#define PGA280_GPIO_MUX_RESET      0b01111000

#define PGA280_INPUT1_SET          0b01100000
#define PGA280_INPUT2_SET          0b00011000

/** @addtogroup Active probe uncalibrated attenuations
 */
#define _PROBE_RANGESET_EXTENDED_GAIN (float)1.0f
#define _PROBE_RANGESET_13V_GAIN      (float)1.0f
#define _PROBE_RANGESET_15V_GAIN      (float)0.879166666666667f
#define _PROBE_RANGESET_17V_GAIN      (float)0.758333333333333f
#define _PROBE_RANGESET_20V_GAIN      (float)0.6375f
#define _PROBE_RANGESET_25V_GAIN      (float)0.516666666666667f
#define _PROBE_RANGESET_33V_GAIN      (float)0.395833333333333f
#define _PROBE_RANGESET_48V_GAIN      (float)0.275f

/** @addtogroup PGA280 registres bases
 */
typedef enum 
{
  PGA280_GAINMUX_BASE        = 0,
  PGA280_SOFTRESET_BASE      = 1,
  PGA280_GPIOSPIMODE_BASE    = 2,
  PGA280_BUFTIMEOUT_BASE     = 3,
  PGA280_ERROR_BASE          = 4,
  PGA280_GPIO_BASE           = 5,
  PGA280_INPUTSWITCH1_BASE   = 6,
  PGA280_INPUTSWITCH2_BASE   = 7,
  PGA280_GPIOMODE_BASE       = 8,
  PGA280_EXCS_BASE           = 9,
  PGA280_CONF1_BASE          = 10,
  PGA280_CONF2_BASE          = 11,
  PGA280_SPECIALFUNC_BASE    = 12,

} PGA280ProbeRegister_Type;

/** @addtogroup Active selectable gains or attenuations
 * @brief Should be used only for specific gain/att settings
 * via probe_gain_specific() function.
 */
typedef enum
{
  PROBE_GAINSET_0125 = 0,
  PROBE_GAINSET_025,
  PROBE_GAINSET_05,
  PROBE_GAINSET_1,
  PROBE_GAINSET_2,
  PROBE_GAINSET_4,
  PROBE_GAINSET_8,
  PROBE_GAINSET_16,
  PROBE_GAINSET_32,
  PROBE_GAINSET_64,
  PROBE_GAINSET_128,

} ProbeGainSet_Type;

typedef enum
{
  PROBE_RANGESET_EXTENDED = 0,
  PROBE_RANGESET_13V,
  PROBE_RANGESET_15V,
  PROBE_RANGESET_17V,
  PROBE_RANGESET_20V,
  PROBE_RANGESET_25V,
  PROBE_RANGESET_33V,
  PROBE_RANGESET_48V,

} ProbeRangeSet_Type;

/** @addtogroup Active probe selectable gains
 * @brief Gains are stored for each probe separately thus calibration
 * can be applied to compensate voltage gain errors.
 */
typedef struct
{
  float gain025_range48;
  float gain025_range33;
  float gain025_range25;
  float gain025_range20;
  float gain025_range17;
  float gain025_range15;
  float gain025_range13;
  // gain_index = 7
  float gain05_range20;
  float gain05_range17;
  float gain05_range15;
  float gain05_range13;
  // gain_index = 11
  float gain1_range20;
  float gain1_range17;
  float gain1_range15;
  float gain1_range13;
  // gain_index = 15
  float gain2_range20;
  float gain2_range17;
  float gain2_range15;
  float gain2_range13;
  // gain_index = 19
  float gain4_range20;
  float gain4_range17;
  float gain4_range15;
  float gain4_range13;
  // gain_index = 23
  float gain8_range20;
  float gain8_range17;
  float gain8_range15;
  float gain8_range13;
  // gain_index = 27
  float gain16_range20;
  float gain16_range17;
  float gain16_range15;
  float gain16_range13;
  // gain_index = 31
  float gain32_range20;
  float gain32_range17;
  float gain32_range15;
  float gain32_range13;
  // gain_index = 35
  float gain64_range20;
  float gain64_range17;
  float gain64_range15;
  float gain64_range13;
  // gain_index = 39
  float gain128_range20;
  float gain128_range17;
  float gain128_range15;
  float gain128_range13;
  // max 42 index
} ProbeGains_Type;

/** @addtogroup Probe mode state/status definition
 */
typedef enum
{
  PROBE_MODE_RESET = 0,
  PROBE_MODE_SET = 1,
} ProbeModeStatus_Type;

/** @addtogroup Probe modes definition
 * @brief All the mode probe that can be activated
 * by passing them as a prarameter of probe_mode function.
 * Also active probe gpio (bit address - 3) coresponding to 
 * spectific LED mode indication
 */
typedef enum
{
  PROBE_MODE_CURRENT_POSITIVE = 0,
  PROBE_MODE_REFERENCE = 1,
  PROBE_MODE_MEASUREMENT = 2,
  PROBE_MODE_CURRENT_NEGATIVE = 3, 
} ProbeMode_Type;

/** @addtogroup Probe structure definition
 */
typedef struct
{
  uint8_t initialized;
  uint8_t number;
  /** @brief storing last written gpio register of PGA mounted on probe */
  uint8_t gpio_reg;

  /** @brief Stores individual mode states currectly activated for the probe,
   * index of the array corespond to the probe mode declared in ProbeMode_Type */
  ProbeModeStatus_Type modes[4];

  // index of currently selected gain from gains
  uint8_t gain_index;
  // calibrated values of actual configurable gains
  ProbeGains_Type gains;
  // currenty selected gain and range of PGA and ATTENUATOR in the probe respectively
  ProbeGainSet_Type gain;
  ProbeRangeSet_Type range;

  // pointers to SPI communication handler and MUX module handler for probe selection
  SPI_HandleTypeDef* hspix;
  Mux_TypeDef* hmuxx;

} Probe_TypeDef;

/** @addtogroup Probe functions definition
 */
void probe_init(Probe_TypeDef* probe, SPI_HandleTypeDef* hspix, Mux_TypeDef* hmuxx, const uint8_t number);
void probe_write(Probe_TypeDef* probe, const PGA280ProbeRegister_Type register_base, const uint8_t value);
void probe_mode(Probe_TypeDef* probe, const ProbeMode_Type mode, const ProbeModeStatus_Type status);
void probe_input(Probe_TypeDef* probe, const uint8_t input_n);
void probe_gain_specific(Probe_TypeDef* probe, ProbeGainSet_Type gain, ProbeRangeSet_Type range);

void probe_gain(Probe_TypeDef* probe, uint8_t const gain_index);
uint8_t probe_gain_increase(Probe_TypeDef* probe);
uint8_t probe_gain_decrease(Probe_TypeDef* probe);

float probe_voltage(Probe_TypeDef* probe, const uint32_t adc_value);
void probe_calibrate(Probe_TypeDef* probe, const float calib_gain, const uint8_t gain_index);

void probe_error_print(Probe_TypeDef* probe);


#endif //PROBE_H_