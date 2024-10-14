/*
  *****************************************************************************
  * @file           : probe.c
  * @brief          : MEIT probe API
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
#include "stm32f3xx_hal_spi.h"
#include "stm32f3xx_hal_gpio.h"

#include "mux.h"
#include "probe.h"
#include "config.h"

#ifdef LOG_SEMIHOSTING
#include <stdio.h>
#endif

// trecking probe number of currenty selected modes in the mux
int probe_mode_numbers[4] = {-1,-1,-1,-1};

// access to all the probes
extern Probe_TypeDef probes[];

// default gain values
ProbeGains_Type probe_gains_default = {
  .gain025_range48 = 0.25f * _PROBE_RANGESET_48V_GAIN,
  .gain025_range33 = 0.25f * _PROBE_RANGESET_33V_GAIN,
  .gain025_range25 = 0.25f * _PROBE_RANGESET_25V_GAIN,
  .gain025_range20 = 0.25f * _PROBE_RANGESET_20V_GAIN,
  .gain025_range17 = 0.25f * _PROBE_RANGESET_17V_GAIN,
  .gain025_range15 = 0.25f * _PROBE_RANGESET_15V_GAIN,
  .gain025_range13 = 0.25f * _PROBE_RANGESET_13V_GAIN,
  
  .gain05_range20 = 0.5f * _PROBE_RANGESET_20V_GAIN,
  .gain05_range17 = 0.5f * _PROBE_RANGESET_17V_GAIN,
  .gain05_range15 = 0.5f * _PROBE_RANGESET_15V_GAIN,
  .gain05_range13 = 0.5f * _PROBE_RANGESET_13V_GAIN,
  
  .gain1_range20 = 1.0f * _PROBE_RANGESET_20V_GAIN,
  .gain1_range17 = 1.0f * _PROBE_RANGESET_17V_GAIN,
  .gain1_range15 = 1.0f * _PROBE_RANGESET_15V_GAIN,
  .gain1_range13 = 1.0f * _PROBE_RANGESET_13V_GAIN,

  .gain2_range20 = 2.0f * _PROBE_RANGESET_20V_GAIN,
  .gain2_range17 = 2.0f * _PROBE_RANGESET_17V_GAIN,
  .gain2_range15 = 2.0f * _PROBE_RANGESET_15V_GAIN,
  .gain2_range13 = 2.0f * _PROBE_RANGESET_13V_GAIN,

  .gain4_range20 = 4.0f * _PROBE_RANGESET_20V_GAIN,
  .gain4_range17 = 4.0f * _PROBE_RANGESET_17V_GAIN,
  .gain4_range15 = 4.0f * _PROBE_RANGESET_15V_GAIN,
  .gain4_range13 = 4.0f * _PROBE_RANGESET_13V_GAIN,

  .gain8_range20 = 8.0f * _PROBE_RANGESET_20V_GAIN,
  .gain8_range17 = 8.0f * _PROBE_RANGESET_17V_GAIN,
  .gain8_range15 = 8.0f * _PROBE_RANGESET_15V_GAIN,
  .gain8_range13 = 8.0f * _PROBE_RANGESET_13V_GAIN,

  .gain16_range20 = 16.0f * _PROBE_RANGESET_20V_GAIN,
  .gain16_range17 = 16.0f * _PROBE_RANGESET_17V_GAIN,
  .gain16_range15 = 16.0f * _PROBE_RANGESET_15V_GAIN,
  .gain16_range13 = 16.0f * _PROBE_RANGESET_13V_GAIN,

  .gain32_range20 = 32.0f * _PROBE_RANGESET_20V_GAIN,
  .gain32_range17 = 32.0f * _PROBE_RANGESET_17V_GAIN,
  .gain32_range15 = 32.0f * _PROBE_RANGESET_15V_GAIN,
  .gain32_range13 = 32.0f * _PROBE_RANGESET_13V_GAIN,

  .gain64_range20 = 64.0f * _PROBE_RANGESET_20V_GAIN,
  .gain64_range17 = 64.0f * _PROBE_RANGESET_17V_GAIN,
  .gain64_range15 = 64.0f * _PROBE_RANGESET_15V_GAIN,
  .gain64_range13 = 64.0f * _PROBE_RANGESET_13V_GAIN,

  .gain128_range20 = 128.0f * _PROBE_RANGESET_20V_GAIN,
  .gain128_range17 = 128.0f * _PROBE_RANGESET_17V_GAIN,
  .gain128_range15 = 128.0f * _PROBE_RANGESET_15V_GAIN,
  .gain128_range13 = 128.0f * _PROBE_RANGESET_13V_GAIN,
};

/**
 * @brief This function set default/startup bits in the PGA280 registers. It also assignes 
 * number to the probe instance and set specific SPIx inteface and CS/probe selection 
 * for communication with the active probes.
 * 
 * It will soft-restart the PGA280, enable MUX control throuhg special functiuon register 
 * and itterate through all the probe indication LEDs for successfull init visualization.
 * 
 * @note Function shall be called after SPI initialization, hspix definition, CS pin and 
 * GPIO initialization. function services as communication configuration setting 
 * for the probe API, so it need to be called before any API operation.
 * 
 * @param probe refers to probe definition.
 * @param hspix SPI handle Structure of SPI interface connected to the probes.
 * @param hmuxx MUX module handler to which is the probe connected to.
 * @param number specifies number label to be assigned for the probe.
 */
void probe_init(Probe_TypeDef* probe, SPI_HandleTypeDef* hspix, Mux_TypeDef* hmuxx, const uint8_t number)
{
  // non-defined input handling
  if (probe == NULL) goto catch_null_probe;
  if (hspix == NULL) goto catch_null_hspix;
  if (hmuxx == NULL) goto catch_null_hmuxx;
  if (probe->initialized == 1)
    log("INFO: Probe:%d already initialized, re-initialisation.\n", probe->number);

  // init communication interfaces
  probe->hspix = hspix;
  probe->hmuxx = hmuxx;

  probe->number = number;

  // probe is initialized
  probe->initialized = 1;

  // probe default gains
  probe->gains = probe_gains_default;

  // manualy set mode of probe, not recommended outside of init function
  // proventing LED indication before probe init caused by probe_mode function
  probe_mode(probe, PROBE_MODE_MEASUREMENT, RESET);

  // soft-reset probe
  // mux_select(hmuxx, PROBE_MODE_MEASUREMENT, probe->number, PROBE_MODE_SET);
  probe_write(probe, PGA280_SOFTRESET_BASE, 1);
  HAL_Delay(10);

  // special function register
  // enable MUX control through gain control register 0
  probe_write(probe, PGA280_SPECIALFUNC_BASE, 0b111);

  // decrese default buffer timeout
  probe_write(probe, PGA280_BUFTIMEOUT_BASE, 3);

  // maximmut attenuation as default and gain of 1V/V
  probe_gain_specific(probe, PROBE_GAINSET_1, PROBE_RANGESET_48V);
  // default input selection
  probe_input(probe, 2);

  // configuring default GPIO directions
  probe_write(probe, PGA280_GPIOMODE_BASE, PGA280_DEFAULT_GPIO_MODE);

  // testing probe functions
  for(uint8_t i = 0; i < 4; i++)
  {
    // turn on mounted leds one by one
    probe_mode(probe, i, PROBE_MODE_SET);
    HAL_Delay(200);
  }
  // turn off all the LEDs
  for(uint8_t i = 0; i < 4; i++)
    probe_mode(probe, i, PROBE_MODE_RESET);

  // reset error bit
  probe_write(probe, PGA280_ERROR_BASE, 0xFF);

  return;

  // error handling
catch_null_probe:
    log("ERROR: Define probe before initialization.\n");
catch_null_hspix:
    log("ERROR: Invalid/not defined SPIx handler for probe:%d communication.\n", probe->number);
catch_null_hmuxx:
    log("ERROR: Invalid/not defined MUX module handler for probe:%d selection.\n", probe->number);
}

/**
 * @brief Function that writes to register of PGA280 mounted on probe using SPI interface.
 * 
 * @note probe_comm_init shall be called before.
 * 
 * @param probe specifies to which probe you want to the mode.
 * @param register_base specifies PGA280 register you want to write to, refer to PGA280ProbeRegister_Type.
 * @param value that will be writen to the register.
 */
void probe_write(Probe_TypeDef* probe, const PGA280ProbeRegister_Type register_base, const uint8_t value)
{ 
  if (probe == NULL) goto catch_null_probe;
  if (probe->initialized == 0) goto catch_initialized;

  uint8_t data[2] = {0};
  data[0] |= PGA280_CMD_WRITE;
  data[0] |= register_base;
  data[1] |= value;

  mux_select(probe->hmuxx, PROBE_MODE_MEASUREMENT, probe->number, PROBE_MODE_SET);
  HAL_SPI_Transmit(probe->hspix, data, 2, 100);
  mux_select(probe->hmuxx, PROBE_MODE_MEASUREMENT, probe->number, PROBE_MODE_RESET);
  if (probe_mode_numbers[PROBE_MODE_MEASUREMENT] != -1)
    mux_select(probe->hmuxx, PROBE_MODE_MEASUREMENT, probe_mode_numbers[PROBE_MODE_MEASUREMENT], PROBE_MODE_SET);
  return;

catch_null_probe:
  log("ERROR: Define probe before initialization\n");
catch_initialized:
  log("ERROR: Probe is not initialized, call probe_init function.\n");
}

/**
 * @brief Function for probe mode selection, refer to ProbeMode_Type.
 * Mode indication via LEDs mounted on Probe.
 * 
 * @note For probes input enabling PROBE_MODE_MEASUREMENT must be set.
 * PROBE_MODE_POSITIVE and PROBE_MODE_NEGATIVE modes can't be set 
 * at the same time on one probe. Whan this forbidden event acours, 
 * first the oposite mode is reseted and after that desired mode is set.
 * 
 * @param probe specifies on which probe you want to change the mode.
 * @param mode refers to ProbeMode_Type.
 * @param status specifies the state to which the mode will be set to (ACTIVE/RESET).
 */
void probe_mode(Probe_TypeDef* probe, const ProbeMode_Type mode, const ProbeModeStatus_Type status)
{
  if (probe == NULL) goto catch_null_probe;
  if (probe->initialized == 0) goto catch_initialized;

  // control if the statuc is already set, esential for stopping
  // probe_write -> probe_mode -> probe_mode potential inflinity loop
  if(probe->modes[mode] == status) return;

  if (status == PROBE_MODE_RESET)
  {
    probe->gpio_reg &= ~((uint16_t)1 << (3 + mode));
    probe_mode_numbers[mode] = -1;

    // LED indication of selected modes mounted on probe
    probe_write(probe, PGA280_GPIO_BASE, probe->gpio_reg);

    // update and mode selection on mux module
    mux_select(probe->hmuxx, mode, probe->number, status);
    // set desired status of the mode
    probe->modes[mode] = status;
    return;
  }

  // check if there are any probes in oposite mode in current type mode,
  // both current modes (positive and negative) can't be set at the same time in the same probe
  if (mode == PROBE_MODE_CURRENT_POSITIVE && probe->modes[PROBE_MODE_CURRENT_NEGATIVE] == PROBE_MODE_SET)
  {
    log("INFO: Positive and negative current modes can't be set at the same time, disabling CURRENT_NEGATIVE mode on probe:%d\n", probe->number);
    probe_mode(probe, PROBE_MODE_CURRENT_NEGATIVE, PROBE_MODE_RESET);
  }
  else if (mode == PROBE_MODE_CURRENT_NEGATIVE && probe->modes[PROBE_MODE_CURRENT_POSITIVE] == PROBE_MODE_SET)
  {
    log("INFO: Positive and negative current modes can't be set at the same time, disabling CURRENT_POSITIVE mode on probe:%d\n", probe->number);
    probe_mode(probe, PROBE_MODE_CURRENT_POSITIVE, PROBE_MODE_RESET);
  }

  // Diactivate probe where the mode is currently active in multiplexor to which the probes are
  // connected if there are any. Mode can't be se on multiple probe at the same time.
  if (probe_mode_numbers[mode] != -1 && probe->number != probe_mode_numbers[mode])
    probe_mode(&probes[probe_mode_numbers[mode]], mode, PROBE_MODE_RESET);

  // update and mode selection on mux module
  mux_select(probe->hmuxx, mode, probe->number, status);

  // set desired status of the mode
  probe->modes[mode] = status;

  // set the gpio bits coresponding to LEDs mounted on probe
  // update probe number to mux mode tracker
  probe->gpio_reg |= ((uint16_t)1 << (3 + mode));
  probe_mode_numbers[mode] = probe->number;

  // LED indication of selected modes mounted on probe
  probe_write(probe, PGA280_GPIO_BASE, probe->gpio_reg);

  return;

catch_null_probe:
  log("ERROR: Define probe before initialization\n");
catch_initialized:
  log("ERROR: Probe is not initialized, call probe_init function.\n");
}

void probe_input(Probe_TypeDef* probe, const uint8_t input_n)
{
  if (probe == NULL) goto catch_null_probe;
  if (probe->initialized == 0) goto catch_initialized;
  if (input_n > 2 || input_n < 1) goto catch_input_n;

  if (input_n == 1)
    probe_write(probe, PGA280_INPUTSWITCH1_BASE, PGA280_INPUT1_SET);
  else
    probe_write(probe, PGA280_INPUTSWITCH1_BASE, PGA280_INPUT2_SET);

  return;

catch_null_probe:
  log("ERROR: Define probe before initialization.\n");
catch_input_n:
  log("WARNING: Input selection for probe:%d is out of scope.\n", probe->number);
catch_initialized:
  log("ERROR: Probe is not initialized, call probe_init function.\n");
}

void probe_gain_specific(Probe_TypeDef* probe, ProbeGainSet_Type gain, ProbeRangeSet_Type range)
{
  if (probe == NULL) goto catch_null_probe;
  if (probe->initialized == 0) goto catch_initialized;

  if (probe->modes[PROBE_MODE_MEASUREMENT] != PROBE_MODE_SET)
    log("WARNING: Probe:%d is not in measurement mode, gain setting will not propagate to MEIT system ADC input.\n", probe->number);
  
  // limiting the selection of gain
  if (gain < PROBE_GAINSET_0125)
  {
    gain = PROBE_GAINSET_0125;
    log("INFO: Minimum range reached in probe:%d.\n", probe->number);
  }
  else if (gain > PROBE_GAINSET_128)
  {
    gain = PROBE_GAINSET_128;
    log("INFO: Maximum range reached in probe:%d.\n", probe->number);
  }

  // limiting the selection of range
  if (range < PROBE_RANGESET_EXTENDED)
  {
    range = PROBE_RANGESET_EXTENDED;
    log("INFO: Minimum range reached in probe:%d.\n", probe->number);
  }
  else if (range > PROBE_RANGESET_48V)
  {
    range = PROBE_RANGESET_48V;
    log("INFO: Maximum range reached in probe:%d.\n", probe->number);
  }
  
  // switching inputs of PGA280, selecting between aditional amplifier or variable attenuator
  if (range == PROBE_RANGESET_EXTENDED)
    probe_input(probe, 1);
  else
    probe_input(probe, 2);
  
   
  uint8_t mux_gpio = 0;
  switch (range)
  {
  case PROBE_RANGESET_48V:
    mux_gpio = 0;
    break;
  case PROBE_RANGESET_33V:
    mux_gpio = 6;
    break;
  case PROBE_RANGESET_25V:
    mux_gpio = 2;
    break;
  case PROBE_RANGESET_20V:
    mux_gpio = 7;
    break;
  case PROBE_RANGESET_17V:
    mux_gpio = 4;
    break;
  case PROBE_RANGESET_15V:
    mux_gpio = 5;
    break;
  case PROBE_RANGESET_13V:
    mux_gpio = 1;
    break;
  case PROBE_RANGESET_EXTENDED:
    log("INFO: Swithed to additional gain/extended range of probe:%d.\n", probe->number);
    mux_gpio = 3;
    break;
  
  default:
    log("ERROR: Bad probe:%d range selection, selecting maximum range.\n", probe->number);
    mux_gpio = 0;
    break;
  }

  uint8_t reg = (gain << 3) | (mux_gpio & 0b00000111);
  probe_write(probe, PGA280_GAINMUX_BASE, reg);

  probe->gpio_reg &= PGA280_GPIO_MUX_RESET;
  probe->gpio_reg |= (mux_gpio & 0b00000111);

  probe->range = range;
  probe->gain = gain;

  // provide 16 SCLKs
  uint8_t tmp = 0;
  tmp |= PGA280_CMD_READ;
  tmp |= PGA280_GAINMUX_BASE;

  probe_mode(probe, PROBE_MODE_MEASUREMENT, PROBE_MODE_SET);
  // mux_select_probe(probe->hmuxx, probe->number);
  HAL_SPI_Transmit(probe->hspix, &tmp, 1, 100);
  HAL_SPI_Receive(probe->hspix, &tmp, 1, 100);
  // mux_deselect_probe(probe->hmuxx, probe->number);

  return;

catch_null_probe:
  log("ERROR: Define probe before initialization.\n");
catch_initialized:
  log("ERROR: Probe is not initialized, call probe_init function.\n");
}

void probe_gain(Probe_TypeDef* probe, const uint8_t gain_index)
{
  if (probe == NULL) goto catch_null_probe;
  if (probe->initialized == 0) goto catch_initialized;
  if (gain_index > (sizeof(probe->gains) / sizeof(probe->gains.gain025_range13))) goto catch_outof_range;

  if (gain_index < 7)
    probe_gain_specific(probe, PROBE_GAINSET_025, PROBE_RANGESET_48V - gain_index);
  else
  {
    ProbeGainSet_Type gain = 2 + ((gain_index - 7) / 4);
    ProbeRangeSet_Type range = PROBE_RANGESET_20V - ((gain_index - 7) % 4);
    probe_gain_specific(probe, gain, range);
  }
  
  probe->gain_index = gain_index;
  return;

catch_null_probe:
  log("ERROR: Define probe before initialization.\n");
catch_initialized:
  log("ERROR: Probe is not initialized, call probe_init function.\n");
catch_outof_range:
  log("ERROR: Probe gain index:%d is out out of range.\n", gain_index);
}

uint8_t probe_gain_increase(Probe_TypeDef* probe)
{
  if (probe == NULL) goto catch_null_probe;

  uint8_t incresed = 1;
  if ((probe->gain_index + 1) > (sizeof(probe->gains) / sizeof(probe->gains.gain025_range13)))
    incresed = 0;
  else
    probe_gain(probe, probe->gain_index + 1);
  
  return incresed;

catch_null_probe:
  log("ERROR: Define probe before initialization.\n");
  return 0;
}

uint8_t probe_gain_decrease(Probe_TypeDef* probe)
{
  if (probe == NULL) goto catch_null_probe;

  uint8_t decresed = 1;
  if (probe->gain_index == 0)
    decresed = 0;
  else
    probe_gain(probe, probe->gain_index - 1);
  
  return decresed;

catch_null_probe:
  log("ERROR: Define probe before initialization.\n");
  return 0;
}

float probe_voltage(Probe_TypeDef* probe, const uint32_t adc_value)
{
  if (probe == NULL) goto catch_null_probe;

  float gain = ((float *)(&probe->gains))[probe->gain_index];
  return (float)(2.0f * (3.3f * (adc_value / 4096.0f) - 1.65f) / gain);

catch_null_probe:
  log("ERROR: Define probe before initialization.\n");
  return 0;
}

void probe_calibrate(Probe_TypeDef* probe, const float calib_gain, const uint8_t gain_index)
{
  if (probe == NULL) goto catch_null_probe;

  ((float *)(&probe->gains))[gain_index] = calib_gain;
  return;

catch_null_probe:
  log("ERROR: Define probe before initialization.\n");
  return;
}

/**
 * @brief Function that prints error register of PGA280 mounted on probe.
 * @note Should not be usen in final product, function assumes that we can read from probe.
 * 
 * @param probe specifies from which probe you want read.
 */
void probe_error_print(Probe_TypeDef* probe)
{
  if (probe == NULL) goto catch_null_probe;

  // reset error bit
  probe_write(probe, PGA280_ERROR_BASE, 0xFF);

  uint8_t data = 0;
  data |= PGA280_CMD_READ;
  data |= PGA280_ERROR_BASE;
  probe_mode(probe, PROBE_MODE_MEASUREMENT, PROBE_MODE_SET);
  // mux_select_probe(probe->hmuxx, probe->number);
  HAL_SPI_Transmit(probe->hspix, &data, 1, 100);
  HAL_SPI_Receive(probe->hspix, &data, 1, 100);
  // mux_deselect_probe(probe->hmuxx, probe->number);


  if (data != 0) 
    log("ERROR: PGA280 of probe:%d error register listing:\n", probe->number);
  else 
  {
    log("INFO: PGA280 no error bits present in error register or could not read from probe.\n");
    return;
  }
  
  for (uint8_t bit_pos = 0; bit_pos < 8; bit_pos++)
  {
    // check if the error bot is even set
    if (!(data & (1 << bit_pos))) continue;

    switch (bit_pos)
    {
      case 0:
        log("   PGA280 ERROR: Input Overvoltage\n");
        break;
      case 1:
        log("   PGA280 ERROR: Gain Network Overload\n");
        break;
      case 2:
        log("   PGA280 ERROR: Output Stage Error (allow approximately 6µs activation delay)\n");
        break;
      case 3:
        log("   PGA280 ERROR: Error Flag. Logic OR combination of error bits of Register 10\n");
        break;
      case 4:
        log("   PGA280 ERROR: Input Clamp Active\n");
        break;
      case 5:
        log("   PGA280 ERROR: Buffer Active\n");
        break;
      case 6:
        log("   PGA280 ERROR: Input Amplifier Saturation\n");
        break;
      case 7:
        log("   PGA280 ERROR: Checksum error in SPI.\n");
        break;
      
      default:
        break;
    }
  }
  return;

catch_null_probe:
  log("ERROR: Define probe before initialization\n");
}