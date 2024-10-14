/******************************************************************************
  * @file           : main.c
  * @brief          : Main configuration file
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

#ifndef CONFIG_H_
#define CONFIG_H_

/**
 * @brief define DEBUG if probes SPI is connected via test point
 * thus MISO is present.
 */
#define DEBUG

/**
 * @brief Define LOG_SEMIHOSTING if you want to enable logging
 * via SEMIHOSTING.
 */
#define LOG_SEMIHOSTING

#ifdef LOG_SEMIHOSTING
  #include <stdio.h>
  #define USE_FULL_ASSERT
  #define log(...) printf(__VA_ARGS__)
#else
  #define log(...) ((void)0U)
#endif

#endif