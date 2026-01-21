/*
 * freq_detect.h
 *
 *  Created on: Oct 29, 2025
 *      Author: IBRAHIM MUADH & VARUN SHAH
 */

#ifndef FREQ_DETECT_H
#define FREQ_DETECT_H

#include "stm32f4xx_hal.h"

void freq_init(void);
float freq_estimate(float sample);
float flow_calculate(float freq_hz, float temperature_c);

#endif
