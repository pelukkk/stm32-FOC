/*
 * DRV8302_config.h
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#ifndef DRV8302_DRIVER_INC_DRV8302_CONFIG_H_
#define DRV8302_DRIVER_INC_DRV8302_CONFIG_H_

#include "stm32f4xx_hal.h"

/* Defined External Pin */

#define USE_MPWM_EXTPIN		1

#define USE_MOC_EXTPIN		1

#define USE_GAIN_EXTPIN		1

#define USE_DC_CAL_EXTPIN	1

#define USE_OC_ADJ_EXTPIN	0

#define USE_OCTW_EXTPIN		1

#define USE_FAULT_EXTPIN	1

#define USE_EN_GATE_EXTPIN	1

#define ADC_RES	4096
#define AVDD	3.3f

// = AVDD / ADC_RES
#define ADC_2_VOLT	0.0008056640625f

#define ADC_2_POWER_VOLT	0.01651611328125f

#define CURRENT_FILTER_ALPHA 0.71539f //0.466512f //0.71539f 

#endif /* DRV8302_DRIVER_INC_DRV8302_CONFIG_H_ */
