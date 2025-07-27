/*
 * AS5047P.C
 *
 *  Created on: Jun 27, 2025
 *      Author: munir
 */

#include "AS5047P.h"
#include <string.h>
#include <math.h>


int AS5047P_config(AS5047P_t *encd, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    if (encd == NULL || hspi == NULL || cs_port == NULL || cs_pin == 0) {
        return 0;
    }

    encd->AS5047P_spi = hspi;
    encd->AS5047P_cs_port = cs_port;
    encd->AS5047P_cs_pin = cs_pin;
    
    AS5047P_cs_set(encd);
    
    return 1;
}

int AS5047P_start(AS5047P_t *encd) {
	uint8_t data_tx[2] = {0x3f, 0xff};
	AS5047P_cs_reset(encd);

	if (HAL_SPI_TransmitReceive_DMA(encd->AS5047P_spi, data_tx, encd->spi_rx_buffer, 2) != HAL_OK) {
        return 0;
    }

	return 1;
}


float AS5047P_get_degree(AS5047P_t *encd) {

    AS5047P_cs_set(encd);

    uint16_t encd_data_rx = (encd->spi_rx_buffer[0] << 8) | encd->spi_rx_buffer[1];

    // Cek error flag dengan bit masking
    if (encd_data_rx & (1 << 14)) {
        return encd->angle_filtered;  // Return filtered value if error
    }

    float angle_raw = (float)(encd_data_rx & 0x3FFF) * ANGLE_SCALE_FACTOR;

    float angle_diff = angle_raw - encd->prev_raw_angle;

    angle_diff = fmodf(angle_diff + 180.0f, 360.0f) - 180.0f;

    if (fabsf(angle_diff) > MAX_ANGLE_JUMP_DEG) {
        encd->spike_counter++;

        if (encd->spike_counter < SPIKE_REJECT_COUNT) {
            angle_raw = encd->prev_raw_angle;
        } else {
            encd->spike_counter = 0;
        }
    } else {
        encd->spike_counter = 0;
    }

    encd->prev_raw_angle = angle_raw;

    // Filter IIR dengan optimasi wrap-around
    float filtered_diff = fmodf(angle_raw - encd->angle_filtered + 180.0f, 360.0f) - 180.0f;
    encd->angle_filtered += ANGLE_FILTER_ALPHA * filtered_diff;

    encd->angle_filtered = fmodf(encd->angle_filtered, 360.0f);
    if (encd->angle_filtered < 0.0f) {
        encd->angle_filtered += 360.0f;
    }

    return encd->angle_filtered;
}

float AS5047P_get_rpm(AS5047P_t *encd, uint32_t dt_us) {
    // Handle angle wrap-around (optimized)
    float angle_diff = encd->angle_filtered - encd->prev_angle;
    angle_diff -= 360.0f * floorf((angle_diff + 180.0f) * (1.0f/360.0f));
    encd->prev_angle = encd->angle_filtered;

#if 0
    // Accumulate angle and time for low-RPM precision
    encd->angle_accumulator += angle_diff;
    encd->time_accumulator += dt_us;

    // Only calculate RPM when sufficient data is collected
    if (encd->time_accumulator < MIN_DT_US && encd->filtered_rpm > 1.0f) {
        return encd->filtered_rpm;
    }

    // Calculate RPM (optimized floating point)
    float rpm_instant = (encd->angle_accumulator * MICROS_TO_MINUTES) /
                        (encd->time_accumulator * DEGREES_PER_REV);

    // Reset accumulators
    encd->angle_accumulator = 0.0f;
    encd->time_accumulator = 0;
#else
    // Calculate RPM (optimized floating point)
    float rpm_instant = (angle_diff * MICROS_TO_MINUTES) / (dt_us * DEGREES_PER_REV);
#endif
    // Two-stage spike rejection
    float rpm_delta = rpm_instant - encd->prev_rpm;
    float abs_delta = fabsf(rpm_delta);

    if (abs_delta > MAX_RPM_JUMP) {
        // Gradual rejection with 50% of the delta, capped at MAX_RPM_JUMP
        float limited_delta = copysignf(fminf(abs_delta * 0.5f, MAX_RPM_JUMP), rpm_delta);
        rpm_instant = encd->prev_rpm + limited_delta;
    }

    // IIR Filter with dynamic weighting
    float filtered = encd->filtered_rpm * (1.0f - RPM_FILTER_ALPHA) + rpm_instant * RPM_FILTER_ALPHA;

    // Very low RPM clamping (0.1 RPM resolution)
    if (fabsf(filtered) < 0.1f) {
        filtered = 0.0f;
    }

    // Update state
    encd->prev_rpm = rpm_instant;
    encd->filtered_rpm = filtered;

    return encd->filtered_rpm;
}

float AS5047P_get_actual_degree(AS5047P_t *encd) {
    const float m_current_angle = encd->angle_filtered;
	float angle_dif = (m_current_angle - encd->output_prev_angle);

	if (angle_dif< -180) {
		encd->output_angle_ovf++;
	}
	else if (angle_dif> 180) {
		encd->output_angle_ovf--;
	}
	float out_deg = (m_current_angle + encd->output_angle_ovf * 360.0) * GEAR_RATIO;
    encd->output_angle_filtered = (1.0f - ACTUAL_ANGLE_FILTER_ALPHA) * encd->output_angle_filtered + ACTUAL_ANGLE_FILTER_ALPHA * out_deg;
	encd->output_prev_angle = m_current_angle;

	// return out_deg;
    return encd->output_angle_filtered;
}

