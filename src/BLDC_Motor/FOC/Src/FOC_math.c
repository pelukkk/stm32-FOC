/*
 * FOC_math.c
 *
 *  Created on: Jul 12, 2025
 *      Author: munir
 */

#include "FOC_math.h"
#include "math.h"

float sin_lut[LUT_SIZE];
float cos_lut[LUT_SIZE];

void init_trig_lut(void) {
    for (int i = 0; i < LUT_SIZE; ++i) {
        float angle = i * LUT_STEP;
        sin_lut[i] = sinf(angle);
        cos_lut[i] = cosf(angle);
    }
}

void norm_angle_rad(float *theta) {
    while (*theta < 0) *theta += TWO_PI;
    while (*theta >= TWO_PI) *theta -= TWO_PI;
}

float fast_sin(float theta) {
	norm_angle_rad(&theta);
    float index_f = theta / LUT_STEP;
    int index = (int)index_f;
    float frac = index_f - index;

    int next_index = (index + 1) % LUT_SIZE;

    return sin_lut[index] * (1.0f - frac) + sin_lut[next_index] * frac;
}

float fast_cos(float theta) {
	norm_angle_rad(&theta);
    float index_f = theta / LUT_STEP;
    int index = (int)index_f;
    float frac = index_f - index;

    int next_index = (index + 1) % LUT_SIZE;

    return cos_lut[index] * (1.0f - frac) + cos_lut[next_index] * frac;
}

void pre_calc_sin_cos(float theta, float *sin_theta, float *cos_theta) {
    *sin_theta = fast_sin(theta);
    *cos_theta = fast_cos(theta);
}

// Fast combined Clarke + Park Transform
void clarke_park_transform(float ia, float ib, float sin_theta, float cos_theta, float *id, float *iq) {
    // Clarke transform
    float i_alpha = ia;
    float i_beta  = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;

    // Park transform
    *id = i_alpha * cos_theta + i_beta * sin_theta;
    *iq = i_beta * cos_theta - i_alpha * sin_theta;
}

// Inverse Park Transform
void inverse_park_transform(float vd, float vq, float sin_theta, float cos_theta, float *valpha, float *vbeta) {
    *valpha = vd * cos_theta - vq * sin_theta;
    *vbeta  = vd * sin_theta + vq * cos_theta;
}

// Inverse Clarke Transform
void inverse_clarke_transform(float valpha, float vbeta, float *va, float *vb, float *vc) {
    *va = valpha;
    *vb = -0.5f * valpha + SQRT3_BY_TWO * vbeta;   // cos(120°), sin(120°)
    *vc = -0.5f * valpha - SQRT3_BY_TWO * vbeta;
}

/**
 * @brief Space Vector PWM Modulation
 * @param valpha Alpha component of voltage vector
 * @param vbeta Beta component of voltage vector
 * @param vbus DC bus voltage
 * @param pwm_period Full PWM period value
 * @param pwm_u Output duty cycle for phase U (0 to pwm_period)
 * @param pwm_v Output duty cycle for phase V
 * @param pwm_w Output duty cycle for phase W
 */
void svpwm(float valpha, float vbeta, float vbus, uint32_t pwm_period,
          uint32_t *pwm_u, uint32_t *pwm_v, uint32_t *pwm_w)
{
    // 1. Normalize voltages by vbus
    float alpha = valpha / vbus;
    float beta = vbeta / vbus;

    // 2. Sector determination
    uint8_t sector;
    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            sector = (ONE_BY_SQRT3 * beta > alpha) ? 2 : 1;  // 1/sqrt(3) ≈ 0.577
        } else {
            sector = (-ONE_BY_SQRT3 * beta > alpha) ? 3 : 2;
        }
    } else {
        if (alpha >= 0.0f) {
            sector = (-ONE_BY_SQRT3 * beta > alpha) ? 5 : 6;
        } else {
            sector = (ONE_BY_SQRT3 * beta > alpha) ? 4 : 5;
        }
    }

    // 3. Calculate active vector times
    int32_t t1, t2;
    switch(sector) {
        case 1:
            t1 = (int32_t)((alpha - ONE_BY_SQRT3 * beta) * pwm_period);
            t2 = (int32_t)(TWO_BY_SQRT3 * beta * pwm_period);
            *pwm_u = (pwm_period + t1 + t2) / 2;
            *pwm_v = *pwm_u - t1;
            *pwm_w = *pwm_v - t2;
            break;

        case 2:
            t1 = (int32_t)((alpha + ONE_BY_SQRT3 * beta) * pwm_period);
            t2 = (int32_t)((-alpha + ONE_BY_SQRT3 * beta) * pwm_period);
            *pwm_v = (pwm_period + t1 + t2) / 2;
            *pwm_u = *pwm_v - t2;
            *pwm_w = *pwm_u - t1;
            break;

        case 3:
            t1 = (int32_t)(TWO_BY_SQRT3 * beta * pwm_period);
            t2 = (int32_t)((-alpha - ONE_BY_SQRT3 * beta) * pwm_period);
            *pwm_v = (pwm_period + t1 + t2) / 2;
            *pwm_w = *pwm_v - t1;
            *pwm_u = *pwm_w - t2;
            break;

        case 4:
            t1 = (int32_t)((-alpha + ONE_BY_SQRT3 * beta) * pwm_period);
            t2 = (int32_t)(-TWO_BY_SQRT3 * beta * pwm_period);
            *pwm_w = (pwm_period + t1 + t2) / 2;
            *pwm_v = *pwm_w - t2;
            *pwm_u = *pwm_v - t1;
            break;

        case 5:
            t1 = (int32_t)((-alpha - ONE_BY_SQRT3 * beta) * pwm_period);
            t2 = (int32_t)((alpha - ONE_BY_SQRT3 * beta) * pwm_period);
            *pwm_w = (pwm_period + t1 + t2) / 2;
            *pwm_u = *pwm_w - t1;
            *pwm_v = *pwm_u - t2;
            break;

        case 6:
            t1 = (int32_t)(-TWO_BY_SQRT3 * beta * pwm_period);
            t2 = (int32_t)((alpha + ONE_BY_SQRT3 * beta) * pwm_period);
            *pwm_u = (pwm_period + t1 + t2) / 2;
            *pwm_w = *pwm_u - t2;
            *pwm_v = *pwm_w - t1;
            break;
    }

    // 4. Clamp outputs to valid range
    *pwm_u = (*pwm_u > pwm_period) ? pwm_period : *pwm_u;
    *pwm_v = (*pwm_v > pwm_period) ? pwm_period : *pwm_v;
    *pwm_w = (*pwm_w > pwm_period) ? pwm_period : *pwm_w;
}

