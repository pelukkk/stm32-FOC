#include "pll.h"
#include "FOC_math.h"

void pll_init(pll_t* pll, float kp, float ki, float k_h, float omega_max) {
    pll->kp = kp;
    pll->ki = ki;
    pll->k_h = k_h;
    pll->integral = 0.0f;
    pll->omega_est = 0.0f;
    pll->theta_est = 0.0f;
    pll->omega_max = omega_max;
}

float pll_update(pll_t* pll, float i_alpha_t, float i_beta_t, float Ts) {
    // Phase Detector
    float error = i_beta_t * fast_cos(pll->theta_est) - i_alpha_t * fast_sin(pll->theta_est);
    
    float scaled_error = pll->k_h * error;
    
    // PI Controller
    pll->integral += scaled_error * Ts;
    
    // Anti-windup
    float integral_max = pll->omega_max / pll->ki;
    if (pll->integral > integral_max) pll->integral = integral_max;
    if (pll->integral < -integral_max) pll->integral = -integral_max;
    
    float omega_e = pll->kp * scaled_error + pll->ki * pll->integral;
    
    // Speed limiting
    if (omega_e > pll->omega_max) omega_e = pll->omega_max;
    if (omega_e < -pll->omega_max) omega_e = -pll->omega_max;

    // omega filter
    const float alpha = 0.05f;
    pll->omega_est = (1.0f - alpha) * pll->omega_est + alpha * omega_e;

    // Integrator for position
    float theta_diff = omega_e * Ts;
    pll->theta_est += theta_diff;
    
    // Wrap angle
    norm_angle_rad(&pll->theta_est);
    
    return pll->theta_est;
}
