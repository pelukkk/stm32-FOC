/*
 * FOC_utils.c
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#include "FOC_utils.h"
#include "flash.h"
#include "controller_app.h"
#include <math.h>
#include <string.h>

_Bool foc_ready = 0;

float Vd_buff[MAX_I_SAMPLE];
float Vq_buff[MAX_I_SAMPLE];
float Id_buff[MAX_I_SAMPLE];
float Iq_buff[MAX_I_SAMPLE];

float error_temp[ERROR_LUT_SIZE] = {0};

/**
 * @brief Initialize PWM output pointers for FOC controller
 * @param hfoc Pointer to FOC controller structure
 * @param pwm_a Pointer to PWM channel A register (e.g., &TIM1->CCR1)
 * @param pwm_b Pointer to PWM channel B register (e.g., &TIM1->CCR2)
 * @param pwm_c Pointer to PWM channel C register (e.g., &TIM1->CCR3)
 * @param pwm_res PWM resolution
 */
void foc_pwm_init(foc_t *hfoc, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c,
		uint32_t pwm_res) {
    // Validate pointers
    if(hfoc == NULL || pwm_a == NULL || pwm_b == NULL || pwm_c == NULL) {
        // Error handling (could be an assertion or error code)
        return;
    }

    hfoc->pwm_a = pwm_a;
    hfoc->pwm_b = pwm_b;
    hfoc->pwm_c = pwm_c;
    hfoc->pwm_res = pwm_res;

    // Optional: Initialize PWM values to 0
    *hfoc->pwm_a = 0;
    *hfoc->pwm_b = 0;
    *hfoc->pwm_c = 0;
}

void foc_motor_init(foc_t *hfoc, uint8_t pole_pairs, float kv) {
	if (hfoc == NULL || pole_pairs == 0 || kv <= 0) {
		return;
	}

	hfoc->pole_pairs = pole_pairs;
	hfoc->kv = kv;
}

void foc_sensor_init(foc_t *hfoc, float m_rad_offset, dir_mode_t sensor_dir) {
	if (hfoc == NULL) return;

	hfoc->m_angle_offset = m_rad_offset;
	hfoc->sensor_dir = sensor_dir;
}

void foc_gear_reducer_init(foc_t *hfoc, float ratio) {
	if (hfoc == NULL) return;

	hfoc->gear_ratio = ratio;
}

void foc_set_limit_current(foc_t *hfoc, float i_limit) {
	if (hfoc == NULL) return;

	hfoc->max_current = i_limit;
}

void foc_current_control_update(foc_t *hfoc) {
	if (hfoc == NULL || hfoc->control_mode == AUDIO_MODE) {
		hfoc->id_ctrl.integral = 0.0f;
		hfoc->id_ctrl.last_error = 0.0f;
		hfoc->iq_ctrl.integral = 0.0f;
		hfoc->iq_ctrl.last_error = 0.0f;
		return;
	}

	float id_ref = hfoc->id_ref;
	float iq_ref = hfoc->iq_ref;

    // Hard limit references
    id_ref = CONSTRAIN(id_ref, -hfoc->max_current, hfoc->max_current);
    iq_ref = CONSTRAIN(iq_ref, -hfoc->max_current, hfoc->max_current);

    // pre calculate sin & cos
    float sin_theta, cos_theta;
    pre_calc_sin_cos(hfoc->e_angle_rad_comp, &sin_theta, &cos_theta);

    // Get measured currents
    clarke_park_transform(hfoc->ia, hfoc->ib, sin_theta, cos_theta, &hfoc->id, &hfoc->iq);

    // LPF id & iq
    const float alpha_i_filt = 0.5f;
    hfoc->id_filtered = (1.0f - alpha_i_filt) * hfoc->id_filtered + alpha_i_filt * hfoc->id;
    hfoc->iq_filtered = (1.0f - alpha_i_filt) * hfoc->iq_filtered + alpha_i_filt * hfoc->iq;

    // Continue normal FOC
    float vd_ref = pi_control(&hfoc->id_ctrl, id_ref - hfoc->id);
    float vq_ref = pi_control(&hfoc->iq_ctrl, iq_ref - hfoc->iq);

    float valpha, vbeta;
    uint32_t da, db, dc;
    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &valpha, &vbeta);
    svpwm(valpha, vbeta, hfoc->v_bus, hfoc->pwm_res, &da, &db, &dc);

    // pwm limit
    *(hfoc->pwm_a) = CONSTRAIN(da, 0, hfoc->pwm_res);
    *(hfoc->pwm_b) = CONSTRAIN(db, 0, hfoc->pwm_res);
    *(hfoc->pwm_c) = CONSTRAIN(dc, 0, hfoc->pwm_res);
}

void foc_speed_control_update(foc_t *hfoc, float rpm_reference) {
	if (hfoc == NULL || (hfoc->control_mode != SPEED_CONTROL_MODE && hfoc->control_mode != POSITION_CONTROL_MODE)) {
		hfoc->speed_ctrl.integral = 0.0f;
		hfoc->speed_ctrl.last_error = 0.0f;
		return;
	}

    hfoc->id_ref = 0.0f;
    hfoc->iq_ref = -pi_control(&hfoc->speed_ctrl, rpm_reference - hfoc->actual_rpm);
}

void foc_position_control_update(foc_t *hfoc, float deg_reference) {
	if (hfoc == NULL || hfoc->control_mode != POSITION_CONTROL_MODE) {
		hfoc->pos_ctrl.integral = 0.0f;
		hfoc->pos_ctrl.last_error = 0.0f;
		return;
	}
#if 0
    if (hfoc->loop_count >= 5) {
        hfoc->loop_count = 0;
		// extern float angle_deg;
        // hfoc->actual_angle = get_actual_degree(angle_deg);
        hfoc->rpm_ref = pd_control(&hfoc->pos_ctrl, deg_reference - hfoc->actual_angle);
    }
    hfoc->loop_count++;

    foc_speed_control_update(hfoc, hfoc->rpm_ref);
#else
    hfoc->id_ref = 0.0f;
    hfoc->iq_ref = -pd_control(&hfoc->pos_ctrl, deg_reference - hfoc->actual_angle);
#endif
}

extern _Bool sensor_is_calibrated;

void foc_calc_electric_angle(foc_t *hfoc, float m_rad) {
    // Check for NULL pointer and invalid parameters
    if (hfoc == NULL || hfoc->pole_pairs <= 0) {
        return;
    }

    // Normalize mechanical angle
    hfoc->m_angle_rad = m_rad - hfoc->m_angle_offset;
    norm_angle_rad(&hfoc->m_angle_rad);

    // Calculate raw electric angle
    float e_rad = hfoc->m_angle_rad * hfoc->pole_pairs;
    
    // Handle sensor direction
    if (hfoc->sensor_dir == REVERSE_DIR) {
        e_rad = TWO_PI - e_rad;
    }

    hfoc->e_angle_rad = e_rad;

    // Calculate LUT index with wrap-around
    float lut_idx_f = (hfoc->m_angle_rad / TWO_PI) * ERROR_LUT_SIZE;
    lut_idx_f = fmodf(lut_idx_f, ERROR_LUT_SIZE);
    if (lut_idx_f < 0) {
        lut_idx_f += ERROR_LUT_SIZE;
    }

    // Get neighboring indices with wrap-around
    int idx0 = (int)lut_idx_f % ERROR_LUT_SIZE;
    int idx1 = (idx0 + 1) % ERROR_LUT_SIZE;
    float frac = lut_idx_f - (float)idx0;

    // Linear interpolation
    float encoder_error = m_config.encd_error_comp[idx0] * (1.0f - frac) + m_config.encd_error_comp[idx1] * frac;
    e_rad += encoder_error;
    
    // Normalize final electric angle
    norm_angle_rad(&e_rad);

    hfoc->e_angle_rad_comp = e_rad;
}

void foc_cal_encoder_misalignment(foc_t *hfoc) {
  open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, 0.0f);
  HAL_Delay(500);
  float rad_offset = 0.0f;
  for (int i = 0; i < CAL_ITERATION; i++) {
    rad_offset += DEG_TO_RAD(*hfoc->angle_filtered);
    HAL_Delay(1);
  }
  open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f);
  rad_offset = rad_offset / (float)CAL_ITERATION;
  hfoc->m_angle_offset = rad_offset;
  m_config.encd_offset = rad_offset;
}

void foc_cal_encoder(foc_t *hfoc) {
  memset(error_temp, 0, sizeof(error_temp));

  foc_cal_encoder_misalignment(hfoc);

  for (int i = 0; i < ERROR_LUT_SIZE; i++) {
    float mech_deg = (float)i * (360.0f / (float)ERROR_LUT_SIZE);
    float elec_rad = DEG_TO_RAD(mech_deg * hfoc->pole_pairs);
    open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, elec_rad);
    HAL_Delay(5);

    float mech_rad = hfoc->m_angle_rad;
    float raw_delta = elec_rad - hfoc->e_angle_rad;
    float delta = elec_rad - hfoc->e_angle_rad_comp;
    
    raw_delta -= TWO_PI * floorf((raw_delta + PI) / TWO_PI);
    delta -= TWO_PI * floorf((delta + PI) / TWO_PI);
    
    float lut_pos = (mech_rad / TWO_PI) * ERROR_LUT_SIZE;
    int index = (int)(lut_pos);

    while (index < 0) {
      index += ERROR_LUT_SIZE;
    }
    index %= ERROR_LUT_SIZE;

    error_temp[index] = raw_delta;
    
    // debug
    float buffer_val[2];
    // buffer_val[0] = raw_delta;
    // buffer_val[1] = delta;
    buffer_val[0] = delta;
    send_data_float(buffer_val, 1);
  }
  
  for (int i = 0; i < ERROR_LUT_SIZE; i++) {
    if (error_temp[i] == 0) {
      int last_i = i - 1;
      int next_i = i + 1;
      if (last_i < 0) last_i += ERROR_LUT_SIZE;
      if (next_i > ERROR_LUT_SIZE) next_i -= ERROR_LUT_SIZE;
      error_temp[i] = (error_temp[last_i] + error_temp[next_i]) / 2.0f;
    }
  }

  memcpy(m_config.encd_error_comp, error_temp, sizeof(error_temp));

  open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f);
}

void foc_set_torque_control_bandwidth(foc_t *hfoc, float bandwidth) {
    hfoc->I_ctrl_bandwidth = bandwidth;
}

void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad) {
    float valpha, vbeta;
    uint32_t da, db, dc;
    const uint32_t pwm_res = hfoc->pwm_res;

    float sin_theta, cos_theta;
    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &valpha, &vbeta);
    svpwm(valpha, vbeta, hfoc->v_bus, pwm_res, &da, &db, &dc);

    *(hfoc->pwm_a) = CONSTRAIN(da, 0, pwm_res);
    *(hfoc->pwm_b) = CONSTRAIN(db, 0, pwm_res);
    *(hfoc->pwm_c) = CONSTRAIN(dc, 0, pwm_res);
}

void meas_inj_dq_process(foc_t *hfoc, float ts) {
  if (hfoc->meas_inj_start_flag) {
    float angle = hfoc->meas_inj_omega * hfoc->meas_inj_n * ts;
    float v_inj = hfoc->meas_inj_amp * fast_sin(angle);
    float vd, vq;
    if (hfoc->meas_inj_target == VD) {
        vd = v_inj;
        vq = 0.0f;
    }
    else if (hfoc->meas_inj_target == VQ) {
        vd = 0.0f;
        vq = v_inj;
    }
    else return;

    float theta_e = hfoc->e_angle_rad_comp;

    open_loop_voltage_control(hfoc, vd, vq, theta_e);

    float sin_theta, cos_theta;
    float id, iq;
    pre_calc_sin_cos(theta_e, &sin_theta, &cos_theta);
    clarke_park_transform(hfoc->ia, hfoc->ib, sin_theta, cos_theta, &id, &iq);

    if (hfoc->meas_inj_target == VD) {
        Vd_buff[hfoc->meas_inj_n] = vd;
        Id_buff[hfoc->meas_inj_n] = id;
    }
    else if (hfoc->meas_inj_target == VQ) {
        Vq_buff[hfoc->meas_inj_n] = vq;
        Iq_buff[hfoc->meas_inj_n] = iq;
    }

    hfoc->meas_inj_n++;
    if (hfoc->meas_inj_n >= MAX_I_SAMPLE) {
        hfoc->meas_inj_n = 0;
        hfoc->meas_inj_start_flag = 0;
        open_loop_voltage_control(hfoc, 0, 0, 0);
    }
  }
}

void estimate_motor_param_dq(foc_t *hfoc, float ts) {
    float Vc_d = 0, Vs_d = 0, Ic_d = 0, Is_d = 0;
    float Vc_q = 0, Vs_q = 0, Ic_q = 0, Is_q = 0;
    float mean_vd = 0, mean_vq = 0, mean_id = 0, mean_iq = 0;

    // --- Hilangkan DC offset ---
    for (int i = 0; i < MAX_I_SAMPLE; i++) {
        mean_vd += Vd_buff[i];
        mean_vq += Vq_buff[i];
        mean_id += Id_buff[i];
        mean_iq += Iq_buff[i];
    }
    mean_vd /= MAX_I_SAMPLE;
    mean_vq /= MAX_I_SAMPLE;
    mean_id /= MAX_I_SAMPLE;
    mean_iq /= MAX_I_SAMPLE;

    // --- Hitung komponen DFT fundamental ---
    for (int i = 0; i < MAX_I_SAMPLE; i++) {
        float angle = hfoc->meas_inj_omega * i * ts;

        float vd = Vd_buff[i] - mean_vd;
        float vq = Vq_buff[i] - mean_vq;
        float id = Id_buff[i] - mean_id;
        float iq = Iq_buff[i] - mean_iq;

        // korelasi cos dan sin
        Vc_d += vd * fast_cos(angle);
        Vs_d += vd * fast_sin(angle);
        Ic_d += id * fast_cos(angle);
        Is_d += id * fast_sin(angle);

        Vc_q += vq * fast_cos(angle);
        Vs_q += vq * fast_sin(angle);
        Ic_q += iq * fast_cos(angle);
        Is_q += iq * fast_sin(angle);
    }

    float norm = 2.0f / MAX_I_SAMPLE;
    Vc_d *= norm; Vs_d *= norm;
    Ic_d *= norm; Is_d *= norm;
    Vc_q *= norm; Vs_q *= norm;
    Ic_q *= norm; Is_q *= norm;

    // --- Amplitudo tegangan dan arus ---
    float Vd_mag = sqrtf(Vc_d * Vc_d + Vs_d * Vs_d);
    float Id_mag = sqrtf(Ic_d * Ic_d + Is_d * Is_d);
    float Vq_mag = sqrtf(Vc_q * Vc_q + Vs_q * Vs_q);
    float Iq_mag = sqrtf(Ic_q * Ic_q + Is_q * Is_q);

    // --- Sudut fasa (phi = arctan(Vs/Vc) - arctan(Is/Ic)) ---
    float phi_d = atan2f(Vs_d, Vc_d) - atan2f(Is_d, Ic_d);
    float phi_q = atan2f(Vs_q, Vc_q) - atan2f(Is_q, Ic_q);

    // --- Impedansi dan parameter ---
    float Zd_mag = Vd_mag / Id_mag;
    float Zq_mag = Vq_mag / Iq_mag;

    float Rs_d = Zd_mag * cosf(phi_d);
    float Rs_q = Zq_mag * cosf(phi_q);
    float Ld_est = (Zd_mag * sinf(phi_d)) / hfoc->meas_inj_omega;
    float Lq_est = (Zq_mag * sinf(phi_q)) / hfoc->meas_inj_omega;

    hfoc->Rs = (Rs_d + Rs_q) * 0.5;
    hfoc->Ld = fabs(Ld_est);
    hfoc->Lq = fabs(Lq_est);
}