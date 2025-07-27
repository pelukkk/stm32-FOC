/*
 * FOC_utils.h
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#ifndef FOC_INC_FOC_UTILS_H_
#define FOC_INC_FOC_UTILS_H_

#include <stdint.h>
#include "FOC_math.h"
#include "pid_utils.h"

#define MAG_CAL_RES (1024*2)
#define MAG_CAL_STEP ((TWO_PI * POLE_PAIR) / (float)MAG_CAL_RES)

#define VD_CAL 0.5f
#define VQ_CAL 0.0f

#define is_foc_ready() (foc_ready)
#define foc_reset_flag() (foc_ready = 0)
#define foc_set_flag() (foc_ready = 1)

extern _Bool foc_ready;

typedef enum {
	TORQUE_CONTROL_MODE,
	SPEED_CONTROL_MODE,
	POSITION_CONTROL_MODE,
	AUDIO_MODE,
	CALIBRATION_MODE,
}motor_mode_t;

typedef enum {
	NORMAL_DIR, REVERSE_DIR
}dir_mode_t;


typedef struct {
	uint8_t pole_pairs;
	float kv;
	float La, Lb, Lc;
	float Ra, Rb, Rc;
	float max_current;

	float m_angle_rad; // mechanical angle
	float e_angle_rad; // electrical angle
	float m_angle_offset;

	float vd, vq;
	float id, iq;
	float va, vb, vc;
	float ia, ib, ic;
	float v_bus;
	float i_bus;

	float actual_rpm;
	float actual_angle;

	float id_ref, iq_ref;
	float rpm_ref;

    uint8_t loop_count;

	volatile uint32_t *pwm_a;
	volatile uint32_t *pwm_b;
	volatile uint32_t *pwm_c;
	uint32_t pwm_res;

	PID_Controller_t id_ctrl, iq_ctrl;
	PID_Controller_t speed_ctrl;
	PID_Controller_t pos_ctrl;

	motor_mode_t control_mode;

	float gear_ratio;
	dir_mode_t sensor_dir;
}foc_t;

void foc_pwm_init(foc_t *hfoc, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c,
		uint32_t pwm_res);
void foc_motor_init(foc_t *hfoc, uint8_t pole_pairs, float kv);
void foc_sensor_init(foc_t *hfoc, float m_rad_offset, dir_mode_t sensor_dir);
void foc_gear_reducer_init(foc_t *hfoc, float ratio);
void foc_set_limit_current(foc_t *hfoc, float i_limit);
void foc_current_control_update(foc_t *hfoc);
void foc_speed_control_update(foc_t *hfoc, float rpm_reference);
void foc_position_control_update(foc_t *hfoc, float deg_reference);
void foc_calc_electric_angle(foc_t *hfoc, float m_rad);
void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad);

#endif /* FOC_INC_FOC_UTILS_H_ */
