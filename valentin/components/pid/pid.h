/*
 * PID.h
 *
 *  Created on: 12 mar. 2022
 *      Author: Administrador
 */

#ifndef COMPONENTS_PID_PID_H_
#define COMPONENTS_PID_PID_H_

#include <stdio.h>
#include "../../main/constants.h"

// Proportional, Integral and Derivative terms
#define _Kp (float) 15
#define _Ki (float) 7.3
#define _Kd (float) 0.23
#define _dt (float)TIMER_INTERVAL_RPM_MEASURE

/*
 * For PID computing
 * */
typedef struct {
    float rpm_actual;
    float rpm_destino;
    float _integral;
    float _pre_error;
    signed int output;
} PID_params_t;

void PID_Compute(PID_params_t *params_in);
void reset_pid_state(PID_params_t *pid_params);

#endif /* COMPONENTS_PID_PID_H_ */
