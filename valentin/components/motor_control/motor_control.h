/*
 * MotorControl.h
 *
 *  Created on: 12 mar. 2022
 *      Author: Administrador
 */

#ifndef COMPONENTS_MOTORCONTROL_MOTORCONTROL_H_
#define COMPONENTS_MOTORCONTROL_MOTORCONTROL_H_

#include "driver/ledc.h"

#define CANT_LEDC_CHANNELS  8

#define MOT_A_SEL       0
#define MOT_B_SEL       1
#define MOT_C_SEL       2
#define MOT_D_SEL       3

#define MOT_A_1_GPIO    18
#define MOT_A_2_GPIO    5
#define MOT_B_1_GPIO    19
#define MOT_B_2_GPIO    21
#define MOT_C_1_GPIO    32
#define MOT_C_2_GPIO    33
#define MOT_D_1_GPIO    25
#define MOT_D_2_GPIO    26

void motorSetSpeed(uint8_t selection, signed int pwm_value);
void motorStop(uint8_t selection);
void motorInitialize();

#endif /* COMPONENTS_MOTORCONTROL_MOTORCONTROL_H_ */
