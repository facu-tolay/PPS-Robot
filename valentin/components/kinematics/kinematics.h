/*
 * Kinematics.h
 *
 *  Created on: 12 mar. 2022
 *      Author: Administrador
 */

#ifndef COMPONENTS_KINEMATICS_KINEMATICS_H_
#define COMPONENTS_KINEMATICS_KINEMATICS_H_

#include <stdio.h>

void calculo_matriz_cinematica_inversa(float *vector_velocidad_lineal, float *vector_velocidad_angular);
void calculo_matriz_cinematica_directa(float *vector_velocidad_angular, float *vector_velocidad_lineal);
void calculo_error_velocidades_lineales(float *velocidad_lineal, float *velocidad_lineal_real, float *delta_velocidad_lineal);

#endif /* COMPONENTS_KINEMATICS_KINEMATICS_H_ */
