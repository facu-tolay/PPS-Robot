/*
 * Kinematics.c
 *
 *  Created on: 12 mar. 2022
 *      Author: Administrador
 */

#include "kinematics.h"

float desplazamiento_accum[VELOCITY_VECTOR_SIZE] = {0};
float desplazamiento_rot_accum = 0;

/*
 * Obtiene las velocidades angulares de cada rueda segun los parametros (Xr, Yr, theta)
 * */
void calculo_matriz_cinematica_inversa(float *vector_velocidad_lineal, float *vector_velocidad_angular)
{
    float matriz_velocidad_lineal[4][VELOCITY_VECTOR_SIZE] = {0};

    /*matriz_velocidad_lineal[0][0] = (-sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[0][1] = (sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[0][2] = ROBOT_RADIUS/WHEEL_RADIUS;
    matriz_velocidad_lineal[1][0] = (-sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[1][1] = (-sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[1][2] = ROBOT_RADIUS/WHEEL_RADIUS;
    matriz_velocidad_lineal[2][0] = (sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[2][1] = (-sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[2][2] = ROBOT_RADIUS/WHEEL_RADIUS;
    matriz_velocidad_lineal[3][0] = (sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[3][1] = (sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI)); //rpm/m
    matriz_velocidad_lineal[3][2] = ROBOT_RADIUS/WHEEL_RADIUS;*/

    matriz_velocidad_lineal[0][0] = -265.8414; // (-sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[0][1] = 265.8414; // (sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[0][2] = 5.51181; // ROBOT_RADIUS/WHEEL_RADIUS;
    matriz_velocidad_lineal[1][0] = -265.8414; // (-sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[1][1] = -265.8414; // (-sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[1][2] = 5.51181; // ROBOT_RADIUS/WHEEL_RADIUS;
    matriz_velocidad_lineal[2][0] = 265.8414; // (sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[2][1] = -265.8414; // (-sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[2][2] = 5.51181; // ROBOT_RADIUS/WHEEL_RADIUS;
    matriz_velocidad_lineal[3][0] = 265.8414; // (sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI));
    matriz_velocidad_lineal[3][1] = 265.8414; // (sqrt(2)/2)*(30/(WHEEL_RADIUS*M_PI)); //rpm/m
    matriz_velocidad_lineal[3][2] = 5.51181; // ROBOT_RADIUS/WHEEL_RADIUS;

    for (int i=0; i<4; ++i)
    {
        vector_velocidad_angular[i] = 0;
    }

    for (int i=0; i<4; ++i)
    {
        for (int k=0; k<VELOCITY_VECTOR_SIZE; ++k)
        {
            vector_velocidad_angular[i] += matriz_velocidad_lineal[i][k] * vector_velocidad_lineal[k]; //rpm
        }
    }

    return;
}

/*
 * Obtiene las velocidades lineales segun las velocidades angulares de las ruedas (w1, w2, w3, w4)
 * */
void calculo_matriz_cinematica_directa(float *vector_velocidad_angular, float *vector_velocidad_lineal)
{
    float matriz_inversa[VELOCITY_VECTOR_SIZE][4] = {0};

    /*matriz_inversa[0][0] = (-sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[0][1] = (-sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[0][2] = (sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[0][3] = (sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[1][0] = (sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[1][1] = (-sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[1][2] = (-sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[1][3] = (sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[2][0] = (1/(2*ROBOT_RADIUS))*(WHEEL_RADIUS/2);
    matriz_inversa[2][1] = (1/(2*ROBOT_RADIUS))*(WHEEL_RADIUS/2);
    matriz_inversa[2][2] = (1/(2*ROBOT_RADIUS))*(WHEEL_RADIUS/2);
    matriz_inversa[2][3] = (1/(2*ROBOT_RADIUS))*(WHEEL_RADIUS/2);*/

    matriz_inversa[0][0] = -9.4041E-4; // (-sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[0][1] = -9.4041E-4; // (-sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[0][2] = 9.4041E-4; // (sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[0][3] = 9.4041E-4; // (sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[1][0] = 9.4041E-4; // (sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[1][1] = -9.4041E-4; // (-sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[1][2] = -9.4041E-4; // (-sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[1][3] = 9.4041E-4; // (sqrt(2)/2)*(((WHEEL_RADIUS/2)*M_PI)/30);
    matriz_inversa[2][0] = 0.09071; // (1/(2*ROBOT_RADIUS))*(WHEEL_RADIUS/2);
    matriz_inversa[2][1] = 0.09071; // (1/(2*ROBOT_RADIUS))*(WHEEL_RADIUS/2);
    matriz_inversa[2][2] = 0.09071; // (1/(2*ROBOT_RADIUS))*(WHEEL_RADIUS/2);
    matriz_inversa[2][3] = 0.09071; // (1/(2*ROBOT_RADIUS))*(WHEEL_RADIUS/2);

    for (int i=0; i<VELOCITY_VECTOR_SIZE; ++i)
    {
        vector_velocidad_lineal[i] = 0;
    }

    for (int i=0; i<VELOCITY_VECTOR_SIZE; ++i)
    {
        for (int k=0; k<4; ++k)
        {
            vector_velocidad_lineal[i] += matriz_inversa[i][k] * vector_velocidad_angular[k];
        }
    }

    return;
}

/*
 * Calcula el error entre la velocidad lineal deseada (Xr, Yr, theta) y la real.
   Si da un numero positivo significa que debo bajar la velocidad en esa componente.
   Si da un numero negativo significa que debo subir la velocidad en esa componente.
 * */
void calculo_error_velocidades_lineales(float *velocidad_lineal, float *velocidad_lineal_real, float *delta_velocidad_lineal)
{
    for(int i=0; i<VELOCITY_VECTOR_SIZE; i++)
    {
        delta_velocidad_lineal[i] = velocidad_lineal_real[i] - velocidad_lineal[i];
    }
}

void calculo_distancia_recorrida_acumulada(float *velocidad_lineal_real, float delta_t, float *distancia_accum, float *delta_distance)
{
    for (int i=0; i<VELOCITY_VECTOR_SIZE; i++)
    {
        delta_distance[i] = fabs(velocidad_lineal_real[i]) * delta_t;
        desplazamiento_accum[i] = desplazamiento_accum[i] + delta_distance[i];
        distancia_accum[i] = desplazamiento_accum[i];
    }
    desplazamiento_rot_accum = desplazamiento_rot_accum + velocidad_lineal_real[2] * delta_t;
}

void calculo_compensacion_linea_magnetica(uint8_t is_velocidad_rotacional_zero, float velocidades_lineales_reales[VELOCITY_VECTOR_SIZE], int line_follower_count[HALL_SENSOR_COUNT])
{
    for(int i=0; i<HALL_SENSOR_COUNT; i++)
    {
        if ((i == 0 || i == 2) && line_follower_count[i] != 0)
        {
            if(!is_velocidad_rotacional_zero)
            {
                if(i==0)
                {
                    // velocidades_lineales_reales[2] = velocidades_lineales_reales[2] + line_follower_count[i]*12.0;
                    desplazamiento_rot_accum = desplazamiento_rot_accum + line_follower_count[i]*2.5;
                }
                else
                {
                    // velocidades_lineales_reales[2] = velocidades_lineales_reales[2] - line_follower_count[i]*12.0;
                    desplazamiento_rot_accum = desplazamiento_rot_accum - line_follower_count[i]*2.5;
                }
            }
            else
            {
                if(i==0)
                {
                    // velocidades_lineales_reales[2] = velocidades_lineales_reales[2] + line_follower_count[i]*9.2;
                    desplazamiento_rot_accum = desplazamiento_rot_accum + line_follower_count[i]*2.5;
                }
                else
                {
                    // velocidades_lineales_reales[2] = velocidades_lineales_reales[2] - line_follower_count[i]*9.2;
                    desplazamiento_rot_accum = desplazamiento_rot_accum - line_follower_count[i]*2.5;
                }
            }
        }
    }
}

void calculo_rompensacion_rotacional(float velocidades_lineales_reales[VELOCITY_VECTOR_SIZE])
{
    // FIXME esto quiza necesitaria un if para saber si esta yendo en linea recta o no ( velocidad_lineal[2] == v_rotacional == 0 )
    // creo que no funcionaria bien para los casos que tiene que rotar, dado que esto lo que hace es tratar siempre de llevar el desplazamiento rotazional a cero
    if(desplazamiento_rot_accum != 0)
    {
        velocidades_lineales_reales[2] = velocidades_lineales_reales[2] + (desplazamiento_rot_accum * 1.95); // se compensa la rotacion en base a cuanto desplazamiento rotacional se detecte
    }
}

void reset_accum()
{
    desplazamiento_rot_accum = 0;
    for (int i=0; i<VELOCITY_VECTOR_SIZE; i++)
    {
        desplazamiento_accum[i] = 0;
    }
}

uint8_t robot_in_radius_of_setpoint(float desired_setpoint, float *current_position)
{
    float delta_x = fabs(current_position[0] - desired_setpoint);
    float delta_y = fabs(current_position[1] - desired_setpoint);
    return (delta_x <= MIN_DESTINATION_RADIUS || current_position[0] >= desired_setpoint || delta_y <= MIN_DESTINATION_RADIUS || current_position[1] >= desired_setpoint);
}