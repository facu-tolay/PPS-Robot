/*
 * Kinematics.c
 *
 *  Created on: 12 mar. 2022
 *      Author: Administrador
 */

#include "kinematics.h"

/*
 * Obtiene las velocidades angulares de cada rueda segun los parametros (Xr, Yr, theta)
 * */
void calculo_matriz_cinematica_inversa(float *vector_velocidad_lineal, float *vector_velocidad_angular)
{
    float matriz_velocidad_lineal[4][3] = {0};

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
        for (int k=0; k<3; ++k)
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
    float matriz_inversa[3][4] = {0};

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

    for (int i=0; i<3; ++i)
    {
        vector_velocidad_lineal[i] = 0;
    }

    for (int i=0; i<3; ++i)
    {
        for (int k=0; k<4; ++k)
        {
            vector_velocidad_lineal[i] += matriz_inversa[i][k] * vector_velocidad_angular[k];
        }
    }

    return;
}

/*
 * Calcula el error entre la velocidad lineal deseada (Xr, Yr, theta) y la real
 * */
void calculo_error_velocidades_lineales(float *velocidad_lineal, float *velocidad_lineal_real, float *delta_velocidad_lineal)
{
    for(int i=0; i<3; i++)
    {
        delta_velocidad_lineal[i] = velocidad_lineal_real[i] - velocidad_lineal[i];
    }
}

void seteo_parametros_vectores(float *vector_velocidad_lineal, float *vector_velocidad_angular, movement_vector_t *movement_vector)
{
    vector_velocidad_lineal[0] = movement_vector->velocidad_lineal_x;
    vector_velocidad_lineal[1] = movement_vector->velocidad_lineal_y;
    vector_velocidad_lineal[2] = movement_vector->velocidad_angular;

    calculo_matriz_cinematica_inversa(vector_velocidad_lineal, vector_velocidad_angular);
}

void seteo_datos_motor_task(float velocidad_angular, float setpoint, motor_movement_vector_t *motor, QueueHandle_t queue)
{
    motor->rpm = velocidad_angular;
    motor->setpoint = setpoint;

    xQueueSend(queue, motor, 0);
}