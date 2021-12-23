// Online C compiler to run C program online
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define ROBOT_RADIUS    (float)0.14
#define WHEEL_RADIUS    (float)0.0508

void calculo_matriz(float *vector_velocidad_lineal, float *vector_velocidad_angular);
void calculo_matriz_inversa(float *vector_velocidad_angular, float *vector_velocidad_lineal);
void error_calc(float *velocidad_lineal, float *velocidad_lineal_real, float *delta_velocidad_lineal);
void make_variations(float *input, float *output, int size);

int main()
{
    float velocidad_lineal[3] = {60, 40, 10};
    float velocidad_lineal_real[3] = {0};
    float velocidad_angular[4] = {0};
    float velocidad_angular_real[4] = {0};
    float velocidad_angular_compensacion[4] = {0};
    float velocidad_angular_compensada[4] = {0};
    float delta_velocidad_lineal[3] = {0};

    srand(time(NULL));   // Seed para generacion de ruido
    
    printf("# Velocidad lineal deseada\n");
    for(int i=0; i<3; i++)
    {
        printf("item %d: %4.2f\n", i, velocidad_lineal[i]);
    }
    printf("\n");

    // Calculo las velocidades angulares de los motores
    printf("# Velocidad angular motores\n");
    calculo_matriz(velocidad_lineal, velocidad_angular);
    for(int i=0; i<4; i++)
    {
        printf("motor %d: %4.2f\n", i, velocidad_angular[i]);
    }
    printf("\n");
    
    // Simulacion de medicion de velocidad angular de las ruedas - induce variacion como si fuese un caso real
    printf("# Generating reality factors...\n");
    //make_variations(velocidad_angular, velocidad_angular_real, 4);
    for(int i=0; i<4; i++)
    {
        printf("motor %d real: %4.2f\n", i, velocidad_angular[i]);
    }
    printf("\n");
    
    // Calculo la velocidad lineal real
    printf("# Velocidad lineal real\n");
    calculo_matriz_inversa(velocidad_angular, velocidad_lineal_real);
    for(int i=0; i<3; i++)
    {
        printf("item %d: %4.2f\n", i, velocidad_lineal_real[i]);
    }
    printf("\n");
    
    // Calculo del error
    printf("# Calculo delta velocidad lineal\n");
    error_calc(velocidad_lineal, velocidad_lineal_real, delta_velocidad_lineal);
    for(int i=0; i<3; i++)
    {
        printf("delta item %d: %4.2f\n", i, delta_velocidad_lineal[i]); // delta > 0 significa que debe compensar en negativo, ,es decir ir mas lento
    }
    printf("\n");
    
    // Calculo de la velocidad angular de compensacion
    printf("# Calculo velocidad angular de compensacion\n");
    calculo_matriz(delta_velocidad_lineal, velocidad_angular_compensacion);
    for(int i=0; i<4; i++)
    {
        printf("motor %d delta comp: %4.2f\n", i, velocidad_angular_compensacion[i]); // delta > 0 significa que debe compensar en negativo, es decir ir mas lento
    }
    printf("\n");
    
    // Seteo de velocidades angulares compensadas
    printf("# Calculo velocidad angular compensada\n");
    for(int i=0; i<4; i++)
    {
        velocidad_angular_compensada[i] = velocidad_angular[i] - velocidad_angular_compensacion[i];
        printf("motor %d comp: %4.2f\n", i, velocidad_angular_compensada[i]);
    }
    printf("\n");
    
    return 0;
}

/*
 * Obtiene las velocidades angulares de cada rueda segun los parametros (Xr, Yr, theta)
 * */
void calculo_matriz(float *vector_velocidad_lineal, float *vector_velocidad_angular)
{
    float matriz_velocidad_lineal[4][3] = {0};

    matriz_velocidad_lineal[0][0] = (-sqrt(2)/2); // -sen(pi/4) / WHEEL_RADIUS
	matriz_velocidad_lineal[0][1] = (sqrt(2)/2); // cos(pi/4) / WHEEL_RADIUS
	matriz_velocidad_lineal[0][2] = ROBOT_RADIUS;
	matriz_velocidad_lineal[1][0] = (-sqrt(2)/2); // -sen(3/4 pi) / WHEEL_RADIUS
	matriz_velocidad_lineal[1][1] = (-sqrt(2)/2); // cos(3/4 pi) / WHEEL_RADIUS
	matriz_velocidad_lineal[1][2] = ROBOT_RADIUS;
	matriz_velocidad_lineal[2][0] = (sqrt(2)/2); // 5/4 pi
	matriz_velocidad_lineal[2][1] = (-sqrt(2)/2);
	matriz_velocidad_lineal[2][2] = ROBOT_RADIUS;
	matriz_velocidad_lineal[3][0] = (sqrt(2)/2); // 7/4 pi
	matriz_velocidad_lineal[3][1] = (sqrt(2)/2);
	matriz_velocidad_lineal[3][2] = ROBOT_RADIUS;

    for (int i=0; i<4; ++i)
    {
        vector_velocidad_angular[i] = 0;
    }
    
    for (int i=0; i<4; ++i)
    {
        for (int k=0; k<3; ++k)
        {
            vector_velocidad_angular[i] += matriz_velocidad_lineal[i][k] * vector_velocidad_lineal[k];
        }
    }
    
    printf("rpm setting:");
    for (int i=0; i<4; ++i)
    {
        printf(" %4.2f /", vector_velocidad_angular[i]);
    }
    printf("\n");
    
    return;
}

/*
 * Obtiene los parametros Xr, Yr, theta a partir de las velocidades angulares de las ruedas
 * */
void calculo_matriz_inversa(float *vector_velocidad_angular, float *vector_velocidad_lineal)
{
    float matriz_inversa[3][4] = {0};

    matriz_inversa[0][0] = (-sqrt(2) * WHEEL_RADIUS) / 4;
    matriz_inversa[0][1] = (-sqrt(2) * WHEEL_RADIUS) / 4;
    matriz_inversa[0][2] = (sqrt(2) * WHEEL_RADIUS) / 4;
    matriz_inversa[0][3] = (sqrt(2) * WHEEL_RADIUS) / 4;
    matriz_inversa[1][0] = (sqrt(2) * WHEEL_RADIUS) / 4;
    matriz_inversa[1][1] = (-sqrt(2) * WHEEL_RADIUS) / 4;
    matriz_inversa[1][2] = (-sqrt(2) * WHEEL_RADIUS) / 4;
    matriz_inversa[1][3] = (sqrt(2) * WHEEL_RADIUS) / 4;
    matriz_inversa[2][0] = WHEEL_RADIUS/(4*ROBOT_RADIUS);
    matriz_inversa[2][1] = WHEEL_RADIUS/(4*ROBOT_RADIUS);
    matriz_inversa[2][2] = WHEEL_RADIUS/(4*ROBOT_RADIUS);
    matriz_inversa[2][3] = WHEEL_RADIUS/(4*ROBOT_RADIUS);

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
        vector_velocidad_lineal[i] /= WHEEL_RADIUS;
   }

    /*printf("vel lineal real:");
    for (int i = 0; i < 3; i++)
    {
        printf(" %4.2f -", vector_velocidad_lineal[i]);
    }
    printf("\n");*/

	return;
}

void error_calc(float *velocidad_lineal, float *velocidad_lineal_real, float *delta_velocidad_lineal)
{
    for(int i=0; i<3; i++)
    {
        delta_velocidad_lineal[i] = velocidad_lineal_real[i] - velocidad_lineal[i];
    }
}

void make_variations(float *input, float *output, int size)
{
    int noise;
    
    for(int i=0; i<size; i++)
    {
        noise = rand() / 200000000.0;
        
        if(rand() % 2)
        {
            noise = -noise;
        }
        
        output[i] = input[i] + noise;
    }
}

