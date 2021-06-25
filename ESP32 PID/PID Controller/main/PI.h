

#define MINPWM			1200		//Minima potencia PWM
#define MAXPWM			3800  	//Maxima potencia PWM

/*
 * Prototypes
 */

void controladorPI(int dist_actual, int dist_destino);
//void set_PI_dist(int);
//int get_PI_dist();


#include "PI.c"

