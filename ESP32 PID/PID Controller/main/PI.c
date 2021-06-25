
//#include "MotorControl.h"
//
//
//#define MINPWM			1200		//Minima potencia PWM
//#define MAXPWM			3800  	//Maxima potencia PWM
//
//#include "PI.h"
//#include "MotorControl.h"


//static unsigned int dist_set = 200; // distancia destino (0-400)

//static const float Kpwm = (MAXPWM - MINPWM)/400;
//static const float OffsetPwm = MINPWM;
//static const float Ts = 0.001;		// Tiempo de muestreo [s] usado para el controlador PI

/*
 * Prototypes
 */
int abs(int);
int constrain(int, int, int);
//int get_PI_dist();

/*
 * Toma como argumento la distancia actual y la compara con la distancia seteada.
 */
void controladorPI(int dist_actual, int dist_destino) {

	static const float Kpwm = (MAXPWM - MINPWM)/400;
	static const float OffsetPwm = MINPWM;
	static const float Ts = 0.001;		// Tiempo de muestreo [s] usado para el controlador PI

	//unsigned int dist_set = 200; // distancia destino (0-400)

	/* Variables y constantes de la ley de control */
	const float  Kp = 10.00; 	// Constante Kp. Mientras mayor, mas potencia tiene el auto cuando el error es grande (Este valor es Kc)
	const float  Ti = 0.075;  	// Constante Ti. SI ES MUY GRANDE SE ANULA LA ACCION INTEGRADORA
	int 		 u  = 0;	 	// Acción de control
	float 		 P  = 0.0;	 	// Acción proporcional.
    float 		 I  = 0.0;	 	// Acción integral, estado k.  Trata de llevar el error al minimo.
 	static float Ik_1 = 0.0; 	// Accion integral, estado k+1

	/* Cálculo de coeficientes del controlador */
	//int error = dist_actual-dist_set; 	// 0<error<400 [cm]
 	int error = dist_actual-dist_destino; 	// 0<error<400 [cm]
	if(error>0){						// Define la direccion del motor
 		motorSetDirection(FORWARD);}
	else{
		motorSetDirection(BACKWARDS);
	}
	error = abs(error);					//Devuelve el valor absoluto de un entero

	/* Ley de Control PID */
	P = (float) Kp*error;
	I = (float) Ik_1;

	/* Ejecución de la acción */
	u = (int) constrain(((P + I) * Kpwm + OffsetPwm),MINPWM,MAXPWM); 	/* Limita un valor a un minimo y un maximo */

	/* Control para que el motor no siga trabajando cuando llega a la distancia objetivo */
	if(error>1)
		motorSetSpeed(u);
	else
		motorStop(STOP_H);

	// Actualización de valores para la próxima iteración
	if(error>1)
		Ik_1 = I + Kp*Ts / Ti*error;
	else
		Ik_1 = 0.0;
}


/* Devuelve el valor absoluto de un entero */
int abs(int val){
	if(val>0)
		return val;
	else
		return (val*(-1));
}

/* Limita un valor a un minimo y un maximo */
int constrain(int val,int min, int max)
{
	if((val>=min) && (val<=max))
	{
		return val;
	}

	else if (val<min)
	{
		return min;
	}

	else if (val>max)
	{
		return max;
	}

	return 0; //error
}

