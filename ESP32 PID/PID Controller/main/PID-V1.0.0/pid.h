 /*
------------------------------------------------------------------------------
~ File   : pid.h
~ Author : Majid Derhambakhsh
~ Version: V1.0.0
~ Created: 02/11/2021 03:43:00 AM
~ Brief  :
~ Support:
		   E-Mail : Majid.do16@gmail.com (subject : Embedded Library Support)

		   Github : https://github.com/Majid-Derhambakhsh
------------------------------------------------------------------------------
~ Description:

~ Attention  :

~ Changes    :
------------------------------------------------------------------------------
*/

#ifndef __PID_H_
#define __PID_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* PID Mode */
typedef enum
{
	
	_PID_MODE_MANUAL    = 0,
	_PID_MODE_AUTOMATIC = 1
	
}PIDMode_TypeDef;

/* PID P On x */
typedef enum
{
	
	_PID_P_ON_M = 0, /* Proportional on Measurement */
	_PID_P_ON_E = 1
	
}PIDPON_TypeDef;

/* PID Control direction */
typedef enum
{
	
	_PID_CD_DIRECT  = 0,
	_PID_CD_REVERSE = 1
	
}PIDCD_TypeDef;

/* PID Structure */
typedef struct
{
	PIDPON_TypeDef  POnE;
	PIDPON_TypeDef  POn;
	PIDCD_TypeDef   ControllerDirection;

	double          Kp;
	double          Ki;
	double          Kd;

	double		MyInput;
	double		MyOutput;
	double		MySetpoint;

	double          OutputSum;
	double          LastInput;

	double          OutMin;
	double          OutMax;
	
}PID_TypeDef;

#endif /* __PID_H_ */
