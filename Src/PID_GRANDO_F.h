#ifndef PID_GRANDO_F_H_
#define PID_GRANDO_F_H_
#include "stdint.h"
#include "tgmath.h" 

//*********** Structure Definitions *******//
typedef struct {
	float  Ref;   			// Input: reference set-point
	float  Fbk;   			// Input: feedback
	float  Out;   			// Output: controller output
	float  c1;   			// Internal: derivative filter coefficient 1
	float  c2;   			// Internal: derivative filter coefficient 2
} PID_GRANDO_F_TERMINALS;
// note: c1 & c2 placed here to keep structure size under 8 words

typedef struct {
	float  Kr;				// Parameter: reference set-point weighting
	float  Kp;				// Parameter: proportional loop gain
	float  Ki;			    // Parameter: integral gain
	float  Kd; 		        // Parameter: derivative gain
	float  Km; 		        // Parameter: derivative weighting
	float  Umax;			// Parameter: upper saturation limit
	float  Umin;			// Parameter: lower saturation limit
} PID_GRANDO_F_PARAMETERS;

typedef struct {
	float  up;				// Data: proportional term
	float  ui;				// Data: integral term
	float  ud;				// Data: derivative term
	float  v1;				// Data: pre-saturated controller output
	float  i1;				// Data: integrator storage: ui(k-1)
	float  d1;				// Data: differentiator storage: ud(k-1)
	float  d2;				// Data: differentiator storage: d2(k-1)
	float  w1;				// Data: saturation record: [u(k-1) - v(k-1)]
} PID_GRANDO_F_DATA;

typedef struct {
	PID_GRANDO_F_TERMINALS	term;
	PID_GRANDO_F_PARAMETERS param;
	PID_GRANDO_F_DATA		data;
} PID_GRANDO_F_CONTROLLER;

typedef PID_GRANDO_F_CONTROLLER	*PID_handle;

//*********** Function Declarations *******//
void PID_GRANDO_F_init(PID_GRANDO_F_CONTROLLER *v);
void PID_GRANDO_F_FUNC(PID_GRANDO_F_CONTROLLER *v);

//*********** Macro Definition ***********//
#define PID_GRANDO_F_MACRO(v)																		\
	/* proportional term */ 																		\
	v.data.up = (v.param.Kr* v.term.Ref) - v.term.Fbk;												\
																									\
	/* integral term */ 																			\
	v.data.ui = (v.param.Ki* (v.data.w1* (v.term.Ref - v.term.Fbk))) + v.data.i1;					\
	v.data.i1 = v.data.ui;																			\
																									\
	/* derivative term */ 																			\
	v.data.d2 = (v.param.Kd* (v.term.c1* ((v.term.Ref* v.param.Km) - v.term.Fbk))) - v.data.d2;		\
	v.data.ud = v.data.d2 + v.data.d1;																\
	v.data.d1 = (v.data.ud* v.term.c2);																\
																									\
	/* control output */ 																			\
	v.data.v1 = (v.param.Kp* (v.data.up + v.data.ui + v.data.ud));									\
	v.term.Out= __fmax((__fmin(v.param.Umax,v.data.v1)),v.param.Umin);								\
	v.data.w1 = (v.term.Out == v.data.v1) ? (1.0) : (0.0);											\
	

#endif /* PID_GRANDO_F_H_ */
