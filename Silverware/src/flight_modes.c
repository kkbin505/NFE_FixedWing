#include "flight_modes.h"
#include "defines.h"
#include "util.h"
#include "math.h"

extern float rxcentered[3];
extern char aux[AUXNUMBER];
extern float rx[];
extern float rates[3];
extern float error[PIDNUMBER];
extern float gyro[3];
extern float attitude[];
extern float angleerror[];
extern float apid(int x);
extern float setpoint[3];
extern int acro_override;
extern int levelmode_override;

void apply_flight_modes(){
if (aux[LEVELMODE] || levelmode_override){
	extern void stick_vector( float rx_input[] , float maxangle);
	extern float errorvect[]; // level mode angle error calculated by stick_vector.c	
	extern float GEstG[3]; // gravity vector for yaw feedforward
	float yawerror[3] = {0}; // yaw rotation vector
	// calculate roll / pitch error
	stick_vector( rxcentered , 0 );
	// apply yaw from the top of the quad 
	yawerror[0] = GEstG[1] * rates[2];
	yawerror[1] = - GEstG[0] * rates[2];
	yawerror[2] = GEstG[2] * rates[2];
	
	
	// *************************************************************************
	//horizon modes tuning variables
	// *************************************************************************
	// 1.0 is pure angle based transition, 0.0 is pure stick defelction based transition, values inbetween are a mix of both.  Adjust from 0 to 1
	float HORIZON_SLIDER = 0.3f;
	//leveling transitions into acro below this angle - above this angle is all acro.  DO NOT SET ABOVE 85 DEGREES!
	float HORIZON_ANGLE_TRANSITION = 55.0f;
	//leveling transitions into acro below this stick position - beyond this stick position is all acro. Adjust from 0 to 1
	float HORIZON_STICK_TRANSITION = 0.95f;
	// *************************************************************************
	// *************************************************************************
	
	
	if ((aux[RACEMODE] && !aux[HORIZON]) && !levelmode_override)  { //racemode with angle behavior on roll ais
			if (GEstG[2] < 0 ){ // acro on roll and pitch when inverted
					error[0] = rates[0] - gyro[0];
					error[1] = rates[1] - gyro[1];
			}else{
					//roll is leveled to max angle limit
					angleerror[0] = errorvect[0] ; 
					error[0] = apid(0) + yawerror[0] - gyro[0];
					//pitch is acro 
					error[1] = rates[1] - gyro[1];}
			// yaw
			error[2] = yawerror[2] - gyro[2];
		
	}else if((aux[RACEMODE] && aux[HORIZON]) && !levelmode_override){	//racemode with horizon behavior on roll axis	
			float inclinationRoll	= attitude[0];
			float inclinationPitch = attitude[1];
			float inclinationMax;
			if (fabsf(inclinationRoll) >= fabsf(inclinationPitch)){
					inclinationMax = fabsf(inclinationRoll);
			}else{
					inclinationMax = fabsf(inclinationPitch);}
			float angleFade;
			// constrains acroFade variable between 0 and 1
			if (inclinationMax <= HORIZON_ANGLE_TRANSITION){
					angleFade = inclinationMax/HORIZON_ANGLE_TRANSITION;
			}else{
					angleFade = 1;}
			float stickFade;
			float deflection = fabsf(rxcentered[0]);
			if (deflection <= HORIZON_STICK_TRANSITION){
					stickFade = deflection/HORIZON_STICK_TRANSITION;
			}else{
					stickFade = 1;}
			float fade = (stickFade *(1-HORIZON_SLIDER))+(HORIZON_SLIDER * angleFade);
			// apply acro to roll for inverted behavior
			if (GEstG[2] < 0 ){
					error[0] = rates[0] - gyro[0];
					error[1] = rates[1] - gyro[1];
			}else{ // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point					
					angleerror[0] = errorvect[0] ;
					// roll angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
					error[0] = ((apid(0) + yawerror[0] - gyro[0]) * (1 - fade)) + (fade * (rates[0] - gyro[0]));
					//pitch is acro
					error[1] = rates[1] - gyro[1];
			}
	
			// yaw
			error[2] = yawerror[2]  - gyro[2];  
		
	}else if((!aux[RACEMODE] && aux[HORIZON]) && !levelmode_override){ //horizon overrites standard level behavior	
			//pitch and roll
			for ( int i = 0 ; i <=1; i++){	
			  	float inclinationRoll	= attitude[0];
					float inclinationPitch = attitude[1];
					float inclinationMax;
					if (fabsf(inclinationRoll) >= fabsf(inclinationPitch)){
						inclinationMax = fabsf(inclinationRoll);
					}else{
						inclinationMax = fabsf(inclinationPitch);}
					float angleFade;
					// constrains acroFade variable between 0 and 1
					if (inclinationMax <= HORIZON_ANGLE_TRANSITION){
						angleFade = inclinationMax/HORIZON_ANGLE_TRANSITION;
					}else{
						angleFade = 1;
					}
					float stickFade;
					float deflection = fabsf(rxcentered[i]);
					if (deflection <= HORIZON_STICK_TRANSITION){
						stickFade = deflection/HORIZON_STICK_TRANSITION;
					}else{
						stickFade = 1;
					}
					float fade = (stickFade *(1-HORIZON_SLIDER))+(HORIZON_SLIDER * angleFade);
					// apply acro to roll and pitch sticks for inverted behavior
					if (GEstG[2] < 0 ){
						error[i] = rates[i] - gyro[i];
					}else{ // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point					
						angleerror[i] = errorvect[i] ;
						//  angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
						error[i] = ((apid(i) + yawerror[i] - gyro[i]) * (1 - fade)) + (fade * (rates[i] - gyro[i]));
					}
			}
			// yaw
			error[2] = yawerror[2]  - gyro[2];  
			
	}else{ //standard level mode or levelmode override
	    // pitch and roll
			for ( int i = 0 ; i <=1; i++){
					angleerror[i] = errorvect[i] ;    
					error[i] = apid(i) + yawerror[i] - gyro[i];
			}
      // yaw
			error[2] = yawerror[2]  - gyro[2];
		} 
}else{	// rate mode

    setpoint[0] = rates[0];
    setpoint[1] = rates[1];
    setpoint[2] = rates[2];
          
	for ( int i = 0; i < 3; i++ ) {
		error[i] = setpoint[i] - gyro[i];
	}
}

}

