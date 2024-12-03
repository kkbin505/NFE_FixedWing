#include "mixer.h"
#include "defines.h"
#include "util.h"

extern float pidoutput[PIDNUMBER];
extern float mix[4];
extern float throttle;
extern float rxcopy[3];



void clip_mixer_outputs(){
	extern float trim[3];
	for ( int i = 0 ; i < 3 ; i++){				
		//***********************Clip mmixer outputs - CRITICAL - This needs to be located before ALL USER OUTPUT ADJUSTMENTS like scale, subtrim, etc
		if ( mix[i] < (0 +(trim[i]/2.0f))) mix[i] = 0 +(trim[i]/2.0f);    
		if ( mix[i] > (1 +(trim[i]/2.0f))) mix[i] = 1 +(trim[i]/2.0f);
	}
	if ( mix[3] < 0 ) mix[3] = 0;    
	if ( mix[3] > 1 ) mix[3] = 1;
}



void invert_servo_throws(){
#ifdef INVERT_FLAPPERON_SERVO_LEFT
	mix[0] = 1.0f - mix[0];
#endif
#ifdef INVERT_FLAPPERON_SERVO_RIGHT
	mix[1] = 1.0f - mix[1];
#endif
#ifdef INVERT_RUDDER_SERVO
	mix[2] = 1.0f - mix[2];
#endif
}



#ifndef ROLL_SUBTRIM 
#define ROLL_SUBTRIM 0.00
#endif
#ifndef PITCH_SUBTRIM 
#define PITCH_SUBTRIM 0.00
#endif
#ifndef YAW_SUBTRIM 
#define YAW_SUBTRIM 0.00
#endif
void apply_subtrim(){				//****Has to be applied before servo inversion to keep it "in sync" with user assumption of positive and negative axis movement for configuration ease
	mix[0] += (float)ROLL_SUBTRIM/2.0f;
	mix[1] += (float)PITCH_SUBTRIM/2.0f;
	mix[2] += (float)YAW_SUBTRIM/2.0f;
}



#ifndef ROLL_SCALE_POS
#define ROLL_SCALE_POS 1.00
#endif
#ifndef ROLL_SCALE_NEG
#define ROLL_SCALE_NEG 1.00
#endif
#ifndef PITCH_SCALE_POS
#define PITCH_SCALE_POS 1.00
#endif
#ifndef PITCH_SCALE_NEG
#define PITCH_SCALE_NEG 1.00
#endif
#ifndef YAW_SCALE_POS
#define YAW_SCALE_POS 1.00
#endif
#ifndef YAW_SCALE_NEG 
#define YAW_SCALE_NEG 1.00
#endif
void apply_servo_scale(){
	extern float trim[3];
		
	if (mix[ROLL] > (trim[ROLL]/2.0f) + 0.5f) mix[ROLL] = mapf( mix[ROLL], ((trim[ROLL]/2.0f) + 0.5f), 1 +(trim[ROLL]/2.0f), ((trim[ROLL]/2.0f) + 0.5f), (1 +(trim[ROLL]/2.0f))-((1.0f-(float)ROLL_SCALE_POS)/2.0f) );
	if (mix[ROLL] < (trim[ROLL]/2.0f) + 0.5f) mix[ROLL] = mapf( mix[ROLL], ((trim[ROLL]/2.0f) + 0.5f), 0 +(trim[ROLL]/2.0f), ((trim[ROLL]/2.0f) + 0.5f), (0 +(trim[ROLL]/2.0f))+((1.0f-(float)ROLL_SCALE_NEG)/2.0f) );

	if (mix[PITCH] > (trim[PITCH]/2.0f) + 0.5f) mix[PITCH] = mapf( mix[PITCH], ((trim[PITCH]/2.0f) + 0.5f), 1 +(trim[PITCH]/2.0f), ((trim[PITCH]/2.0f) + 0.5f), (1 +(trim[PITCH]/2.0f))-((1.0f-(float)PITCH_SCALE_POS)/2.0f) );
	if (mix[PITCH] < (trim[PITCH]/2.0f) + 0.5f) mix[PITCH] = mapf( mix[PITCH], ((trim[PITCH]/2.0f) + 0.5f), 0 +(trim[PITCH]/2.0f), ((trim[PITCH]/2.0f) + 0.5f), (0 +(trim[PITCH]/2.0f))+((1.0f-(float)PITCH_SCALE_NEG)/2.0f) );

	if (mix[YAW] > (trim[YAW]/2.0f) + 0.5f) mix[YAW] = mapf( mix[YAW], ((trim[YAW]/2.0f) + 0.5f), 1 +(trim[YAW]/2.0f), ((trim[YAW]/2.0f) + 0.5f), (1 +(trim[YAW]/2.0f))-((1.0f-(float)YAW_SCALE_POS)/2.0f) );
	if (mix[YAW] < (trim[YAW]/2.0f) + 0.5f) mix[YAW] = mapf( mix[YAW], ((trim[YAW]/2.0f) + 0.5f), 0 +(trim[YAW]/2.0f), ((trim[YAW]/2.0f) + 0.5f), (0 +(trim[YAW]/2.0f))+((1.0f-(float)YAW_SCALE_NEG)/2.0f) );
}



//silverware mixer output is limited from 0 to 1, throttle is already 0 to 1 but sticks in rx[] are -1 to 1.  Neutral servos would be expressed as .5
void apply_rate_mixer(){	//used in levelmode & horizon
	//for the rate based mixer we take the raw stick conversion factor and plug in a neutral starting point (0.5) plus any detected trim which needs to be retained (autocenter[axis]/2 since we scale total stick range from 2 to 1).
	extern float trim[3];
	mix[MOTOR_BL] = (trim[ROLL]/2.0f) + 0.5f + pidoutput[ROLL]/2.0f + pidoutput[PITCH]/2.0f;	// M0			Flapperon Left
	mix[MOTOR_FL] = (trim[PITCH]/2.0f) + 0.5f + pidoutput[ROLL]/2.0f - pidoutput[PITCH]/2.0f;   // M1			Flapperon Right
	mix[MOTOR_BR] = ((rxcopy[2]+1.0f)/2.0f) + pidoutput[YAW];	 	 			    // M2			Rudder
	mix[MOTOR_FR] = throttle; 														// M3			Throttle		
}


void apply_racemode_mixer(){	//used in racemode and racemode horizon
	extern float trim[3];
	mix[MOTOR_BL] = (trim[ROLL]/2.0f) + 0.5f + pidoutput[ROLL]/2.0f + pidoutput[PITCH]/2.0f;	// M0			Flapperon Left
	mix[MOTOR_FL] = (trim[PITCH]/2.0f) + 0.5f + pidoutput[ROLL]/2.0f - pidoutput[PITCH]/2.0f;   // M1			Flapperon Right
	mix[MOTOR_BR] = ((rxcopy[2]+1.0f)/2.0f) + pidoutput[YAW];	 	 			 	// M2			Rudder
	mix[MOTOR_FR] = throttle; 														// M3			Throttle		
}


void apply_manual_mixer(){	//no stabilization
	mix[MOTOR_BL] = ((rxcopy[0]+1.0f)/2.0f+(rxcopy[1]+1.0f)/2.0f)/2.0f;				// M0			Flapperon Left
	mix[MOTOR_FL] = ((rxcopy[0]+1.0f)/2.0f-(rxcopy[1]+1.0f)/2.0f)/2.0f;	  			// M1			Flapperon Right
	mix[MOTOR_BR] = (rxcopy[2]+1.0f)/2.0f;  										// M2			Rudder
	mix[MOTOR_FR] = throttle; 														// M3			Throttle
}


void apply_sport_mixer(){	//nfe special sauce
	mix[MOTOR_BL] = ((rxcopy[0]+1.0f)/2.0f) + pidoutput[ROLL]/2.0f + pidoutput[PITCH]/2.0f;						// M0			Flapperon Left
	mix[MOTOR_FL] = ((rxcopy[1]+1.0f)/2.0f) + pidoutput[ROLL]/2.0f - pidoutput[PITCH]/2.0;   					// M1			Flapperon Right
	mix[MOTOR_BR] = ((rxcopy[2]+1.0f)/2.0f) + pidoutput[YAW];  						// M2			Rudder
	mix[MOTOR_FR] = throttle; 														// M3			Throttle
}


void modify_mixer_outputs(){
	clip_mixer_outputs();
	apply_servo_scale();
	apply_subtrim();
	invert_servo_throws();
	
	

#ifdef TORQUE_BOOST
	for ( int i = 0 ; i < 3 ; i++){			
		float motord( float in , int x);           
		mix[i] = motord(  mix[i] , i);
	}
#endif  
}

#ifndef TORQUE_BOOST
    #define TORQUE_BOOST   0.0
#endif
 float motord( float in , int x)
 {
   float factor = TORQUE_BOOST;
   static float lastratexx[4][4];
     
        float out  =  ( + 0.125f *in + 0.250f * lastratexx[x][0]
                    - 0.250f * lastratexx[x][2] - ( 0.125f) * lastratexx[x][3]) * factor; 						;

        lastratexx[x][3] = lastratexx[x][2];
        lastratexx[x][2] = lastratexx[x][1];
        lastratexx[x][1] = lastratexx[x][0];
        lastratexx[x][0] = in;
        
    return in + out;
 }

