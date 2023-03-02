#include "mixer.h"
#include "defines.h"

extern float pidoutput[PIDNUMBER];
extern float mix[4];
extern float throttle;
extern float rxcopy[3];



void invert_servo_throws(){
#ifdef INVERT_AILERON_SERVO
	mix[0] = 1.0f - mix[0];
#endif
#ifdef INVERT_ELEVATOR_SERVO
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
void apply_subtrim(){
	mix[0] += (float)ROLL_SUBTRIM/2.0f;
	mix[1] += (float)PITCH_SUBTRIM/2.0f;
	mix[2] += (float)YAW_SUBTRIM/2.0f;
}

//silverware mixer output is limited from 0 to 1, throttle is already 0 to 1 but sticks in rx[] are -1 to 1.  Neutral servos would be expressed as .5
void apply_rate_mixer(){	//used in levelmode & horizon
	//for the rate based mixer we take the raw stick conversion factor and plug in a neutral starting point (0.5) plus any detected trim which needs to be retained (autocenter[axis]/2 since we scale total stick range from 2 to 1).
	extern float trim[3];
	mix[MOTOR_BL] = (trim[ROLL]/2.0f) + 0.5f + pidoutput[ROLL];						// M0			Aileron
	mix[MOTOR_FL] = (trim[PITCH]/2.0f) + 0.5f + pidoutput[PITCH];   			// M1			Elevator
	mix[MOTOR_BR] = ((rxcopy[2]+1.0f)/2.0f) + pidoutput[YAW];	 	 					// M2			Rudder
	mix[MOTOR_FR] = throttle; 																						// M3			Throttle		
}


void apply_racemode_mixer(){	//used in racemode and racemode horizon
	extern float trim[3];
	mix[MOTOR_BL] = (trim[ROLL]/2.0f) + 0.5f + pidoutput[ROLL];						// M0			Aileron
	mix[MOTOR_FL] = ((rxcopy[1]+1.0f)/2.0f) + pidoutput[PITCH];				    // M1			Elevator
	mix[MOTOR_BR] = ((rxcopy[2]+1.0f)/2.0f) + pidoutput[YAW];	 	 					// M2			Rudder
	mix[MOTOR_FR] = throttle; 																						// M3			Throttle		
}


void apply_manual_mixer(){	//no stabilization
	mix[MOTOR_BL] = (rxcopy[0]+1.0f)/2.0f;																// M0			Aileron
	mix[MOTOR_FL] = (rxcopy[1]+1.0f)/2.0f;   															// M1			Elevator
	mix[MOTOR_BR] = (rxcopy[2]+1.0f)/2.0f;  															// M2			Rudder
	mix[MOTOR_FR] = throttle; 																						// M3			Throttle
}


void apply_sport_mixer(){	//nfe special sauce
	mix[MOTOR_BL] = ((rxcopy[0]+1.0f)/2.0f) + pidoutput[ROLL];						// M0			Aileron
	mix[MOTOR_FL] = ((rxcopy[1]+1.0f)/2.0f) + pidoutput[PITCH];   				// M1			Elevator
	mix[MOTOR_BR] = ((rxcopy[2]+1.0f)/2.0f) + pidoutput[YAW];  						// M2			Rudder
	mix[MOTOR_FR] = throttle; 																						// M3			Throttle
}


void modify_mixer_outputs(){
	invert_servo_throws();
	apply_subtrim();

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

