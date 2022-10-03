#include "mixer.h"
#include "defines.h"

extern float pidoutput[PIDNUMBER];
extern float mix[4];
extern float throttle;
extern float rxcopy[4];



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


void apply_rate_mixer(){
	mix[MOTOR_BL] = 0.5f + pidoutput[ROLL];																// M0			Aileron
	mix[MOTOR_FL] = 0.5f + pidoutput[PITCH];   														// M1			Elevator
	mix[MOTOR_BR] = 0.5f + pidoutput[YAW];  															// M2			Rudder
	mix[MOTOR_FR] = throttle; 																						// M3			Throttle		
	invert_servo_throws();
}


void apply_manual_mixer(){
	mix[MOTOR_BL] = (rxcopy[0]+1.0f)/2.0f;																// M0			Aileron
	mix[MOTOR_FL] = (rxcopy[1]+1.0f)/2.0f;   															// M1			Elevator
	mix[MOTOR_BR] = (rxcopy[2]+1.0f)/2.0f;  															// M2			Rudder
	mix[MOTOR_FR] = throttle; 																						// M3			Throttle
	invert_servo_throws();
}


void apply_sport_mixer(){
	mix[MOTOR_BL] = ((rxcopy[0]+1.0f)/2.0f) + pidoutput[ROLL];						// M0			Aileron
	mix[MOTOR_FL] = ((rxcopy[1]+1.0f)/2.0f) + pidoutput[PITCH];   				// M1			Elevator
	mix[MOTOR_BR] = ((rxcopy[2]+1.0f)/2.0f) + pidoutput[YAW];  						// M2			Rudder
	mix[MOTOR_FR] = throttle; 																						// M3			Throttle
	invert_servo_throws();
}




