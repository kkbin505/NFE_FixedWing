/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <inttypes.h>
#include <math.h>

#include "pid.h"
#include "util.h"
#include "drv_pwm.h"
#include "control.h"
#include "defines.h"
#include "drv_time.h"
#include "sixaxis.h"
#include "drv_fmc2.h"
#include "drv_fmc.h"
#include "flip_sequencer.h"
#include "gestures.h"
#include "led.h"
#include "rx.h"
#include "flight_modes.h"
#include "throttle.h"
#include "mixer.h"
#include "safety.h"


extern int armed_state;
extern int in_air;
extern int arming_release;
extern int binding_while_armed;
extern int rx_ready;
extern float rx[];
extern int failsafe;
extern float angleerror[];
extern float attitude[];
extern char aux[AUXNUMBER];
extern int ledcommand;
extern float apid(int x);
extern float throttle;

int onground = 1;
int levelmode_override = 0;
float mix[4];
float thrsum;
float servo_pwm(float mixer_output);


void control( void)
{	

	apply_rates();
	
	pid_precalc();	

	apply_flight_modes();
       
#ifdef YAW_FIX		//not sure how this will translate to planes
	rotateErrors();
#endif
	
	pid(0);
	pid(1);
	pid(2);
	
	apply_safety_flags();	
	
#ifdef STICK_TRAVEL_CHECK																				//This feature completely disables throttle and allows visual feedback if control inputs reach full throws
	//Stick endpoints check tied to aux channel stick gesture
	apply_stick_travel_check();
#endif



// FAILSAFE AND DISARMED BEHAVIOR
	if ( failsafe || armed_state == 0 ) {
		// motor off
		throttle = 0;										//zero out throttle so it does not come back on as idle up value if enabled			
		onground = 1;
		thrsum = 0;
		
		
		if ( failsafe && in_air ) {
			//craft has failsafed in the air and should come down in levelmode with motor off
			levelmode_override = 1;
			apply_rate_mixer();
		}else{
			//craft is safely on the ground and should display manual mode behavior with no throttle
			apply_manual_mixer();							// manualmode mixer
			levelmode_override = 0;
		}	
	}
	else// ARMED / READY TO FLY / FLYING BEHAVIOR
	{
		
		onground = 0;
		levelmode_override = 0;
		apply_throttle();
		
		if (aux[LEVELMODE]){
			if (aux[RACEMODE]){
				apply_racemode_mixer(); 								// racemode and racemode horizon mixer
			}else{
				apply_rate_mixer(); 								// levelmode mixer and horizon mixer
			}
		}else{
			if (aux[MANUALMODE]){
				apply_manual_mixer();							// manualmode mixer
			}else{
				apply_sport_mixer();						// sport/acro mixer is the default with all flight mode aux set to off
			}		
		}        
	}
	
	modify_mixer_outputs();

	//***********************Send Servo PWM Command Logic
	for ( int i = 0 ; i <= 3 ; i++){				
		pwm_set( i , servo_pwm( mix[i] ) );	
	}
	//***********************End Servo PWM Command Logic
	
	thrsum = throttle;		//calculate throttle sum for voltage monitoring logic in main loop	
}
// end of control function

//float servo_pwm(float mixer_output);
float servo_pwm (float mixer_output){
	float pwm_pulse = .001f * ( PWMFREQ + ( PWMFREQ * mixer_output ) ); //Normalize mixer output to servo pulses, compensating for pwm frequency
#ifdef PWM_MOSFET_INVERSION	
	//the line below inverts the signal when using a brushed FC through the mosfets
	pwm_pulse = 1.0f - pwm_pulse;
#endif
	return pwm_pulse;
}



