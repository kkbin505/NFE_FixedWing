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

int onground = 1;
int levelmode_override = 0;
float	throttle;
int idle_state;
float mix[4];
float thrsum;
float motormap( float input);


void control( void)
{	

apply_rates();
	
pid_precalc();	

apply_flight_modes();
       
#ifdef YAW_FIX		//not sure how this will translate to planes
	rotateErrors();
#endif
	//******************Going to need to call different pid systems here or have them switch inside these functions depending on flight mode
	pid(0);
	pid(1);
	pid(2);
	

#ifndef THROTTLE_SAFETY
	#define THROTTLE_SAFETY .15f
#endif

		
#ifndef ARMING
 armed_state = 1;																							 									 // if arming feature is disabled - quad is always armed
#else																												  											// CONDITION: arming feature is enabled
	if (!aux[ARMING]){																					 										  // 						CONDITION: switch is DISARMED
		armed_state = 0;																															  // 												disarm the quad by setting armed state variable to zero
		if (rx_ready ==1)	binding_while_armed = 0;																			//                        rx is bound and has been disarmed so clear binding while armed flag
	}else{ 																				   						  										// 						CONDITION: switch is ARMED
		if (((rx[3] > THROTTLE_SAFETY) && (arming_release == 0)) || (binding_while_armed == 1)){ 		//				   CONDITION: (throttle is above safety limit and ARMING RELEASE FLAG IS NOT CLEARED) OR (bind just took place with transmitter armed)		
			armed_state = 0;																				 										  //                         	 				override to disarmed state and rapid blink the leds
		  ledcommand = 1;
		}else{																									  										  //            					 CONDITION: quad is being armed in a safe state 																		
			armed_state = 1;                                        										  //                      					  arm the quad by setting armed state variable to 1
		  arming_release = 1;																														//                       						clear the arming release flag - the arming release flag being cleared
		}																													 										  //											 						is what stops the quad from automatically disarming again the next time
	}																																									//											 						throttle is raised above the safety limit
#endif

#ifndef IDLE_UP
 idle_state = 0;
#else
	if (!aux[IDLE_UP]){
		idle_state = 0;
	}else{ idle_state = 1;}
#endif
	
#ifndef IDLE_THR
	#define IDLE_THR .001f
#endif

	if (armed_state == 0){                                     												// CONDITION: armed state variable is 0 so quad is DISARMED					
		throttle = 0;																																		//						override throttle to 0
		in_air = 0;																																			//						flag in air variable as NOT IN THE AIR for mix throttle increase safety
		arming_release = 0;																															//						arming release flag is set to not cleared to reactivate the throttle safety limit for the next arming event
	
	}else{                                                    	  										// CONDITION: armed state variable is 1 so quad is ARMED							 
			if (idle_state == 0){                                     										//            CONDITION: idle up is turned OFF				
				if ( rx[3] < 0.05f ){
					throttle = 0;                      																				//   											set a small dead zone where throttle is zero and
				  in_air = 0;																																//												deactivate mix increase 3 since throttle is off
				}else{ 
					throttle = (rx[3] - 0.05f)*1.05623158f;            												//                        map the remainder of the the active throttle region to 100%
					in_air = 1;}																															//												activate mix increase since throttle is on
			}else{ 																																				//						CONDITION: idle up is turned ON												
				throttle =  (float) IDLE_THR + rx[3] * (1.0f - (float) IDLE_THR);						//            						throttle range is mapped from idle throttle value to 100%							  
				if ((rx[3] > THROTTLE_SAFETY) && (in_air == 0)) in_air = 1; 			  				//            						change the state of in air flag when first crossing the throttle 
			}																																							//            						safety value to indicate craft has taken off for mix increase safety
	}

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
			apply_manual_mixer();
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
			apply_rate_mixer(); 								// levelmode mixer
		}else{
			if (aux[MANUALMODE]){
				apply_manual_mixer();							// manualmode mixer
			}else{
				apply_sport_mixer();							// sport/acro mixer
			}		
		}

#ifdef TORQUE_BOOST
		for ( int i = 0 ; i < 3 ; i++){			
			float motord( float in , int x);           
			mix[i] = motord(  mix[i] , i);
		}
#endif         
 
	}// end armed/motor on logic - send pwm signals next

	//begin for loop to send out pwm signals
	for ( int i = 0 ; i <= 3 ; i++){
		//***********************Send Motor PWM Command Logic			
		//***********************Clip mmixer outputs 
			if ( mix[i] < 0 ) mix[i] = 0;    
			if ( mix[i] > 1 ) mix[i] = 1;
			mix[i] = .001f * ( PWMFREQ + ( PWMFREQ * mix[i] ) ); //Normalize mixer output to servo pulses, compensating for pwm frequency
		#ifdef PWM_MOSFET_INVERSION	
			//the line below inverts the signal when using a brushed FC through the mosfets
			mix[i] = 1.0f - mix[i];
		#endif
		pwm_set( i ,motormap( mix[i] ) );	
	}
	//***********************End Motor PWM Command Logic
	
	thrsum = throttle;		//calculate throttle sum for voltage monitoring logic in main loop	
}
// end of control function




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

