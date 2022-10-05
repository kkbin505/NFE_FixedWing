

extern int armed_state;
extern int in_air;
extern int arming_release;
extern int binding_while_armed;
extern int rx_ready;
extern float rx[];
extern int failsafe;
extern int idle_state;

float	throttle;
int idle_state;

void apply_safety_flags(){
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
}

