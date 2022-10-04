#include "rx.h"
#include "util.h"
#include "defines.h"
#include "math.h"

extern float rxcopy[4];
extern char aux[AUXNUMBER];
extern float rx[];
extern float throttle;
extern int ledcommand;
extern int levelmode_override;

float rxcopy[4];
float rxcentered[3];
float autocenter[3];
float rates[3];

//probably need to make the returns of this function conditional on flight mode to support mixed combos from horizon and racemode and to eliminate the need for an acro function
float get_angle_expo (int axis){
	if (axis==0) return ANGLE_EXPO_ROLL;
	if (axis==2) return ANGLE_EXPO_PITCH;
	if (axis==3) return ANGLE_EXPO_YAW;
	return 0;
}


void apply_rates(){
	//establish rate multiplier from high-low rates switch 
	float rate_multiplier = 1.0;
	#if (defined USE_ANALOG_AUX && defined ANALOG_RATE_MULT)
		rate_multiplier = aux_analog[ANALOG_RATE_MULT];
	#else
		if ( aux[RATES]  ){	//rate multiplier already set at 1.0
		}else{ 
			rate_multiplier = LOW_RATES_MULTI;
			//expo = expo * rate_multiplier;
		}
	#endif
	for( int i = 0 ; i <3;i++){
	//get raw rx[] from radio
		rxcopy[i] = rx[i];
	//autocentering algorithm detects trimmed centerpoint	
		//static float autocenter[3];
		static float lastrx[3];
		static unsigned int consecutive[3];
		rxcentered[i] = (rxcopy[i] - autocenter[i]);
		if ( rxcopy[i] == lastrx[i] ){
			consecutive[i]++;
		}else{
			consecutive[i] = 0;
		}
		lastrx[i] = rxcopy[i];
		if ( consecutive[i] > 750 && fabsf( rxcopy[i]) < 0.1f ){
			autocenter[i] = rxcopy[i];
		}
	//if in a leveled mode - any leveled axis will be using rxcentered[] as stick input so we can aply ANGLE_EXPO_AXIS to rxcentered[] unconditionally
	//apply low rates to rxcentered (used in levelmode) - sticks are already corrected to a 0 centerpoint so we can use the low rates multiplier with multiplication
		rxcentered[i] = rxcentered[i] * rate_multiplier;
	//finally we apply expo normally but also cut expo by a factor of low rates multiplier when low rates are active
		if (get_angle_expo(i) > 0.01f) rxcentered[i] = rcexpo(rx[i], (rate_multiplier * get_angle_expo(i)) ); 	
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++		
	//if in sport or manual mode
	//first we nned to scale and remap sticks since pilot trims will shift the detected stick range beyond 1 or -1
	//the result should accept incoming sticks that are out of range due to trims and will stretch / squeeze throws to fit back into range at the expense of resolution
	//but we also need to apply the low rates if selected.  But using low rates multiplier as a factor against rxcopy[] will cause a loss of trim on the control surfaces when selecting low rates.
	//So low rates can also be performed as part of the map function substituting 1.0 in the previous stick scale for low rates multiplier - this will retain the trimmed pilot centerpoint.
	
		//scale raw sticks ....  range: from [(detected trimmed centerpoint) to (1.0 +/- detected trim amount)] to [(-1 to detected trimmed center) and (detected trim center to +1)]
		//if (rxcopy[i] > autocenter[i])  mapf the things that need mapping and scaling

			
	
	//finally apply expo from detected trim center to endpoints
		//this one will take some new math
	//limit both rxcopy[] and rxcentered[]
		limitf(&rxcopy[i], 1.0);
		limitf(&rxcentered[i], 1.0);
	//end for loop
	}
	//apply rates
  rates[0] = rxcentered[0] * (float) MAX_RATE * DEGTORAD;
  rates[1] = rxcentered[1] * (float) MAX_RATE * DEGTORAD;
  rates[2] = rxcentered[2] * (float) MAX_RATEYAW * DEGTORAD;
}



void apply_stick_travel_check(){
	if (aux[CH_AUX1]){
		throttle = 0;
		if ((rx[0]<= -0.99f) || (rx[0] >= 0.99f) || (rx[1] <= -0.99f) || (rx[1] >= 0.99f) || (rx[2] <= -0.99f) || (rx[2] >= 0.99f) || (rx[3] <= 0.0f) || (rx[3] >= 0.99f)){
		ledcommand = 1;}
	}
}




















