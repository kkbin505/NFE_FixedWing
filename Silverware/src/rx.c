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


float get_flitemode_expo (int axis){
	if (aux[LEVELMODE]){
		if (aux[RACEMODE] && !aux[HORIZON]){
			if (axis==0) return ANGLE_EXPO_ROLL;
			if (axis==1) return ACRO_EXPO_PITCH;
			if (axis==2) return ANGLE_EXPO_YAW;
		}else if (aux[HORIZON]){
			if (axis==0) return ACRO_EXPO_ROLL;
			if (axis==1) return ACRO_EXPO_PITCH;
			if (axis==2) return ANGLE_EXPO_YAW;
		}else{
			if (axis==0) return ANGLE_EXPO_ROLL;
			if (axis==1) return ANGLE_EXPO_PITCH;
			if (axis==2) return ANGLE_EXPO_YAW;
		}
	}else{
		if (axis==0) return ACRO_EXPO_ROLL;
		if (axis==1) return ACRO_EXPO_PITCH;
		if (axis==2) return ACRO_EXPO_YAW;
	}
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
		static float lastrx[3];
		static unsigned int consecutive[3];
		rxcentered[i] = (rx[i] - autocenter[i]);
		if ( rx[i] == lastrx[i] ){
			consecutive[i]++;
		}else{
			consecutive[i] = 0;
		}
		lastrx[i] = rx[i];
		if ( consecutive[i] > 750 && fabsf( rx[i]) < 0.1f ){
			autocenter[i] = rx[i];
		}
		//rxcentered[] array is shifted by the detected amount autocenter[] to restore 0 centering even if pilot has trimmed the radio and is ready to be used by the pid controller for leveled modes
		//the autocenter[] amount is the amount that each axis has been trimmed and this value will be forwarded to the mixer as a centerpoint for any leveled axis
		//apply expo normally but also cut expo by a factor of low rates multiplier when low rates are active
		if (get_flitemode_expo(i) > 0.01f) rxcentered[i] = rcexpo(rxcentered[i], (rate_multiplier * get_flitemode_expo(i)) );
		//now apply high/low rates to rxcentered as a map function instead of a factor so that expo does not cause low rates to come up short
		rxcentered[ i ] = mapf( rxcentered[ i ], -1, 1, -rate_multiplier, rate_multiplier );	
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++		
		//for sport and manual mode the rxcentered[] sticks need to be shifted back to restore pilot trims as rxcopy[i] is passed directly through the mixer
		rxcopy[i] = (rxcentered[i] + autocenter[i]);		
		//but we need to scale and remap sticks since adding back the pilot trims will have shifted the detected stick range beyond 1 or -1
		//the result should accept incoming sticks that are out of range due to trims and will stretch / squeeze throws to fit back into range at the expense of resolution
		//but also need to compensate for low rates if selected just in case (this may be redundant)	
		if (rxcopy[i] > autocenter[i]) rxcopy[i] = mapf( rxcopy[ i ], autocenter[i], rate_multiplier + autocenter[i], autocenter[i], rate_multiplier );
		if (rxcopy[i] < autocenter[i]) rxcopy[i] = mapf( rxcopy[ i ], autocenter[i], -rate_multiplier + autocenter[i], autocenter[i], -rate_multiplier );		
		//limit both rxcopy[] and rxcentered[] just in case (should have been done by the rcexpo function)
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




















