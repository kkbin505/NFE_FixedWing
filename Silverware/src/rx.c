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
float rates[3];




void apply_rates(){																		//apply rates high/low rates first then expo.  when low rates are selected - 
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
		static float autocenter[3];
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
	//apply low rates to rxcentered (used in levelmode) - sticks are already corrected to a 0 centerpoint so we can use the low rates multiplier with multiplication
		if (!aux[RATES]){
			rxcentered[i] = rxcentered[i] * rate_multiplier;
		}
	//finally we apply expo normally but also cut expo by a factor of low rates multiplier when low rates are active
		
		

			//if in sport or manual mode
			//first we scale since pilot trims will shift the detected stick range beyond 1 or -1
			//this will accept a stick scale that is out of range due to trims and will stretch / squeeze throws to fit back into range at the expense of resolution
			//scale raw sticks ....  range: from [(detected trimmed centerpoint) to (1.0 +/- detected trim amount)] to [(-1 to detected trimmed center) and (detected trim center to +1)]
	
			//next we apply low rates - Using low rates multiplier as a factor against rx[] will cause a loss of trim on the control surfaces.  A different approach must be taken.
			//low rates are also performed as a map function substituting 1.0 in the previous stick scale for low rates multiplier - this will retain the trimmed pilot centerpoint
	
			//finally apply expo from detected trim center to endpoints
		
	//limit rxcopy[]
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




















