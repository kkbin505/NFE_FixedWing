#include "rx.h"
#include "util.h"
#include "defines.h"
#include "math.h"


extern char aux[AUXNUMBER];
extern float rx[];
extern float throttle;
extern int ledcommand;
extern int levelmode_override;

float rxcopy[3];
float rxcentered[3];
float autocenter[3];
float trim[3];
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

float get_axis_expo (int axis){
	if (axis==0) return ACRO_EXPO_ROLL;
	if (axis==1) return ACRO_EXPO_PITCH;
	if (axis==2) return ACRO_EXPO_YAW;
	return 0;
}

//	https://www.desmos.com/calculator/rlfeqkmvh7    Interactive graph of unicorn rates
float apply_unicorn_expo_rates(float in, float rates, float exp){
	//unicorn rates use an adjustment on expo to bend the non-linear portion of the expo curve down to achieve a lower rate while leaving the linear portion of the expo rate curve intact.
	//This magic function will keep applied trims to control surfaces as pilot switching from high to low rates as long as trims are within the linear portion of the expo curve (safe to +/- 10% deflection)
	//and can handle low rates up to 70% lower than the slope of the linear portion of the base expo selection.  Fantastic for using lots of expo!
	limitrangef(&rates, (0.7f *(1-exp)), 1.0f );
	//the linear portion of an expo curve
	float linear = in * (1 - exp);
	//the curve portion of the expo curve with unicorn magic for low rates
	float curve = (in * in * in * in * in * exp) * ( (rates - (1 - exp)) / exp);
	return (linear + curve);
}

//flight report 1-100601  finding an incompatibility with expo and autocentering algorithm.  Already there is the issue that computing expo in the flight controller
//will always apply a curve to trims but it also reduces the applied trim range.  Maybe there is a way to match the trim setp better?  Moreover, when the autocentering algorithm finally detects a new center point ->
//there is a small but sudden change in the trimmed servo position .... because the trim moves along the expo curve - but then the expo curve re-centers.
//In order to keep this approach - it will need to track live in flight only for things like I term, but other things like stick expo recentering should only update on the ground.
//Angle mode engaged and the results with the plane being out of trim were quite terrifying.  It really only needed small corrections but controls, while doing something for sure,  didnt feel intuitive and
//I'm not sure how to fly the plane in this mode yet.  Couldnt make a turn before needing to switch out of angle.  Some of this is probably also cause by the same problem above - The impact of changing autocenter
//live in flight needs to be re-evaluated.

void apply_rates(){
	//establish rate multiplier from high-low rates switch 
	float rate_multiplier = 1.0;
	#if (defined USE_ANALOG_AUX && defined ANALOG_RATE_MULT)
		rate_multiplier = aux_analog[ANALOG_RATE_MULT];
	#else
		if ( aux[RATES]  ){	//rate multiplier already set at 1.0
		}else{ 
			rate_multiplier = LOW_RATES_MULTI;
		}
	#endif
	for( int i = 0 ; i <3;i++){
	//get raw rx[] from radio
	//	rxcopy[i] = rx[i];	not needed
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
		if ( consecutive[i] > 750 && fabsf( rx[i]) < AUTOCENTER_TRIM_LIMIT ){		//make sure we are in the linear range of expo where trims are accepted
			autocenter[i] = rx[i];																//the raw value from -1 to 1 that transmitter has been trimmed
			trim[i] = (1 - get_axis_expo(i)) * rx[i];							//the actual amount that the control surfaces deflect due to raw trim (need only compute this by the linear portion of an expo curve)
			//One unavoidable limitation of expo on FC is that transmitter trims will always be run through the expo function which reduces their throw.
			//We need to track this translation to the shorter throws to be able to command a trimmed neutral to control surfaces for levelmodes.
			//To calculate the actual trim[] deflection we dont need to compute the full expo equation since trims are very near center and follow the linear half of the expo sum     y = (1-EXP)x
			//current problem here is that we may want to use different expo values in levelmode - maybe take the unicorn approach to reduce throws
		}
		
		//rxcentered[] array is shifted by the detected raw trim amount autocenter[] to restore 0 centering even if pilot has trimmed the radio and is ready to be used by the pid controller for leveled modes
		//the autocenter[axis] amount is raw value (before expo) will compute to a trim[axis] that each axis has actually been trimmed and this value will be forwarded to the mixer as a centerpoint for any leveled axis
		//apply expo normally but also cut expo by the "unicorn rates" factor whenever low rates are active ///////a factor of low rates multiplier when low rates are active
		
		//initial tests using only acro expo values
		//Unicorn rates are magic - they stay linear where trims take place near center stick and then manipulate curved part of expo to accomplish high and low rates
		if (fabsf(get_axis_expo(i)) > 0.01f) rxcentered[i] = apply_unicorn_expo_rates(rxcentered[i], rate_multiplier, get_axis_expo(i) );
		//for sport and manual mode the rxcentered[] sticks need to be shifted back to restore pilot trims as rxcopy[i] is passed directly through the mixer
		rxcopy[i] = (rxcentered[i] + autocenter[i]);	
		
		//Is this all wrong below?  The purpose of trim is to find neutral on a control surface.  If things are even close to mechanically set up correctly then
		//we should need 1000us of throw on either side of the trimmed neutral position.  Scaling will probably just cause an imbalance in control surface deflection authority
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++		
		//Make Go Away or Optional Maybe?		
		//but we need to scale and remap sticks since adding back the pilot trims will have shifted the detected stick range beyond 1 or -1
		//the result should accept incoming sticks that are out of range due to trims and will stretch / squeeze throws to fit back into range at the expense of resolution
		//but also need to compensate for low rates if selected just in case (this may be redundant - but is most certainly broken with unicorn rates)	
		//#define FORCE_SCALE_STICK_THROWS
		#ifdef FORCE_SCALE_STICK_THROWS
		if (rxcopy[i] > trim[i]) rxcopy[i] = mapf( rxcopy[ i ], trim[i], 1.0f + autocenter[i], trim[i], 1 );
		if (rxcopy[i] < trim[i]) rxcopy[i] = mapf( rxcopy[ i ], trim[i], -1.0f + autocenter[i], trim[i], -1 );	
		#endif

//NOPE - this can go away too.  We dont want to limit the sticks on an airplane.  Servos can respond outside of the official range and if pilot inputs trims then we
//want to allow the overall stick range to shift within reason.  Limits for the Pidsums or maybe the mixer need to be carefully considered so that the pid controller doesnt explode servo gears 
		//limit both rxcopy[] and rxcentered[] just in case (should have been done by the rcexpo function)
		//limitf(&rxcopy[i], 1.0);
		//limitf(&rxcentered[i], 1.0);
//a better limit for the rxcopy sticks would be this
		//limitrangef(&rxcopy[i], -1.0f + autocenter[i], 1.0f + autocenter[i]);
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




















