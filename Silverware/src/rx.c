#include "rx.h"
#include "util.h"
#include "defines.h"

extern float rxcopy[4];
extern char aux[AUXNUMBER];
extern float rx[];
extern float rates[3];
extern float throttle;
extern int ledcommand;




void apply_rates(){																		//apply rates high/low rates first then expo.  when low rates are selected - also cut expo by a factor of low rates multiplier
	//hash out ideas here
	//get raw rx[] from radio
	//autocentering algorithm detects trimmed centerpoint
	//establish rate multiplier
	
	//if in sport or manual mode
			//first we scale since pilot trims will shift the detected stick range beyond 1 or -1
			//this will accept a stick scale that is out of range due to trims and will stretch / squeeze throws to fit back into range at the expense of resolution
			//scale raw sticks ....  range: from [(detected trimmed centerpoint) to (1.0 +/- detected trim amount)] to [(-1 to detected trimmed center) and (detected trim center to +1)]
	
			//next we apply low rates - Using low rates multiplier as a factor against rx[] will cause a loss of trim on the control surfaces.  A different approach must be taken.
			//low rates are also performed as a map function substituting 1.0 in the previous stick scale for low rates multiplier - this will retain the trimmed pilot centerpoint
	
			//finally apply expo from detected trim center to endpoints

	//if in levelmode or levelmode_override
			//here we take the opposite approach and instead of keeping the detected trimmed centerpoint we subtract it away shifting the sticks to be back in the -1 to 1 range
			
			//next we apply trims
			////sticks are already corrected to a 0 centerpoint so we can use the low rates multiplier with multiplication
			
			//finally we apply expo normally
	//limit rxcopy[]
	//end








	// high-low rates switch 
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
	
		
		
		
		
		
		
#ifdef STOCK_TX_AUTOCENTER
		for( int i = 0 ; i <3;i++)
			{
				if ( rx[i] == lastrx[i] )
					{
						consecutive[i]++;
						
					}
				else consecutive[i] = 0;
				lastrx[i] = rx[i];
				if ( consecutive[i] > 1000 && fabsf( rx[i]) < 0.1f )
					{
						autocenter[i] = rx[i];
					}
			}
#endif	
		
		
		
		
		
		
		
	// make local copy of rx array											//need to apply low rates, expo, and auto-centering scale mapping to sticks here	
		for ( int i = 0 ; i < 3 ; i++)
	{
		#ifdef STOCK_TX_AUTOCENTER
		rxcopy[i] = (rx[i] - autocenter[i]) * rate_multiplier;
		limitf(&rxcopy[i], 1.0);
		#else
		rxcopy[i] = rx[i] * rate_multiplier;
		limitf(&rxcopy[i], 1.0);
		#endif
		
		
		
		#ifdef STICKS_DEADBAND
		if ( fabsf( rxcopy[ i ] ) <= STICKS_DEADBAND ) {
			rxcopy[ i ] = 0.0f;
		} else {
			if ( rxcopy[ i ] >= 0 ) {
				rxcopy[ i ] = mapf( rxcopy[ i ], STICKS_DEADBAND, 1, 0, 1 );
			} else {
				rxcopy[ i ] = mapf( rxcopy[ i ], -STICKS_DEADBAND, -1, 0, -1 );
			}
		}
		#endif
		limitf(&rxcopy[i], 1.0);
	}


  rates[0] = rxcopy[0] * (float) MAX_RATE * DEGTORAD;
  rates[1] = rxcopy[1] * (float) MAX_RATE * DEGTORAD;
  rates[2] = rxcopy[2] * (float) MAX_RATEYAW * DEGTORAD;
}





void apply_stick_travel_check(){
	if (aux[CH_AUX1]){
		throttle = 0;
		if ((rx[0]<= -0.99f) || (rx[0] >= 0.99f) || (rx[1] <= -0.99f) || (rx[1] >= 0.99f) || (rx[2] <= -0.99f) || (rx[2] >= 0.99f) || (rx[3] <= 0.0f) || (rx[3] >= 0.99f)){
		ledcommand = 1;}
	}
}




















