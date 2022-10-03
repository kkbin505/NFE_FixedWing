#include "rx.h"
#include "util.h"
#include "defines.h"

extern float rxcopy[4];
extern char aux[AUXNUMBER];
extern float rx[];
extern float rates[3];





void apply_rates(){
	// high-low rates switch 
	float rate_multiplier = 1.0;

	#if (defined USE_ANALOG_AUX && defined ANALOG_RATE_MULT)
		rate_multiplier = aux_analog[ANALOG_RATE_MULT];
	#else
		if ( aux[RATES]  )
		{		
		
		}
		else
		{
			rate_multiplier = LOW_RATES_MULTI;
		}

	#endif
	

	// make local copy of rx array											//need to apply low rates, expo, and auto-centering scale mapping to sticks here	
		for ( int i = 0 ; i < 3 ; i++)
	{
		#ifdef STOCK_TX_AUTOCENTER
		rxcopy[i] = (rx[i] - autocenter[i]);
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
	}


  rates[0] = rxcopy[0] * (float) MAX_RATE * DEGTORAD;
  rates[1] = rxcopy[1] * (float) MAX_RATE * DEGTORAD;
  rates[2] = rxcopy[2] * (float) MAX_RATEYAW * DEGTORAD;
}
