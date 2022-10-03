#include "defines.h"


extern float throttle;


void apply_throttle(){
#ifdef 	THROTTLE_BOOST       
#ifndef THROTTLE_BOOST_FACTOR 
#define THROTTLE_BOOST_FACTOR 7.0 
#endif        
		extern float throttlehpf( float in );     
		throttle += (float) (THROTTLE_BOOST_FACTOR) * throttlehpf(throttle);
		if (throttle < 0)
			throttle = 0;
		if (throttle > 1.0f)
			throttle = 1.0f;
#endif
	           
            
#ifdef LVC_LOWER_THROTTLE
	
	#ifdef SWITCHABLE_FEATURE_2
	extern float vbatt_comp;
	extern float vbattfilt;
	extern int flash_feature_2;
	static float throttle_i = 0.0f;
	float throttle_p = 0.0f;
		if (flash_feature_2 == 1){
			// can be made into a function
			if (vbattfilt < (float) LVC_LOWER_THROTTLE_VOLTAGE_RAW ) 
			throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE_RAW - vbattfilt) *(float) LVC_LOWER_THROTTLE_KP;
			// can be made into a function
			if (vbatt_comp < (float) LVC_LOWER_THROTTLE_VOLTAGE) 
			throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE - vbatt_comp) *(float) LVC_LOWER_THROTTLE_KP;	

			if ( throttle_p > 1.0f ) throttle_p = 1.0f;	

			if ( throttle_p > 0 ) 
			{
				throttle_i += throttle_p * 0.0001f; //ki
			}
			else throttle_i -= 0.001f;// ki on release

			if ( throttle_i > 0.5f) throttle_i = 0.5f;
			if ( throttle_i < 0.0f) throttle_i = 0.0f;

			throttle -= throttle_p + throttle_i;
		}else{
			//do nothing - feature is disabled via stick gesture
		}
	#else 
	extern float vbatt_comp;
	extern float vbattfilt;
	static float throttle_i = 0.0f;
	float throttle_p = 0.0f;
		// can be made into a function
		if (vbattfilt < (float) LVC_LOWER_THROTTLE_VOLTAGE_RAW ) 
		throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE_RAW - vbattfilt) *(float) LVC_LOWER_THROTTLE_KP;
		// can be made into a function
		if (vbatt_comp < (float) LVC_LOWER_THROTTLE_VOLTAGE) 
		throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE - vbatt_comp) *(float) LVC_LOWER_THROTTLE_KP;	

		if ( throttle_p > 1.0f ) throttle_p = 1.0f;

		if ( throttle_p > 0 ) 
		{
			throttle_i += throttle_p * 0.0001f; //ki
		}
		else throttle_i -= 0.001f;// ki on release

		if ( throttle_i > 0.5f) throttle_i = 0.5f;
		if ( throttle_i < 0.0f) throttle_i = 0.0f;

		throttle -= throttle_p + throttle_i;		
	#endif
#endif


}

