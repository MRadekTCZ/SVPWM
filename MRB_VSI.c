/*
 * MRB_VSI.c
 *
 *  Created on: 25.03.2023
 *      Author: macie
 */
#include <math.h>
unsigned short int MRB_VSI_Ramp(float* actual_frequency, float set_ramp_freqency, float ramp_time, float T_imp, int* start)
{


	static float freq_ramp_in;
	if((*start))
	{
		freq_ramp_in = *actual_frequency;
		*start = 0;
	}
	if (fabs(*actual_frequency - set_ramp_freqency) < 0.05)
    {

		return 1;
    }
    else if (*actual_frequency < set_ramp_freqency)
    {
        *actual_frequency =*actual_frequency + (set_ramp_freqency - freq_ramp_in) * T_imp / ramp_time;
        return 0;
    }
    else if (*actual_frequency > set_ramp_freqency)
    {
        *actual_frequency = *actual_frequency - (freq_ramp_in - set_ramp_freqency) * T_imp / ramp_time;
        return 0;
    }
    else return 0;
}

unsigned short int MRB_VSI_SoftStart(float* actual_frequency, float ramp_time, float T_imp)
{
	if (fabs(*actual_frequency - 50)< 0.05)
		{
	        return 1;
	    }
	    else if (*actual_frequency < 50)
	    {
	        *actual_frequency = *actual_frequency + 50 * T_imp / ramp_time;
	        return 0;
	    }
	    else return 0;
}

unsigned short int MRB_VSI_SoftStop(float* actual_frequency, float ramp_time, float T_imp)
{
	if (fabs(*actual_frequency - 0)< 0.05)
		{
	        return 1;
	    }
	    else if (*actual_frequency > 0)
	    {
	        *actual_frequency = *actual_frequency - 50 * T_imp / ramp_time;
	        return 0;
	    }
	    else return 0;
}
