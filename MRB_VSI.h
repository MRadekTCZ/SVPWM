/*
 * MRB_VSI.h
 *
 *  Created on: 25.03.2023
 *      Author: macie
 */

#ifndef INC_MRB_VSI_H_
#define INC_MRB_VSI_H_

unsigned short int MRB_VSI_Ramp(float* actual_frequency, float set_ramp_freqency, float ramp_time, float T_imp, int* start);
unsigned short int MRB_VSI_SoftStart(float* actual_frequency, float ramp_time, float T_imp);
unsigned short int MRB_VSI_SoftStop(float* actual_frequency, float ramp_time, float T_imp);
enum VSI_STATE {off, on};
enum VSI_CONTROL {wait, ramp, soft_start, soft_stop};

enum Error_Code{NoError=0, WrongInputCode=0xFFF1, SetFreqTooHigh=0xFFF2, NotANumber=0xFFF3, SetTimeTooLong =0xFFF4}ErrorCode;

struct RAMP_PARAMS
{
	float set_ramp_freqency;
	float ramp_time;
	int ramp_enable;
};


#endif /* INC_MRB_VSI_H_ */
