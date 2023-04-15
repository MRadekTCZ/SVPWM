/*
 * MRB_math.h
 *
 *  Created on: 25.03.2023
 *      Author: macie
 */
#define INC_MRB_MATH_H_
#ifdef INC_MRB_MATH_H_

void MRB_sinus_LUT_START(unsigned int magnitude, unsigned int samples, float* LUT_array);
void MRB_cosinus_LUT_START(unsigned int magnitude, unsigned int samples, float* LUT_array);
double MRB_PLL_LUT(double frequency, double T_imp);

#endif /* INC_MRB_MATH_H_ */
