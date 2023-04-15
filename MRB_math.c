/*
 * MRB_math.c
 *
 *  Created on: 25.03.2023
 *      Author: macie
 */

void MRB_sinus_LUT_START(unsigned int magnitude, unsigned int samples, float* LUT_array)
{

   float j = 0;
   if (samples > 0)
   {

       for (int i = 0; i < samples; i++)
       {
           *(LUT_array+i) = sin(j / samples * 6.283185);
           j = j + 1;
       }
   }
}
void MRB_cosinus_LUT_START(unsigned int magnitude, unsigned int samples, float* LUT_array)
{
    unsigned int num_samples = sizeof(LUT_array);

    float j = 0;
    if (samples > 0)
    {

        for (int i = 0; i < samples; i++)
        {
            *(LUT_array + i) = cos(j / samples * 6.283185);
            j = j + 1;
        }
    }


}
double MRB_PLL_LUT(double frequency, double T_imp)
{

    static double angle;
    double dfi_dt = frequency * T_imp;

    angle = angle + dfi_dt* 6.28318530718;
    if (angle > 6.28318530718) angle = 0;
    //it returns 0 to 2pi
    return angle;
}
