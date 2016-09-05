#ifndef _MATHLIB_H
#define _MATHLIB_H

#ifndef M_PI
#define M_PI			3.14159265358979323846
#endif

#ifndef M_PI_F
#define M_PI_F			((float)M_PI)
#endif

/*
#if defined(__cplusplus)
#define isfinite(x) true
#define isnan(x) false
#define isinf(x) false
#define fminf(x, y) ((x) > (y) ? y : x)
#endif
*/

struct LowPassFilter2p
{
    float           cutoff_freq; 
    float           a1;
    float           a2;
    float           b0;
    float           b1;
    float           b2;
    float           delay_element_1;        // buffered sample -1
    float           delay_element_2;        // buffered sample -2
};

void LowPassFilter2p_Init(struct LowPassFilter2p *filter, float sample_freq, float cutoff_freq);
void LowPassFilter2p_set_cutoff_frequency(struct LowPassFilter2p *filter, float sample_freq, float cutoff_freq);
float LowPassFilter2p_apply(struct LowPassFilter2p *filter, float sample);
float LowPassFilter2p_reset(struct LowPassFilter2p *filter, float sample) ;

#endif

