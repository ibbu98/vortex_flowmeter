/*
 * freq_detect.c
 *
 *  Created on: Oct 29, 2025
 *      Author: IBRAHIM MUADH & VARUN SHAH
 */
/**
 * @file freq_detect.c
 * @brief Frequency detection and flow calculation
 */

#include "freq_detect.h"
#include <math.h>
#include <stdbool.h>

// persistent state variables
static bool prev_above = false;
static unsigned int count = 0;
static unsigned int N = 0;
static float last_freq = 0.0f;

// sampling parameters
#define FS          10000.0f     // 10 kHz sample rate
#define WINDOW_SAMPLES 1000      // update every 0.1 s
#define TH_HIGH     0.7f
#define TH_LOW     -0.7f

// constants for flow calc
#define PID_INCHES  2.900f
#define D_INCHES    0.5f
#define INCH_TO_M   0.0254f

float freq_estimate(float sample)
{
    N++;

    if (!prev_above && sample >= TH_HIGH)
    {
        count++;
        prev_above = true;
    }
    else if (prev_above && sample <= TH_LOW)
    {
        prev_above = false;
    }

    if (N >= WINDOW_SAMPLES)
    {
        last_freq = (count * FS) / (float)WINDOW_SAMPLES;
        count = 0;
        N = 0;
    }

    return last_freq;
}

/**
 * Compute flow (gallons/minute) given vortex frequency and temperature (°C)
 */
float flow_calculate(float f_hz, float T_c)
{
    // density (kg/m³)
    float rho = 1000.0f * (1.0f - ((T_c + 288.9414f) /
                  (508929.2f * (T_c + 68.12963f))) * powf((T_c - 3.9863f), 2.0f));
    // viscosity (kg/m*s)
    float viscosity = 2.4e-5f * powf(10.0f, (247.8f / (T_c + 273.15f - 140.0f)));

    // convert bluff body width to m
    float d_m = D_INCHES * INCH_TO_M;
    float pid_m = PID_INCHES * INCH_TO_M;

    // initial Strouhal (Re dependent)
    float v_est = f_hz * d_m / 0.2f; // rough guess for Re iteration
    float Re = (rho * v_est * pid_m) / viscosity;
    float St = 0.2684f - 1.0356f / sqrtf(Re);

    // refined velocity
    float v_m_s = f_hz * d_m / St;
    float v_ft_s = v_m_s * 3.28084f;

    // Flow = 2.45*(PID)^2 * v(ft/s)
    float flow_gpm = 2.45f * PID_INCHES * PID_INCHES * v_ft_s;
    return flow_gpm;
}

void freq_init(void)
{
    prev_above = false;
    count = 0;
    N = 0;
    last_freq = 0.0f;
}


