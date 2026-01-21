/*
 * flow_cal.c
 *
 *  Created on: Oct 29, 2025
 *      Author: IBRAHIM MUADH & VARUN SHAH
 */

#include "flow_cal.h"
#include <math.h>

static inline float inch_to_m(float in){ return in * 0.0254f; }

void flow_geom_init(flow_geom_t *g, float d_inch, float pid_inch){
    g->d_m   = inch_to_m(d_inch);
    g->pid_m = inch_to_m(pid_inch);
    g->area_m2 = 3.14159265358979323846f * (g->pid_m*0.5f) * (g->pid_m*0.5f);
}

float water_density_kg_m3(float T_C){
    float T = T_C;
    return 1000.0f*(1.0f - ((T+288.9414f)/(508929.2f*(T+68.12963f)))*((T-3.9863f)*(T-3.9863f)));
}

float water_viscosity_kg_ms(float T_K){
    return 2.4e-5f * powf(10.0f, 247.8f/(T_K - 140.0f));
}

static inline float strouhal_from_Re(float Re){
    return 0.2684f - 1.0356f/sqrtf(fmaxf(Re,1.0f));
}

float vortex_velocity_mps(float f_Hz, float T_C, const flow_geom_t *g){
    if (f_Hz <= 0.0f) return 0.0f;
    const float rho = water_density_kg_m3(T_C);
    const float mu  = water_viscosity_kg_ms(T_C + 273.15f);

    // fixed-point iteration on v
    float v = 1.0f; // initial guess
    for (int i=0;i<50;i++){
        float Re = (rho * v * g->pid_m) / fmaxf(mu, 1e-9f);
        float St = strouhal_from_Re(Re);
        float v_new = (f_Hz * g->d_m) / fmaxf(St, 1e-6f);
        if (fabsf(v_new - v) < 1e-6f) { v = v_new; break; }
        v = v_new;
    }
    return v; // m/s
}

float flow_gpm(float v_mps, const flow_geom_t *g){
    const float Q_m3s = g->area_m2 * v_mps;
    return Q_m3s * 15850.323141489f; // GPM
}

