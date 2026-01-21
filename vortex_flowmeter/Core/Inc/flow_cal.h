/*
 * flow_cal.h
 *
 *  Created on: Oct 29, 2025
 *      Author: IBRAHIM MUADH & VARUN SHAH
 */

#pragma once
#include <stdint.h>

typedef struct {
    float d_m;        // bluff width (m)
    float pid_m;      // pipe ID (m)
    float area_m2;    // pipe area (m^2)
} flow_geom_t;

void flow_geom_init(flow_geom_t *g, float d_inch, float pid_inch);

// property models
float water_density_kg_m3(float T_C);
float water_viscosity_kg_ms(float T_K);

// solve v from f, T using fixed point on St(Re)
float vortex_velocity_mps(float f_Hz, float T_C, const flow_geom_t *g);

// compute GPM
float flow_gpm(float v_mps, const flow_geom_t *g);

