/*
 * Copyright (C) Matteo Guerra
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/ftgs/ftgs.h"
 * @author Matteo Guerra
 * stabilization module for fixedwing
 */

#ifndef FTGS_H
#define FTGS_H

#include <inttypes.h>
#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"
#include "state.h"

// model coefficients
extern float Cz0;
extern float Cza;
extern float Czq;
extern float Czdm;
extern float Cx0;
extern float ki;
extern float Cxdm;
extern float Cm0;
extern float Cma;
extern float Cmq;
extern float Cmdm;

extern float cyb;
extern float cyp;
extern float cyr;
extern float cydl;
extern float cydn;
extern float clb;
extern float clp;
extern float clr;
extern float cldl;
extern float cldn;
extern float cnb;
extern float cnp;
extern float cnr;
extern float cndl;
extern float cndn;

extern float S;
extern float Rho;
extern float l;
extern float A;
extern float B;
extern float CC;
extern float EE;
extern float m;
extern float g;

extern float Cx;
extern float Cz;
extern float CZ;

extern float POT;

// control laws variables
extern int a;
extern float repiove;
extern float vBx, vBy, vBz;
extern pprz_t h_ctl_aileron_setpoint;
extern pprz_t h_ctl_rudder_setpoint;
extern pprz_t h_ctl_elevator_setpoint;
extern pprz_t v_ctl_throttle_setpoint;

extern float eth_last, evx_last, ephi_last,epsi_last; 

extern bool h_ctl_disabled;
extern float k_th,k_vx;
extern float eth, des_th;

extern void fts_init(void);
extern void activate_ftgs_external_roll(void);
extern void activate_ftgs_external_pitch(void);
extern void deactivate_ftgs(void);
extern void ftgs_module_loop(void);
extern void longitudinal_ftgs(void);
extern void lateral_ftgs(void);

#endif
