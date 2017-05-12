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
 * @file "modules/ftgs/ftgs.c"
 * @author Matteo Guerra
 * stabilization module for fixedwing based on NDI and sliding mode techniques
 */
#include <math.h>
#include "firmwares/fixedwing/guidance/guidance_common.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "modules/ftgs/ftgs.h"
#include "std.h"
#include <stdio.h>
#include "state.h"
#include "led.h"
#include CTRL_TYPE_H // dovrebbe servire per i comandi
#include "firmwares/fixedwing/autopilot.h" // a che serve? 


/* include common mode and variables definitions */

float Cz0 = -0.028;
float Cza = 3.9444;
float Czq = 4.8198;
float Czdm = 0.016558;
float Cx0 = 0.02394;

float ki = 0.0606;

float Cxdm = 0.000409*57.2957;

float Cm0 = 0;
float Cma = -0.3234;
float Cmq = -1.6834;
float Cmdm = -0.0076*57.2957;


float cyb = -0.2708;
float cyp = 0.01695;
float cyr = 0.05003;
float cydl = -0.000254*57.2957;
float cydn = 0;

float clb = 0.03319;
float clp = -0.4095;
float clr = 0.06203;
float cldl = -0.001956*57.2957;
float cldn = 0;

float cnb = 0.0228;
float cnp = -0.04139;
float cnr = -0.01002;
float cndl = 0.0001236*57.2957;
float cndn = 0;

float A = 0.02471284;
float CC = 0.037424499;
float EE = 0;
float B = 0.015835159;

float S = 0.27;
float l = 0.21;

float Rho = 1.1901;
float m = 1.1;
float g = 9.81;
float Rho0 = 1.225;

float POT = 40.0;

int a;
pprz_t h_ctl_aileron_setpoint;
pprz_t h_ctl_rudder_setpoint;
pprz_t h_ctl_elevator_setpoint;
pprz_t v_ctl_throttle_setpoint;

int sign(float x)
{
    if(x > 0) return 1;
    if(x < 0) return -1;
    return 0;
}

#ifndef Aldo
#define Aldo 0.0
#endif


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ftgs(struct transport_tx *trans, struct link_device *dev)
{
  //il messaggio deve stare dentro il telemetry-----.xml
  pprz_msg_send_FTGS(trans, dev, AC_ID,  &repiove, &k_th, &k_vx, &eth, &des_th);
}

#endif

float repiove =7.0;
bool h_ctl_disabled;
float k_th = 3;
float k_vx = 3;
float eth;
float des_th;

void fts_init(void)
{ 

h_ctl_disabled = false;

repiove = Aldo;

register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FTGS, send_ftgs);

}

void activate_ftgs_external_pitch(void)
{
if (h_ctl_disabled) {
h_ctl_pitch_loop();
lateral_ftgs();
repiove=75;
}
else repiove = 50;
}

void activate_ftgs_external_roll(void)
{
if (h_ctl_disabled) {
h_ctl_roll_loop();
longitudinal_ftgs();
repiove=75;
}
else repiove = 50;
}


void deactivate_ftgs(void)
{

h_ctl_attitude_loop();
repiove=50;
}


void ftgs_module_loop(void)
{
if (h_ctl_disabled) {
repiove = 100;
longitudinal_ftgs();
lateral_ftgs();
}
else repiove = 50;
}

float vBx, vBy, vBz;

void NEDtoBody(void)
{
 struct FloatRMat * mat = stateGetNedToBodyRMat_f();
 
 float vN = stateGetSpeedNed_f()->x;
 float vE = stateGetSpeedNed_f()->y;
 float vD = stateGetSpeedNed_f()->z;
 
 vBx = (mat->m[0*0])*vN + (mat->m[0*1])*vE + (mat->m[0*2])*vD; 
 vBy = (mat->m[1*0])*vN + (mat->m[1*1])*vE + (mat->m[1*2])*vD; 
 vBz = (mat->m[2*0])*vN + (mat->m[2*1])*vE + (mat->m[2*2])*vD;
 
 
}

float  bound_cmd(float comm, float lim)
{
float cmd;
if (comm>=lim) cmd = lim;
else {
      if (comm<-lim) cmd = -lim;
      else cmd = comm;
	 }
cmd = cmd*(9600/lim);	 
return cmd;
}

float  bound_thr(float comm)
{
float cmd;
if (comm>1) cmd = 1;
else {
      if (comm<0) cmd = 0;
      else cmd = comm;
	 }
cmd = cmd*9600;	 
return cmd;
}


void longitudinal_ftgs(void)
{

static float last_eth, last_evx;
static float theta_ref_last, theta_ref_last_d;
struct FloatEulers * franco;

franco = stateGetNedToBodyEulers_f();

float theta = franco->theta;
float q = stateGetBodyRates_f()->q;

// li troviamo dopo i venti
float wx = stateGetHorizontalWindspeed_f()->x;
float wz = stateGetHorizontalWindspeed_f()->y;

float l1=0; //dopo
float l2=0; //dopo



NEDtoBody();
float vx = vBx; // Statem(1);
float vz = vBz; //Statem(2);

//NDI
// variabili di appoggio
float ct = cosf(theta);
float st = sinf(theta);
float c2t=cosf(2.f*theta);
float s2t=sinf(2.f*theta);
float c3t=cosf(3.f*theta);
float s3t=sinf(3.f*theta);

float wx2 = powf(wx,2.f);
float wx3 = powf(wx,3.f);
float wx4 = powf(wx,4.f);
float wx5 = powf(wx,5.f);
float wx6 = powf(wx,6.f);

float wz2 = powf(wz,2.f);
float wz3 = powf(wz,3.f);
float wz4 = powf(wz,4.f);
float wz5 = powf(wz,5.f);
float wz6 = powf(wz,6.f);

float vx2 = powf(vx,2.f);
float vx3 = powf(vx,3.f);
float vx4 = powf(vx,4.f);
float vx5 = powf(vx,5.f);
float vx6 = powf(vx,6.f);

float vz2 = powf(vz,2.f);
float vz3 = powf(vz,3.f);
float vz4 = powf(vz,4.f);
float vz5 = powf(vz,5.f);
float vz6 = powf(vz,6.f);

float Czdm2 = powf(Czdm,2.f);


float Srhom = 0.5*S*Rho;
float SLrhom = 0.5*l*S*Rho;

float uno = (wz*ct + wx*st - vz);
float due = ((-wx)*ct + wz*st + vx);
float tre = ((-wz)*ct - wx*st + vz);
float quattro = (wx*ct - wz*st - vx);
float cinque = ((-wx)*ct + wz*st)*vz;

float sum1 = (wz*ct + wx*st);
float sum2 = (-wx*ct + wz*st);
float sum3 = (-wz*ct - wx*st);
float sum4 = (wx*ct - wz*st);

float p1 = powf(uno,2.f);
float p2 = powf(due,2.f);

float Radice1 = sqrtf(1.f + p1/p2);
float Radice2 = sqrtf(wx2 + wz2 + 2*sum2*vx + vx2 - 2*sum1*vz + vz2);

float ARCTAN=atan2(tre,due); 
float Radice=sqrtf(powf(due,2) + powf(uno,2));

float pow1 = powf(-Cx0 - Cxdm*l1 - ki*(Cz0 + Cza*ARCTAN + Czdm*l1 + (Czq*l*q)/Radice),2.f);
float pow2 = powf(Cz0 + Cza*ARCTAN + Czdm*l1 + (Czq*l*q)/Radice,2.f);

float coeff1 = Cz0*Czdm*ki;
float coeff2 = Czdm*Cza*ki;
float coeff3 = Czdm*Czq*ki*l;





//first controlled variable *theta*




float f1 = (1/B)*(SLrhom*(Cm0 + Cma*ARCTAN + (Cmq*l*q)/Radice)*(powf(due,2) + powf(uno,2)));

float g1dx = 0;

float g1dm = (1/B)*(Cmdm*l*S*Rho*(0.5*wx2 + 0.5*wz2 + sum2*vx + 0.5*vx2 + sum3*vz + 0.5*vz2));


// second controlled variable


float f2= (q*(sum1*vx + cinque) + tre*(g*ct + q*vx + (1/(m*Radice1))*(Srhom*(p2 + p1)*(-Cz0 - Cza*ARCTAN - Czdm*l1 - (Czq*l*q)/Radice + (1/due)*(pow1*tre)))) + due*((-g)*st + (POT*l2)/Radice - q*vz + (1/(m*Radice1))*(Srhom*(p2 + p1)*(-Cx0 - Cxdm*l1 - ki*pow2 + ((Cz0 + Cza*ARCTAN + Czdm*l1 + (Czq*l*q)/Radice)*tre)/due))))/Radice;

float g2dm = (Srhom*(1.*Cxdm*wx6 + 2.*coeff1*wx6 + 3.*Cxdm*wx4*wz2 + 6.*coeff1*wx4*wz2 + 3.*Cxdm*wx2*wz4 + 6.*coeff1*wx2*wz4 + 1.*Cxdm*wz6 + 2.*coeff1*wz6 + 2.*coeff2*wx6*ARCTAN + 6.*coeff2*wx4*wz2*ARCTAN + 6.*coeff2*wx2*wz4*ARCTAN +  2.*coeff2*wz6*ARCTAN + ((-6.*Cxdm*wx - 12.*coeff1*wx)*ct + (6.*Cxdm + 12.*coeff1)*wz*st + coeff2*ARCTAN*(-12.*wx*ct + 12.*wz*st))*vx5 + (1.*Cxdm + 2.*coeff1 + 2.*coeff2*ARCTAN)*vx6 - 6.*Cxdm*wx4*wz*ct*vz - 12.*coeff1*wx4*wz*ct*vz - 12.*Cxdm*wx2*wz3*ct*vz - 24.*coeff1*wx2*wz3*ct*vz - 6.*Cxdm*wz5*ct*vz - 12.*coeff1*wz5*ct*vz - 12.*coeff2*wx4*wz*ARCTAN*ct*vz - 24.*coeff2*wx2*wz3*ARCTAN*ct*vz - 12.*coeff2*wz5*ARCTAN*ct*vz - 6.*Cxdm*wx5*st*vz - 12.*coeff1*wx5*st*vz - 12.*Cxdm*wx3*wz2*st*vz - 24.*coeff1*wx3*wz2*st*vz - 6.*Cxdm*wx*wz4*st*vz - 12.*coeff1*wx*wz4*st*vz - 12.*coeff2*wx5*ARCTAN*st*vz - 24.*coeff2*wx3*wz2*ARCTAN*st*vz - 12.*coeff2*wx*wz4*ARCTAN*st*vz + 9.*Cxdm*wx4*vz2 + 18.*coeff1*wx4*vz2 + 18.*Cxdm*wx2*wz2*vz2 + 36.*coeff1*wx2*wz2*vz2 + 9.*Cxdm*wz4*vz2 + 18.*coeff1*wz4*vz2 + 18.*coeff2*wx4*ARCTAN*vz2 + 36.*coeff2*wx2*wz2*ARCTAN*vz2 + 18.*coeff2*wz4*ARCTAN*vz2 - 6.*Cxdm*wx4*c2t*vz2 - 12.*coeff1*wx4*c2t*vz2 + 6.*Cxdm*wz4*c2t*vz2 + 12.*coeff1*wz4*c2t*vz2 - 12.*coeff2*wx4*ARCTAN*c2t*vz2 + 12.*coeff2*wz4*ARCTAN*c2t*vz2 + 12.*Cxdm*wx3*wz*s2t*vz2 + 24.*coeff1*wx3*wz*s2t*vz2 + 12.*Cxdm*wx*wz3*s2t*vz2 + 24.*coeff1*wx*wz3*s2t*vz2 + 24.*coeff2*wx3*wz*ARCTAN*s2t*vz2 + 24.*coeff2*wx*wz3*ARCTAN*s2t*vz2 - 18.*Cxdm*wx2*wz*ct*vz3 - 36.*coeff1*wx2*wz*ct*vz3 - 18.*Cxdm*wz3*ct*vz3 - 36.*coeff1*wz3*ct*vz3 - 36.*coeff2*wx2*wz*ARCTAN*ct*vz3 - 36.*coeff2*wz3*ARCTAN*ct*vz3 + 6.*Cxdm*wx2*wz*c3t*vz3 + 12.*coeff1*wx2*wz*c3t*vz3 - 2.*Cxdm*wz3*c3t*vz3 - 4.*coeff1*wz3*c3t*vz3 + 12.*coeff2*wx2*wz*ARCTAN*c3t*vz3 - 4.*coeff2*wz3*ARCTAN*c3t*vz3 - 18.*Cxdm*wx3*st*vz3 - 36.*coeff1*wx3*st*vz3 - 18.*Cxdm*wx*wz2*st*vz3 - 36.*coeff1*wx*wz2*st*vz3 - 36.*coeff2*wx3*ARCTAN*st*vz3 - 36.*coeff2*wx*wz2*ARCTAN*st*vz3 + 2.*Cxdm*wx3*s3t*vz3 + 4.*coeff1*wx3*s3t*vz3 - 6.*Cxdm*wx*wz2*s3t*vz3 - 12.*coeff1*wx*wz2*s3t*vz3 + 4.*coeff2*wx3*ARCTAN*s3t*vz3 - 12.*coeff2*wx*wz2*ARCTAN*s3t*vz3 + 9.*Cxdm*wx2*vz4 + 18.*coeff1*wx2*vz4 + 9.*Cxdm*wz2*vz4 + 18.*coeff1*wz2*vz4 + 18.*coeff2*wx2*ARCTAN*vz4 + 18.*coeff2*wz2*ARCTAN*vz4 - 6.*Cxdm*wx2*c2t*vz4 - 12.*coeff1*wx2*c2t*vz4 + 6.*Cxdm*wz2*c2t*vz4 + 12.*coeff1*wz2*c2t*vz4 - 12.*coeff2*wx2*ARCTAN*c2t*vz4 + 12.*coeff2*wz2*ARCTAN*c2t*vz4 + 12.*Cxdm*wx*wz*s2t*vz4 + 24.*coeff1*wx*wz*s2t*vz4 + 24.*coeff2*wx*wz*ARCTAN*s2t*vz4 - 6.*Cxdm*wz*ct*vz5 - 12.*coeff1*wz*ct*vz5 - 12.*coeff2*wz*ARCTAN*ct*vz5 - 6.*Cxdm*wx*st*vz5 - 12.*coeff1*wx*st*vz5 - 12.*coeff2*wx*ARCTAN*st*vz5 + 1.*Cxdm*vz6 + 2.*coeff1*vz6 + 2.*coeff2*ARCTAN*vz6 + 2.*coeff3*wx4*q*Radice2 + 4.*coeff3*wx2*wz2*q*Radice2 + 2.*coeff3*wz4*q*Radice2 - 8.*coeff3*wx2*wz*ct*q*vz*Radice2 - 8.*coeff3*wz3*ct*q*vz*Radice2 - 8.*coeff3*wx3*q*st*vz*Radice2 - 8.*coeff3*wx*wz2*q*st*vz*Radice2 + 8.*coeff3*wx2*q*vz2*Radice2 + 8.*coeff3*wz2*q*vz2*Radice2 - 4.*coeff3*wx2*c2t*q*vz2*Radice2 + 4.*coeff3*wz2*c2t*q*vz2*Radice2 + 8.*coeff3*wx*wz*q*s2t*vz2*Radice2 - 8.*coeff3*wz*ct*q*vz3*Radice2 - 8.*coeff3*wx*q*st*vz3*Radice2 + 2.*coeff3*q*vz4*Radice2 + vx4*(9.*Cxdm*wx2 + 18.*coeff1*wx2 + 9.*Cxdm*wz2 + 18.*coeff1*wz2 + 18.*coeff2*wx2*ARCTAN + 18.*coeff2*wz2*ARCTAN + 6.*Cxdm*wx2*c2t + 12.*coeff1*wx2*c2t - 6.*Cxdm*wz2*c2t - 12.*coeff1*wz2*c2t + 12.*coeff2*wx2*ARCTAN*c2t - 12.*coeff2*wz2*ARCTAN*c2t - 12.*Cxdm*wx*wz*s2t -24.*coeff1*wx*wz*s2t -24.*coeff2*wx*wz*ARCTAN*s2t + ((-6.*Cxdm*wz - 12.*coeff1*wz)*ct + (-6.*Cxdm - 12.*coeff1)*wx*st + coeff2*ARCTAN*(-12.*wz*ct - 12.*wx*st))*vz + (3.*Cxdm + 6.*coeff1 + 6.*coeff2*ARCTAN)*vz2 + 2.*coeff3*q*Radice2) + vx3*(-18.*Cxdm*wx3*ct - 36.*coeff1*wx3*ct - 18.*Cxdm*wx*wz2*ct - 36.*coeff1*wx*wz2*ct - 36.*coeff2*wx3*ARCTAN*ct - 36.*coeff2*wx*wz2*ARCTAN*ct - 2.*Cxdm*wx3*c3t - 4.*coeff1*wx3*c3t + 6.*Cxdm*wx*wz2*c3t + 12.*coeff1*wx*wz2*c3t - 4.*coeff2*wx3*ARCTAN*c3t + 12.*coeff2*wx*wz2*ARCTAN*c3t + 18.*Cxdm*wx2*wz*st + 36.*coeff1*wx2*wz*st + 18.*Cxdm*wz3*st + 36.*coeff1*wz3*st + 36.*coeff2*wx2*wz*ARCTAN*st + 36.*coeff2*wz3*ARCTAN*st + 6.*Cxdm*wx2*wz*s3t + 12.*coeff1*wx2*wz*s3t - 2.*Cxdm*wz3*s3t - 4.*coeff1*wz3*s3t + 12.*coeff2*wx2*wz*ARCTAN*s3t - 4.*coeff2*wz3*ARCTAN*s3t + ((24.*Cxdm + 48.*coeff1)*wx*wz*c2t + (12.*Cxdm*wx2 + 24.*coeff1*wx2 - 12.*Cxdm*wz2 - 24.*coeff1*wz2)*s2t + coeff2*ARCTAN*(48.*wx*wz*c2t + (24.*wx2 - 24.*wz2)*s2t))*vz + ((-12.*Cxdm*wx - 24.*coeff1*wx)*ct + (12.*Cxdm + 24.*coeff1)*wz*st + coeff2*ARCTAN*(-24.*wx*ct + 24.*wz*st))*vz2 - 8.*coeff3*q*sum4*Radice) +Czdm2*ki*l1*(2.*wx6 + 6.*wx4*wz2 + 6.*wx2*wz4 + 2.*wz6 + (-12.*wx*ct + 12.*wz*st)*vx5 + 2.*vx6 + ((-12.*wx4*wz - 24.*wx2*wz3 - 12.*wz5)*ct + wx*(-12.*wx4 - 24.*wx2*wz2 - 12.*wz4)*st)*vz + (18.*wx4 + 36.*wx2*wz2 + 18.*wz4 + (-12.*wx4 + 12.*wz4)*c2t + (24.*wx3*wz + 24.*wx*wz3)*s2t)*vz2 + ((-36.*wx2*wz - 36.*wz3)*ct + (12.*wx2*wz - 4.*wz3)*c3t - 36.*wx3*st - 36.*wx*wz2*st + 4.*wx3*s3t - 12.*wx*wz2*s3t)*vz3 + (18.*wx2 + 18.*wz2 + (-12.*wx2 + 12.*wz2)*c2t + 24.*wx*wz*s2t)*vz4 + (-12.*wz*ct - 12.*wx*st)*vz5 + 2.*vz6 + vx4*(18.*wx2 + 18.*wz2 + (12.*wx2 - 12.*wz2)*c2t - 24.*wx*wz*s2t + (-12.*wz*ct - 12.*wx*st)*vz + 6.*vz2) + vx3*((-36.*wx3 - 36.*wx*wz2)*ct - 4.*wx3*c3t + 12.*wx*wz2*c3t + 36.*wx2*wz*st + 36.*wz3*st + 12.*wx2*wz*s3t -4.*wz3*s3t + (48.*wx*wz*c2t + (24.*wx2 - 24.*wz2)*s2t)*vz +       (-24.*wx*ct + 24.*wz*st)*vz2) + vx2*(18.*wx4 + 36.*wx2*wz2 + 18.*wz4 + (12.*wx4 - 12.*wz4)*c2t - 24.*wx3*wz*s2t - 24.*wx*wz3*s2t + ((-36.*wx2*wz - 36.*wz3)*ct + (-36.*wx2*wz + 12.*wz3)*c3t - 36.*wx3*st - 36.*wx*wz2*st - 12.*wx3*s3t + 36.*wx*wz2*s3t)*vz + (36.*wx2 + 36.*wz2)*vz2 + (-24.*wz*ct - 24.*wx*st)*vz3 + 6.*vz4) + vx*((-12.*wx5 - 24.*wx3*wz2 - 12.*wx*wz4)*ct + wz*(12.*wx4 + 24.*wx2*wz2 + 12.*wz4)*st + ((48.*wx3*wz + 48.*wx*wz3)*c2t + (24.*wx4 - 24.*wz4)*s2t)*vz + ((-36.*wx3 - 36.*wx*wz2)*ct + (12.*wx3 - 36.*wx*wz2)*c3t + 36.*wx2*wz*st + 36.*wz3*st - 36.*wx2*wz*s3t + 12.*wz3*s3t)*vz2 + (48.*wx*wz*c2t + (24.*wx2 - 24.*wz2)*s2t)*vz3 + (-12.*wx*ct + 12.*wz*st)*vz4)) + vx2*(9.*Cxdm*wx4 + 18.*coeff1*wx4 + 18.*Cxdm*wx2*wz2 + 36.*coeff1*wx2*wz2 + 9.*Cxdm*wz4 + 18.*coeff1*wz4 + 18.*coeff2*wx4*ARCTAN + 36.*coeff2*wx2*wz2*ARCTAN + 18.*coeff2*wz4*ARCTAN + 6.*Cxdm*wx4*c2t + 12.*coeff1*wx4*c2t -     6.*Cxdm*wz4*c2t - 12.*coeff1*wz4*c2t + 12.*coeff2*wx4*ARCTAN*c2t - 12.*coeff2*wz4*ARCTAN*c2t -12.*Cxdm*wx3*wz*s2t - 24.*coeff1*wx3*wz*s2t - 12.*Cxdm*wx*wz3*s2t - 24.*coeff1*wx*wz3*s2t - 24.*coeff2*wx3*wz*ARCTAN*s2t - 24.*coeff2*wx*wz3*ARCTAN*s2t + ((-12.*Cxdm*wz - 24.*coeff1*wz)*ct + (-12.*Cxdm - 24.*coeff1)*wx*st + coeff2*ARCTAN*(-24.*wz*ct - 24.*wx*st))*vz3 + (3.*Cxdm + 6.*coeff1 +6.*coeff2*ARCTAN)*vz4 + 8.*coeff3*wx2*q*Radice2 + 8.*coeff3*wz2*q*Radice2 + 4.*coeff3*wx2*c2t*q*Radice2 - 4.*coeff3*wz2*c2t*q*Radice2 - 8.*coeff3*wx*wz*q*s2t*Radice2 + vz2*(18.*Cxdm*wx2 + 36.*coeff1*wx2 + 18.*Cxdm*wz2 + 36.*coeff1*wz2 + coeff2*(36.*wx2 + 36.*wz2)*ARCTAN + 4.*coeff3*q*Radice2) + vz*(-18.*Cxdm*wx2*wz*ct - 36.*coeff1*wx2*wz*ct - 18.*Cxdm*wz3*ct - 36.*coeff1*wz3*ct - 18.*Cxdm*wx2*wz*c3t - 36.*coeff1*wx2*wz*c3t + 6.*Cxdm*wz3*c3t + 12.*coeff1*wz3*c3t - 18.*Cxdm*wx3*st - 36.*coeff1*wx3*st - 18.*Cxdm*wx*wz2*st -  36.*coeff1*wx*wz2*st - 6.*Cxdm*wx3*s3t - 12.*coeff1*wx3*s3t + 18.*Cxdm*wx*wz2*s3t + 36.*coeff1*wx*wz2*s3t + coeff2*ARCTAN*((-36.*wx2*wz - 36.*wz3)*ct + (-36.*wx2*wz + 12.*wz3)*c3t - 36.*wx3*st - 36.*wx*wz2*st - 12.*wx3*s3t +36.*wx*wz2*s3t) - 8.*coeff3*q*sum1*Radice2)) + vx*(-6.*Cxdm*wx5*ct - 12.*coeff1*wx5*ct - 12.*Cxdm*wx3*wz2*ct - 24.*coeff1*wx3*wz2*ct - 6.*Cxdm*wx*wz4*ct - 12.*coeff1*wx*wz4*ct -12.*coeff2*wx5*ARCTAN*ct - 24.*coeff2*wx3*wz2*ARCTAN*ct - 12.*coeff2*wx*wz4*ARCTAN*ct + 6.*Cxdm*wx4*wz*st + 12.*coeff1*wx4*wz*st + 12.*Cxdm*wx2*wz3*st + 24.*coeff1*wx2*wz3*st + 6.*Cxdm*wz5*st + 12.*coeff1*wz5*st + 12.*coeff2*wx4*wz*ARCTAN*st + 24.*coeff2*wx2*wz3*ARCTAN*st + 12.*coeff2*wz5*ARCTAN*st + ((24.*Cxdm + 48.*coeff1)*wx*wz*c2t + (12.*Cxdm*wx2 + 24.*coeff1*wx2 - 12.*Cxdm*wz2 - 24.*coeff1*wz2)*s2t + coeff2*ARCTAN*(48.*wx*wz*c2t + (24.*wx2 - 24.*wz2)*s2t))*vz3 + ((-6.*Cxdm*wx - 12.*coeff1*wx)*ct + (6.*Cxdm + 12.*coeff1)*wz*st + coeff2*ARCTAN*(-12.*wx*ct + 12.*wz*st))*vz4 - 8.*coeff3*wx3*ct*q*Radice2 - 8.*coeff3*wx*wz2*ct*q*Radice2 + 8.*coeff3*wx2*wz*q*st*Radice2 + 8.*coeff3*wz3*q*st*Radice2 + vz2*(-18.*Cxdm*wx3*ct - 36.*coeff1*wx3*ct - 18.*Cxdm*wx*wz2*ct - 36.*coeff1*wx*wz2*ct + 6.*Cxdm*wx3*c3t + 12.*coeff1*wx3*c3t - 18.*Cxdm*wx*wz2*c3t - 36.*coeff1*wx*wz2*c3t + 18.*Cxdm*wx2*wz*st +       36.*coeff1*wx2*wz*st + 18.*Cxdm*wz3*st + 36.*coeff1*wz3*st - 18.*Cxdm*wx2*wz*s3t -  36.*coeff1*wx2*wz*s3t + 6.*Cxdm*wz3*s3t + 12.*coeff1*wz3*s3t + coeff2*ARCTAN*((-36.*wx3 - 36.*wx*wz2)*ct + (12.*wx3 - 36.*wx*wz2)*c3t + 36.*wx2*wz*st + 36.*wz3*st - 36.*wx2*wz*s3t + 12.*wz3*s3t) - 8.*coeff3*q*sum4*Radice2) + vz*(24.*Cxdm*wx3*wz*c2t + 48.*coeff1*wx3*wz*c2t + 24.*Cxdm*wx*wz3*c2t + 48.*coeff1*wx*wz3*c2t + 12.*Cxdm*wx4*s2t + 24.*coeff1*wx4*s2t - 12.*Cxdm*wz4*s2t - 24.*coeff1*wz4*s2t + coeff2*ARCTAN*((48.*wx3*wz + 48.*wx*wz3)*c2t + (24.*wx4 - 24.*wz4)*s2t) + coeff3*q*(16.*wx*wz*c2t +(8.*wx2 - 8.*wz2)*s2t)* Radice2))))/(m*quattro*Radice*Radice1*(wx2 + wz2 + 2*sum2*vx + vx2 + 2*sum3*vz + vz2));

float g2dx = (POT*due)/(wx2 + wz2 + 2*sum2*vx + vx2 - 2*sum1*vz + vz2);

// end NDI



eth = theta - h_ctl_pitch_setpoint; 
des_th = h_ctl_pitch_setpoint;

float Va = stateGetAirspeed_f();
float evx = Va-15; //vx_reference

float edth = eth-last_eth;//last - edphi
last_eth = eth;//last - edt


float theta_ref_d = h_ctl_pitch_setpoint - theta_ref_last; 
theta_ref_last = h_ctl_pitch_setpoint;

float theta_ref_dd = theta_ref_d - theta_ref_last_d;
theta_ref_last_d = theta_ref_d;

float vx_ref_d = 0;




float sm_th = -k_th*sign(eth+edth)*powf(fabsf(eth+edth),3.f/4.f)-f1*theta_ref_dd;
float sm_vx = -k_vx*sign(evx)*powf(fabsf(evx),3.f/4.f)-f2*vx_ref_d;

float determinante = (g1dm*g2dx-g1dx*g2dm);
float a11 =  (1/determinante)*g2dx;
float a12 = -(1/determinante)*g1dx;
float a21 = -(1/determinante)*g2dm;
float a22 =  (1/determinante)*g1dm;

float cmd_elev_p = a11*sm_th + a12*sm_vx;
float cmd_thrust_p = a21*sm_th + a22*sm_vx;

#ifndef MAX_DEFLECTION_ELEVATOR 
#define MAX_DEFLECTION_ELEVATOR 0.3
#endif

//check sign convention elevator
//float cmd_elev = bound_cmd(cmd_elev_p,MAX_DEFLECTION_ELEVATOR);
//float cmd_thrust = bound_thr(cmd_thrust_p);

float cmd_elev = bound_cmd(eth,MAX_DEFLECTION_ELEVATOR);


h_ctl_elevator_setpoint = TRIM_PPRZ(cmd_elev);
//v_ctl_throttle_setpoint = TRIM_UPPRZ(cmd_thrust);

FILE *log_ftgs;
log_ftgs = fopen("/home/mguerra/pprzfk/paparazzi/sw/airborne/modules/ftgs/longi_ftgs.txt","a");

fprintf(log_ftgs,"%f;",vx);//1
fprintf(log_ftgs,"%f;",vz);//2
fprintf(log_ftgs,"%f;",Va);//3
fprintf(log_ftgs,"%f;",theta);//4
fprintf(log_ftgs,"%f;",theta_ref_d);//5
fprintf(log_ftgs,"%f;",theta_ref_dd);//6
fprintf(log_ftgs,"%f;",eth);//7
fprintf(log_ftgs,"%f;",evx);//8
fprintf(log_ftgs,"%f;",des_th);//9
fprintf(log_ftgs,"%f;",sm_th);//10
fprintf(log_ftgs,"%f;",sm_vx);//11
fprintf(log_ftgs,"%f;",cmd_elev_p);//12
//fprintf(log_ftgs,"%f;",cmd_thrust_p);//13
fprintf(log_ftgs,"%f;",cmd_elev);//14
//fprintf(log_ftgs,"%f;",cmd_thrust);//15
fprintf(log_ftgs,"%d;\n", 0);//16
fclose(log_ftgs);

}



void lateral_ftgs(void)
{
FILE *log_ftgs;

static float  last_ephi, last_epsi;
static float phi_ref_last, phi_ref_last_d;
static float psi_ref_last, psi_ref_last_d;
struct FloatEulers * franco;

franco = stateGetNedToBodyEulers_f();

float phi = franco->phi;
float psi = franco->psi;
float p = stateGetBodyRates_f()->p;
float r = stateGetBodyRates_f()->r;

float wy = 0;

float l1 = 0;//L(2);
float l2 = 0;//L(3);

float cph=cosf(phi);
float sph=sinf(phi);
float cps=cosf(psi);
float sps=sinf(psi);

NEDtoBody();
float vy = vBy; // Statem(1)

float Va = stateGetAirspeed_f();

//First output parts, y=phi, y_1^(2)=f1+g1dx*dx+g1dm*dm+d1dm
float ArcSin = asin(((-wy)*(cph*cps + vy)/Va));


float f1 = (1/(A*CC - 1.*powf(EE,2)))*(l*S*Va*Rho*((0.5*CC*clb + 0.5*cnb*EE)*Va*ArcSin + (0.5*CC*clp*l + 0.5*cnp*EE*l)*p + (0.5*CC*clr*l + 0.5*cnr*EE*l)*r));

double g1dl = ((0.5*CC*cldl + 0.5*cndl*EE)*l*S*powf(Va,2)*Rho)/(A*CC - 1.*powf(EE,2));

double g1dn = ((0.5*CC*cldn + 0.5*cndn*EE)*l*S*powf(Va,2)*Rho)/(A*CC - 1.*powf(EE,2));


// Second output parts, y=psi, y_2^(2)=f2+g2dx*dx+g2dm*dm+d2dm


float f2 = (1/(A*CC - 1.*powf(EE,2)))*(l*S*Va*Rho*cph*((0.5*A*cnb + 0.5*clb*EE)*Va*ArcSin + (0.5*A*cnr*l + 0.5*clr*EE*l)*r) + p*((0.5*A*cnp + 0.5*clp*EE)*powf(l,2)*S*Va*Rho*cph + (-1.*A*CC + powf(EE,2))*r*sph));

double g2dl = ((0.5*A*cndl + 0.5*cldl*EE)*l*S*powf(Va,2)*Rho*cph)/(A*CC - 1.*powf(EE,2));

double g2dn = ((0.5*A*cndn + 0.5*cldn*EE)*l*S*powf(Va,2)*Rho*cph)/(A*CC - 1.*powf(EE,2));


// NDI


float ephi = phi - h_ctl_roll_setpoint;


float epsi = psi - h_ctl_course_setpoint;
NormRadAngle(epsi);


float edphi = ephi - last_ephi;//last - edphi
last_ephi = ephi;

float edpsi = epsi - last_epsi;//last - edpsi
last_epsi = epsi;
NormRadAngle(edpsi);

float sign1 = ephi+edphi;
float sign2 = epsi+edpsi;



float phi_ref_d = h_ctl_roll_setpoint - phi_ref_last; 
phi_ref_last = h_ctl_roll_setpoint;

float phi_ref_dd = phi_ref_d - phi_ref_last_d;
phi_ref_last_d = phi_ref_d;

float psi_ref_d = h_ctl_course_setpoint - psi_ref_last; 
psi_ref_last = h_ctl_pitch_setpoint;

float psi_ref_dd = psi_ref_d - psi_ref_last_d;
psi_ref_last_d = psi_ref_d;


float k_phi = 1;
float k_psi = 1;

float sm_phi = -k_phi*sign(sign1)*powf(fabsf(sign1),3/4)-f1*phi_ref_dd;
float sm_psi = -k_psi*sign(sign2)*powf(fabsf(sign2),3/4)-f2*psi_ref_dd;





float uno = g1dl*g2dn*g1dl*g2dn - g1dl*g2dn*g2dl*g1dn - g2dl*g1dn*g1dl*g2dn + g2dl*g1dn*g2dl*g1dn;
float due = g2dl*g2dl + g2dn*g2dn;
float tre = g2dl*g1dl + g2dn*g1dn;
float quattro = g1dl*g2dl + g1dn*g2dn;
float cinque  = g1dl*g1dl + g1dn*g1dn;


double a11 = (g1dl*due)/uno - (g2dl*tre)/uno; 
double a12 = (g2dl*cinque)/uno - (g1dl*quattro)/uno;
double a21 = (g1dn*due)/uno - (g2dn*tre)/uno;
double a22 = (g2dn*cinque)/uno - (g1dn*quattro)/uno;


//double a11 =  (1/(g1dl*g2dn-g1dn*g2dl))*g2dn;
//double a12 = -(1/(g1dl*g2dn-g1dn*g2dl))*g1dn;
//double a21 = -(1/(g1dl*g2dn-g1dn*g2dl))*g2dl;
//double a22 =  (1/(g1dl*g2dn-g1dn*g2dl))*g1dl;

float cmd_ail_p = a11*sm_phi + a12*sm_psi;
float cmd_rud_p = a21*sm_phi + a22*sm_psi;


#ifndef MAX_DEFLECTION_AILERON
#define MAX_DEFLECTION_AILERON 0.6
#endif

#ifndef MAX_DEFLECTION_RUDDER
#define MAX_DEFLECTION_RUDDER 0.6
#endif

//float cmd_ail = bound_cmd(cmd_ail_p,MAX_DEFLECTION_AILERON);//rifatti "bound"
//float cmd_rud = bound_cmd(cmd_rud_p,MAX_DEFLECTION_RUDDER);// rifatti "bound"

float cmd_ail = bound_cmd(ephi,MAX_DEFLECTION_AILERON);//rifatti "bound"
float cmd_rud = bound_cmd(epsi,MAX_DEFLECTION_RUDDER);// rifatti "bound"


h_ctl_aileron_setpoint = TRIM_PPRZ(cmd_ail);
h_ctl_rudder_setpoint = TRIM_PPRZ(cmd_rud);

/*char str[256];
strcpy(str,"/home/mguerra/pprzfk/paparazzi/sw/airborne/modules/ftgs/log_ftgs");
int i = 0;
char str2;
sprintf(str2, "%d", i);
char str3 = ".txt";
snprintf(buf, sizeof buf, "%s%s%s", str1, str2, str3);
bool done = false;
while(!done)
{
 log_ftgs = fopen(buf,"w");
 if (log_ftgs==NULL) done= true;
 else {
       //buf="";
       i+=1;
       sprintf(str2, "%d", i);
       snprintf(buf, sizeof buf, "%s%s%s", str1, str2, str3);
      }
}*/

log_ftgs = fopen("/home/mguerra/pprzfk/paparazzi/sw/airborne/modules/ftgs/lateral_ftgs.txt","a");

//fprintf(log_ftgs,"h_ctl_roll_setpoint phi ephi last_ephi phi_ref_d phi_ref_last phi_ref_dd phi_ref_last_d cmd_ail h_ctl_aileron_setpoint \n");
fprintf(log_ftgs,"%f ",f1);//1
fprintf(log_ftgs,"%f ",f2);//2
fprintf(log_ftgs,"%f ",g1dl);//3
fprintf(log_ftgs,"%f ",g1dn);//4
fprintf(log_ftgs,"%f ",g2dl);//5
fprintf(log_ftgs,"%f ",g2dn);//6
fprintf(log_ftgs,"%f ",a11);//7
fprintf(log_ftgs,"%f ",a12);//8
fprintf(log_ftgs,"%f ",a21);//9
fprintf(log_ftgs,"%f ",a22);//10
fprintf(log_ftgs,"%f ",cmd_ail_p);//11
fprintf(log_ftgs,"%f ",cmd_rud_p);//12
//fprintf(log_ftgs,"%" PRId16 , h_ctl_aileron_setpoint);//13
//fprintf(log_ftgs,"%" PRId16 , h_ctl_rudder_setpoint);//14
fprintf(log_ftgs," %d \n", 0);
fclose(log_ftgs);

};

/*
from state.hs
stateGetHorizontalSpeedNorm_f() suppongo sqrtf(vx^2+vz^2) ottimo
stateGetNedToBodyEulers_f() phi theta psi
stateGetBodyRates_f() p q r
stateGetSpeedEnu_f() x y z e se le voglio in body?
*/

//check if you can keep some part of the other code... (ask gautier maybe!)



