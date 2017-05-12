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

float POT = 120.0;

int a;
pprz_t h_ctl_aileron_setpoint;
pprz_t h_ctl_rudder_setpoint;
pprz_t h_ctl_elevator_setpoint;

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
  pprz_msg_send_FTGS(trans, dev, AC_ID,  &repiove);
}

#endif

float repiove =7.0;
bool h_ctl_disabled;

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
 
 vBx = (mat->m[1*1])*vN + (mat->m[1*2])*vE + (mat->m[1*3])*vD; 
 vBy = (mat->m[2*1])*vN + (mat->m[2*2])*vE + (mat->m[2*3])*vD; 
 vBz = (mat->m[3*1])*vN + (mat->m[3*2])*vE + (mat->m[3*3])*vD;
 
}

float  bound_cmd(float comm, float lim)
{
float cmd;
if (comm>lim) cmd = lim;
else {
      if (comm<-lim) cmd = -lim;
      else cmd = comm;
	 }
cmd = cmd*12000;	 
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

float ct = cosf(theta);
float st = sinf(theta);

NEDtoBody();
float vx = vBx; // Statem(1);
float vz = vBz; //Statem(2);

//first controlled variable *theta*


float ARCTAN=atan2(((-wz)*ct - wx*st + vz),((-wx)*ct + wz*st + vx)); 
float Radice=sqrt(pow(((-wx)*ct + wz*st + vx),2) + pow((wz*ct + wx*st - vz),2));

float f1 = (1/B)*(0.5*l*S*Rho*(Cm0 + Cma*ARCTAN + (Cmq*l*q)/Radice)*(pow(((-wx)*ct + wz*st + vx),2) + pow((wz*ct + wx*st - vz),2)));

float g1dx = 0;

float g1dm = (1/B)*(Cmdm*l*S*Rho*(0.5*pow(wx,2) + 0.5*pow(wz,2) + (-1.*wx*ct + 1.*wz*st)*vx + 0.5*pow(vx,2) + (-1.*wz*ct - 1.*wx*st)*vz + 0.5*pow(vz,2)));


// second controlled variable

float Radice2 = sqrt(1 + pow((wz*ct + wx*st - vz),2)/pow(((-wx)*ct + wz*st + vx),2));
float c2t = cosf(2*theta);
float s2t = sinf(2*theta);
float Radice3 = sqrt(pow(wx,2) + pow(wz,2) + (-2*wx*ct + 2*wz*st)*vx + pow(vx,2) - 2*(wz*ct + wx*st)*vz + pow(vz,2));

float f2 = (-g)*st + (POT*l2)/Radice - q*vz + (1/(m*Radice2))*(0.5*S*Rho*(pow((-wx)*ct + wz*st + vx,2) + pow(wz*ct + wx*st - vz,2))*(-Cx0 - Cxdm*l1 - ki*(Cz0 + Cza*ARCTAN + Czdm*l1 + pow((Czq*l*q)/Radice,2) + ((Cz0 + Cza*ARCTAN + Czdm*l1 + (Czq*l*q)/Radice)*((-wz)*ct - wx*st + vz))/((-wx)*ct + wz*st + vx))));

float g2dx =POT/Radice;



float g2dm =-((1.*S*Rho*(Czdm*Czq*ki*l*q*(1.*pow(wx,3)*ct + 1.*wx*pow(wz,2)*ct - 1.*pow(wx,2)*wz*st - 1.*pow(wz,3)*st + (3.*wx*ct - 3.*wz*st)*pow(vx,2) -pow(vx,3) + (-2.*wx*wz*c2t + (-1.*pow(wx,2) + 1.*pow(wz,2))*s2t)*vz + (1.*wx*ct - 1.*wz*st)*pow(vz,2) + vx*(-2.*pow(wx,2) - 2.*pow(wz,2) + (-1.*pow(wx,2) + 1.*pow(wz,2))*c2t + 2.*wx*wz*s2t + (2.*wz*ct + 2.*wx*st)*vz - 1.*pow(vz,2))) +Radice3*(0.5*Cxdm*pow(wx,3)*ct + 1.*Cz0*Czdm*ki*pow(wx,3)*ct -0.5*Czdm*pow(wx,2)*wz*ct + 0.5*Cxdm*wx*pow(wz,2)*ct + 1.*Cz0*Czdm*ki*wx*pow(wz,2)*ct - 0.5*Czdm*pow(wz,3)*ct + Czdm*Cza*ki*pow(wx,3)*ARCTAN*ct +   1.*Czdm*Cza*ki*wx*pow(wz,2)*ARCTAN*ct - 0.5*Czdm*pow(wx,3)*st -0.5*Cxdm*pow(wx,2)*wz*st - 1.*Cz0*Czdm*ki*pow(wx,2)*wz*st - 0.5*Czdm*wx*pow(wz,2)*st - 0.5*Cxdm*pow(wz,3)*st - 1.*Cz0*Czdm*ki*pow(wz,3)*st - 1.*Czdm*Cza*ki*pow(wx,2)*wz*ARCTAN*st - 1.*Czdm*Cza*ki*pow(wz,3)*ARCTAN*st + (-0.5*Cxdm - 1.*Cz0*Czdm*ki - 1.*Czdm*Cza*ki*ARCTAN)*pow(vx,3) + 1.*Czdm*pow(wx,2)*vz + 1.*Czdm*pow(wz,2)*vz - 0.5*Czdm*pow(wx,2)*c2t*vz - 1.*Cxdm*wx*wz*c2t*vz - 2.*Cz0*Czdm*ki*wx*wz*c2t*vz + 0.5*Czdm*pow(wz,2)*c2t*vz - 2.*Czdm*Cza*ki*wx*wz*ARCTAN*c2t*vz - 0.5*Cxdm*pow(wx,2)*s2t*vz -     1.*Cz0*Czdm*ki*pow(wx,2)*s2t*vz + 1.*Czdm*wx*wz*s2t*vz + 0.5*Cxdm*pow(wz,2)*s2t*vz + 1.*Cz0*Czdm*ki*pow(wz,2)*s2t*vz - 1.*Czdm*Cza*ki*pow(wx,2)*ARCTAN*s2t*vz +  1.*Czdm*Cza*ki*pow(wz,2)*ARCTAN*s2t*vz + 0.5*Cxdm*wx*ct*pow(vz,2) + 1.*Cz0*Czdm*ki*wx*ct*pow(vz,2) - 1.5*Czdm*wz*ct*pow(vz,2) + 1.*Czdm*Cza*ki*wx*ARCTAN*ct*pow(vz,2) - 1.5*Czdm*wx*st*pow(vz,2) - 0.5*Cxdm*wz*st*pow(vz,2) - 1.*Cz0*Czdm*ki*wz*st*pow(vz,2) - 1.*Czdm*Cza*ki*wz*ARCTAN*st*pow(vz,2) + 0.5*Czdm*pow(vz,3) +   pow(vx,2)*(1.5*Cxdm*wx*ct + 3.*Cz0*Czdm*ki*wx*ct - 0.5*Czdm*wz*ct - 0.5*Czdm*wx*st - 1.5*Cxdm*wz*st - 3.*Cz0*Czdm*ki*wz*st + Czdm*Cza*ki*ARCTAN*(3.*wx*ct - 3.*wz*st) + 0.5*Czdm*vz) + vx*(-1.*Cxdm*pow(wx,2) - 2.*Cz0*Czdm*ki*pow(wx,2) - 1.*Cxdm*pow(wz,2) - 2.*Cz0*Czdm*ki*pow(wz,2) - 0.5*Cxdm*pow(wx,2)*c2t - 1.*Cz0*Czdm*ki*pow(wx,2)*c2t + 1.*Czdm*wx*wz*c2t + 0.5*Cxdm*pow(wz,2)*c2t + 1.*Cz0*Czdm*ki*pow(wz,2)*c2t + 0.5*Czdm*pow(wx,2)*s2t + 1.*Cxdm*wx*wz*s2t + 2.*Cz0*Czdm*ki*wx*wz*s2t - 0.5*Czdm*pow(wz,2)*s2t + Czdm*Cza*ki*ARCTAN*(-2.*pow(wx,2) - 2.*pow(wz,2) + (-1.*pow(wx,2) + 1.*pow(wz,2))*c2t + 2.*wx*wz*s2t) + ((-1.*Czdm*wx + 1.*Cxdm*wz + 2.*Cz0*Czdm*ki*wz)*ct + (1.*Cxdm*wx + 2.*Cz0*Czdm*ki*wx + 1.*Czdm*wz)*st + Czdm*Cza*ki*ARCTAN*(2.*wz*ct + 2.*wx*st))*vz + (-0.5*Cxdm - 1.*Cz0*Czdm*ki - 1.*Czdm*Cza*ki*ARCTAN)*pow(vz,2))+ pow(Czdm,2)*ki*l1*(1.*pow(wx,3)*ct + 1.*wx*pow(wz,2)*ct - 1.*pow(wx,2)*wz*st - 1.*pow(wz,3)*st + (3.*wx*ct - 3.*wz*st)*pow(vx,2) - 1.*pow(vx,3) + (-2.*wx*wz*c2t + (-1.*pow(wx,2) + 1.*pow(wz,2))*s2t)*vz + (1.*wx*ct - 1.*wz*st)*pow(vz,2) + vx*-2.*pow(wx,2) - 2.*pow(wz,2) + (-1.*pow(wx,2) + 1.*pow(wz,2))*c2t + 2.*wx*wz*s2t + (2.*wz*ct + 2.*wx*st)*vz - 1.*pow(vz,2))))))/(m*(wx*ct - 1.*wz*st - 1.*vx)*Radice*Radice2);


/* eventual third output - rate control
%% Third output parts, y=q, y_3^(1)=f3+g3dx*dx+g3dm*dm+d3dm

f3=(1/B)*(0.5*l*S*Rho*(Cm0 + Cma*ARCTAN + ...
    (Cmq*l*q)/Radice)*(((-wx)*ct + wz*st + vx)^2 + ...
    (wz*ct + wx*st - vz)^2));


g3dx=0;

g3dm=(1/B)*(Cmdm*l*S*Rho*(0.5*wx^2 + 0.5*wz^2 + (-1.*wx*ct + 1.*wz*st)*vx + 0.5*vx^2 + (-1.*wz*ct - 1.*wx*st)*vz + 0.5*vz^2));

d3dm=0;
*/



float eth = theta - h_ctl_pitch_setpoint; 

float cpsi = cosf(franco->psi);

float evx = (cpsi*stateGetHorizontalSpeedNorm_f())-15;; //-vx_reference

float edth = eth-last_eth;//last - edphi
last_eth = eth;//last - edt

float k_th = 5;
float k_vx = 5;


float theta_ref_d = h_ctl_pitch_setpoint - theta_ref_last; 
theta_ref_last = h_ctl_pitch_setpoint;

float theta_ref_dd = theta_ref_d - theta_ref_last_d;
theta_ref_last_d = theta_ref_d;

float vx_ref_d = 0;




float sm_th = -k_th*sign(eth+edth)*pow(fabs(eth+edth),3/4)-f1*theta_ref_dd;
float sm_vx = -k_vx*sign(evx)*pow(fabs(evx),3/4)-f2*vx_ref_d;


float a11 =  (1/(g1dm*g2dx-g1dx*g2dm))*g2dx;
float a12 = -(1/(g1dm*g2dx-g1dx*g2dm))*g1dx;
float a21 = -(1/(g1dm*g2dx-g1dx*g2dm))*g2dm;
float a22 =  (1/(g1dm*g2dx-g1dx*g2dm))*g1dm;

float cmd_elev_p = a11*sm_th + a12*sm_vx;
float cmd_thrust_p = a21*sm_th + a22*sm_vx;

#ifndef MAX_DEFLECTION_ELEVATOR 
#define MAX_DEFLECTION_ELEVATOR 0.6
#endif

eth = -5*eth;
float cmd_elev = bound_cmd(eth,MAX_DEFLECTION_ELEVATOR);

//float cmd_elev = bound_cmd(cmd_elev_p,MAX_DEFLECTION_ELEVATOR);
float cmd_thrust = bound_thr(cmd_thrust_p);


//float cmd_elev = bound_cmd(eth,MAX_DEFLECTION_ELEVATOR);

h_ctl_elevator_setpoint = TRIM_PPRZ(cmd_elev);

FILE *log_ftgs;
log_ftgs = fopen("/home/mguerra/pprzfk/paparazzi/sw/airborne/modules/ftgs/longi_ftgs.txt","a");

fprintf(log_ftgs,"%f ",f1);//1
fprintf(log_ftgs,"%f ",f2);//2
fprintf(log_ftgs,"%f ",g1dx);//3
fprintf(log_ftgs,"%f ",g1dm);//4
fprintf(log_ftgs,"%f ",g2dx);//5
fprintf(log_ftgs,"%f ",g2dm);//6
fprintf(log_ftgs,"%f ",a11);//7
fprintf(log_ftgs,"%f ",a12);//8
fprintf(log_ftgs,"%f ",a21);//9
fprintf(log_ftgs,"%f ",a22);//10
fprintf(log_ftgs,"%f ",cmd_elev_p);//11
fprintf(log_ftgs,"%f ",cmd_thrust_p);//12
fprintf(log_ftgs,"%f ",cmd_elev);//13
fprintf(log_ftgs,"%f ",cmd_thrust);//14
fprintf(log_ftgs,"%f ",vBx);//15
fprintf(log_ftgs,"%f ",vBz);//16
fprintf(log_ftgs," %d \n", 0);
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


float f1 = (1/(A*CC - 1.*pow(EE,2)))*(l*S*Va*Rho*((0.5*CC*clb + 0.5*cnb*EE)*Va*ArcSin + (0.5*CC*clp*l + 0.5*cnp*EE*l)*p + (0.5*CC*clr*l + 0.5*cnr*EE*l)*r));

double g1dl = ((0.5*CC*cldl + 0.5*cndl*EE)*l*S*pow(Va,2)*Rho)/(A*CC - 1.*pow(EE,2));

double g1dn = ((0.5*CC*cldn + 0.5*cndn*EE)*l*S*pow(Va,2)*Rho)/(A*CC - 1.*pow(EE,2));


// Second output parts, y=psi, y_2^(2)=f2+g2dx*dx+g2dm*dm+d2dm


float f2 = (1/(A*CC - 1.*pow(EE,2)))*(l*S*Va*Rho*cph*((0.5*A*cnb + 0.5*clb*EE)*Va*ArcSin + (0.5*A*cnr*l + 0.5*clr*EE*l)*r) + p*((0.5*A*cnp + 0.5*clp*EE)*pow(l,2)*S*Va*Rho*cph + (-1.*A*CC + pow(EE,2))*r*sph));

double g2dl = ((0.5*A*cndl + 0.5*cldl*EE)*l*S*pow(Va,2)*Rho*cph)/(A*CC - 1.*pow(EE,2));

double g2dn = ((0.5*A*cndn + 0.5*cldn*EE)*l*S*pow(Va,2)*Rho*cph)/(A*CC - 1.*pow(EE,2));


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

float sm_phi = -k_phi*sign(sign1)*pow(fabs(sign1),3/4)-f1*phi_ref_dd;
float sm_psi = -k_psi*sign(sign2)*pow(fabs(sign2),3/4)-f2*psi_ref_dd;





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
from state.h
stateGetHorizontalSpeedNorm_f() suppongo sqrt(vx^2+vz^2) ottimo
stateGetNedToBodyEulers_f() phi theta psi
stateGetBodyRates_f() p q r
stateGetSpeedEnu_f() x y z e se le voglio in body?
*/

//check if you can keep some part of the other code... (ask gautier maybe!)



