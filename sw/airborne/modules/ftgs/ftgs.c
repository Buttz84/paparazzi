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

#include "modules/ftgs/ftgs.h"
#include "std.h"
#include "state.h"
#include <math.h>

/* include common mode and variables definitions */
#include "firmwares/fixedwing/guidance/guidance_common.h"

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
INFO("definito modello aero")


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ftgs(struct transport_tx *trans, struct link_device *dev)
{
  //il messaggio deve stare dentro il telemetry-----.xml
  pprz_msg_send_FTGS(trans, dev, AC_ID,  &repiove);
}
#endif


float repiove = 5.0;

void fts_init(void)
{ 
#ifdef A 
 a=A;
#endif
register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FTGS, send_ftgs);

}

void longitudinal(void)
{

float vx = 0; // Statem(1);
float vz = 0; //Statem(2);
float theta = stateGetNedToBodyEulers_f()->theta;
float q = stateGetBodyRates_f()->q;

// li troviamo dopo i venti
float wx = stateGetHorizontalWindspeed_f()->x;
float wz = stateGetHorizontalWindspeed_f()->y;

float l1=0; //dopo
float l2=0; //dopo

float ct = cosf(theta);
float st = sinf(theta);

//first controlled variable *theta*


float ARCTAN=atan2(((-wz)*ct - wx*st + vz),((-wx)*ct + wz*st + vx)); 
float Radice=sqrt(pow(((-wx)*ct + wz*st + vx),2) + pow((wz*ct + wx*st - vz),2));

float f1 = (1/B)*(0.5*l*S*Rho*(Cm0 + Cma*ARCTAN + (Cmq*l*q)/Radice)*(pow(((-wx)*ct + wz*st + vx),2) + pow((wz*ct + wx*st - vz),2)));

float g1dx = 0;

float g1dm = (1/B)*(Cmdm*l*S*Rho*(0.5*pow(wx,2) + 0.5*pow(wz,2) + (-1.*wx*ct + 1.*wz*st)*vx + 0.5*pow(vx,2) + (-1.*wz*ct - 1.*wx*st)*vz + 0.5*pow(vz,2)));


// secondo controlled variable

float Radice2 = sqrt(1 + pow((wz*ct + wx*st - vz),2)/pow(((-wx)*ct + wz*st + vx),2));
float c2t = cosf(2*theta);
float s2t = sinf(2*theta);
float Radice3 = sqrt(pow(wx,2) + pow(wz,2) + (-2*wx*ct + 2*wz*st)*vx + pow(vx,2) - 2*(wz*ct + wx*st)*vz + pow(vz,2));

float f2 = (-g)*st + (POT*l2)/Radice - q*vz + (1/(m*Radice2))*(0.5*S*Rho*(pow((-wx)*ct + wz*st + vx,2) + pow(wz*ct + wx*st - vz,2))*(-Cx0 - Cxdm*l1 - ki*(Cz0 + Cza*ARCTAN + Czdm*l1 + pow((Czq*l*q)/Radice,2) + ((Cz0 + Cza*ARCTAN + Czdm*l1 + (Czq*l*q)/Radice)*((-wz)*ct - wx*st + vz))/((-wx)*ct + wz*st + vx))));

float g2dx =POT/Radice;



float g2dm =-((1.*S*Rho*(Czdm*Czq*ki*l*q*(1.*pow(wx,3)*ct + 1.*wx*pow(wz,2)*ct - 1.*pow(wx,2)*wz*st - 1.*pow(wz,3)*st + (3.*wx*ct - 3.*wz*st)*pow(vx,2) -pow(vx,3) + (-2.*wx*wz*c2t + (-1.*pow(wx,2) + 1.*pow(wz,2))*s2t)*vz + (1.*wx*ct - 1.*wz*st)*pow(vz,2) + vx*(-2.*pow(wx,2) - 2.*pow(wz,2) + (-1.*pow(wx,2) + 1.*pow(wz,2))*c2t + 2.*wx*wz*s2t + (2.*wz*ct + 2.*wx*st)*vz - 1.*pow(vz,2))) +Radice3*(0.5*Cxdm*pow(wx,3)*ct + 1.*Cz0*Czdm*ki*pow(wx,3)*ct -0.5*Czdm*pow(wx,2)*wz*ct + 0.5*Cxdm*wx*pow(wz,2)*ct + 1.*Cz0*Czdm*ki*wx*pow(wz,2)*ct - 0.5*Czdm*pow(wz,3)*ct + Czdm*Cza*ki*pow(wx,3)*ARCTAN*ct +   1.*Czdm*Cza*ki*wx*pow(wz,2)*ARCTAN*ct - 0.5*Czdm*pow(wx,3)*st -0.5*Cxdm*pow(wx,2)*wz*st - 1.*Cz0*Czdm*ki*pow(wx,2)*wz*st - 0.5*Czdm*wx*pow(wz,2)*st - 0.5*Cxdm*pow(wz,3)*st - 1.*Cz0*Czdm*ki*pow(wz,3)*st - 1.*Czdm*Cza*ki*pow(wx,2)*wz*ARCTAN*st - 1.*Czdm*Cza*ki*pow(wz,3)*ARCTAN*st + (-0.5*Cxdm - 1.*Cz0*Czdm*ki - 1.*Czdm*Cza*ki*ARCTAN)*pow(vx,3) + 1.*Czdm*pow(wx,2)*vz + 1.*Czdm*pow(wz,2)*vz - 0.5*Czdm*pow(wx,2)*c2t*vz - 1.*Cxdm*wx*wz*c2t*vz - 2.*Cz0*Czdm*ki*wx*wz*c2t*vz + 0.5*Czdm*pow(wz,2)*c2t*vz - 2.*Czdm*Cza*ki*wx*wz*ARCTAN*c2t*vz - 0.5*Cxdm*pow(wx,2)*s2t*vz -     1.*Cz0*Czdm*ki*pow(wx,2)*s2t*vz + 1.*Czdm*wx*wz*s2t*vz + 0.5*Cxdm*pow(wz,2)*s2t*vz + 1.*Cz0*Czdm*ki*pow(wz,2)*s2t*vz - 1.*Czdm*Cza*ki*pow(wx,2)*ARCTAN*s2t*vz +  1.*Czdm*Cza*ki*pow(wz,2)*ARCTAN*s2t*vz + 0.5*Cxdm*wx*ct*pow(vz,2) + 1.*Cz0*Czdm*ki*wx*ct*pow(vz,2) - 1.5*Czdm*wz*ct*pow(vz,2) + 1.*Czdm*Cza*ki*wx*ARCTAN*ct*pow(vz,2) - 1.5*Czdm*wx*st*pow(vz,2) - 0.5*Cxdm*wz*st*pow(vz,2) - 1.*Cz0*Czdm*ki*wz*st*pow(vz,2) - 1.*Czdm*Cza*ki*wz*ARCTAN*st*pow(vz,2) + 0.5*Czdm*pow(vz,3) +   pow(vx,2)*(1.5*Cxdm*wx*ct + 3.*Cz0*Czdm*ki*wx*ct - 0.5*Czdm*wz*ct - 0.5*Czdm*wx*st - 1.5*Cxdm*wz*st - 3.*Cz0*Czdm*ki*wz*st + Czdm*Cza*ki*ARCTAN*(3.*wx*ct - 3.*wz*st) + 0.5*Czdm*vz) + vx*(-1.*Cxdm*pow(wx,2) - 2.*Cz0*Czdm*ki*pow(wx,2) - 1.*Cxdm*pow(wz,2) - 2.*Cz0*Czdm*ki*pow(wz,2) - 0.5*Cxdm*pow(wx,2)*c2t - 1.*Cz0*Czdm*ki*pow(wx,2)*c2t + 1.*Czdm*wx*wz*c2t + 0.5*Cxdm*pow(wz,2)*c2t + 1.*Cz0*Czdm*ki*pow(wz,2)*c2t + 0.5*Czdm*pow(wx,2)*s2t + 1.*Cxdm*wx*wz*s2t + 2.*Cz0*Czdm*ki*wx*wz*s2t - 0.5*Czdm*pow(wz,2)*s2t + Czdm*Cza*ki*ARCTAN*(-2.*pow(wx,2) - 2.*pow(wz,2) + (-1.*pow(wx,2) + 1.*pow(wz,2))*c2t + 2.*wx*wz*s2t) + ((-1.*Czdm*wx + 1.*Cxdm*wz + 2.*Cz0*Czdm*ki*wz)*ct + (1.*Cxdm*wx + 2.*Cz0*Czdm*ki*wx + 1.*Czdm*wz)*st + Czdm*Cza*ki*ARCTAN*(2.*wz*ct + 2.*wx*st))*vz + (-0.5*Cxdm - 1.*Cz0*Czdm*ki - 1.*Czdm*Cza*ki*ARCTAN)*pow(vz,2))+ pow(Czdm,2)*ki*l1*(1.*pow(wx,3)*ct + 1.*wx*pow(wz,2)*ct - 1.*pow(wx,2)*wz*st - 1.*pow(wz,3)*st + (3.*wx*ct - 3.*wz*st)*pow(vx,2) - 1.*pow(vx,3) + (-2.*wx*wz*c2t + (-1.*pow(wx,2) + 1.*pow(wz,2))*s2t)*vz + (1.*wx*ct - 1.*wz*st)*pow(vz,2) + vx*-2.*pow(wx,2) - 2.*pow(wz,2) + (-1.*pow(wx,2) + 1.*pow(wz,2))*c2t + 2.*wx*wz*s2t + (2.*wz*ct + 2.*wx*st)*vz - 1.*pow(vz,2))))))/(m*(wx*ct - 1.*wz*st - 1.*vx)*Radice*Radice2);


float d2dm = 0;

/* eventual third output - rate control
%% Third output parts, y=q, y_3^(1)=f3+g3dx*dx+g3dm*dm+d3dm

f3=(1/B)*(0.5*l*S*Rho*(Cm0 + Cma*ARCTAN + ...
    (Cmq*l*q)/Radice)*(((-wx)*ct + wz*st + vx)^2 + ...
    (wz*ct + wx*st - vz)^2));


g3dx=0;

g3dm=(1/B)*(Cmdm*l*S*Rho*(0.5*wx^2 + 0.5*wz^2 + (-1.*wx*ct + 1.*wz*st)*vx + 0.5*vx^2 + (-1.*wz*ct - 1.*wx*st)*vz + 0.5*vz^2));

d3dm=0;
*/

float eth = theta; //-theta_reference 
float evx = vx; //-vx_reference

float edth = 0;//last - edt

float sm_th = -k_th*sign(eth+edth)*pow(absf(eth+edth),3/4)-f1*theta_ref_dd;
float sm_vx = -k_vx*sign(evx)*pow(absf(evx),3/4)-f2*vx_ref_d;


float a11 =  (1/(g1dm*g2dx-g1dx*g2dm))*g2dx;
float a12 = -(1/(g1dm*g2dx-g1dx*g2dm))*g1dx;
float a21 =  (1/(g1dm*g2dx-g1dx*g2dm))*g2dm;
float a22 = -(1/(g1dm*g2dx-g1dx*g2dm))*g1dm;

float cmd_elev = a11*sm_th + a12*sm_vx;
float cmd_thrust = a21*sm_th + a22*sm_vx;

cmd_elev = max(cmd_elev,MAX_DEFLECTION_ELEVATOR);
cmd_thrust = Bound(cmd_thrust,0,1); 


}

void lateral(void)
{

float vy = Statem(1);
float phi = stateGetNedToBodyEulers_f()->phi;
float psi = stateGetNedToBodyEulers_f()->psi;
float p = stateGetBodyRates_f()->p;
float r = stateGetBodyRates_f()->r;

float wy = stateGetverticalWindspeed_f();

float l1 = 0;//L(2);
float l2 = 0;//L(3);

float cph=cosf(phi);
float sph=sinf(phi);
float cps=cosf(psi);
float sps=sinf(psi);

float Va = stategetAirspeed_F();

// First output parts, y=phi, y_1^(2)=f1+g1dx*dx+g1dm*dm+d1dm
float ArcSin = asin(((-wy)*(cph*cps + vy)/Va));


float f1 = (1/(A*CC - 1.*pow(EE,2)))*(l*S*Va*Rho*((0.5*CC*clb + 0.5*cnb*EE)*Va*ArcSin + (0.5*CC*clp*l + 0.5*cnp*EE*l)*p + (0.5*CC*clr*l + 0.5*cnr*EE*l)*r));

float g1dl = ((0.5*CC*cldl + 0.5*cndl*EE)*l*S*pow(Va,2)*Rho)/(A*CC - 1.*pow(EE,2));

float g1dn = ((0.5*CC*cldn + 0.5*cndn*EE)*l*S*pow(Va,2)*Rho)/(A*CC - 1.*pow(EE,2));


// Second output parts, y=psi, y_2^(2)=f2+g2dx*dx+g2dm*dm+d2dm


float f2 = (1/(A*CC - 1.*pow(EE,2)))*(l*S*Va*Rho*cph*((0.5*A*cnb + 0.5*clb*EE)*Va*ArcSin + (0.5*A*cnr*l + 0.5*clr*EE*l)*r) + p*((0.5*A*cnp + 0.5*clp*EE)*pow(l,2)*S*Va*Rho*cph + (-1.*A*CC + pow(EE,2))*r*sph));

float g2dl = ((0.5*A*cndl + 0.5*cldl*EE)*l*S*pow(Va,2)*Rho*cph)/(A*CC - 1.*pow(EE,2));

float g2dn = ((0.5*A*cndn + 0.5*cldn*EE)*l*S*pow(Va22)*Rho*cph)/(A*CC - 1.*pow(EE,2));


// NDI

float ephi = phi; //-phi_reference 
float epsi = psi; //-psi_reference

float edphi = 0;//last - edphi
float edpsi = 0;//last - edpsi

float sm_phi = -k_phi*sign(ephi+edphi)*pow(absf(ephi+edphi),3/4)-f1*phi_ref_dd;
float sm_psi = -k_psi*sign(epsi+edpsi)*pow(absf(epsi+edpsi),3/4)-f2*psi_ref_dd;


float a11 =  (1/(g1dl*g2dn-g1dn*g2dl))*g2dn;
float a12 = -(1/(g1dl*g2dn-g1dn*g2dl))*g1dn;
float a21 =  (1/(g1dl*g2dn-g1dn*g2dl))*g2dl;
float a22 = -(1/(g1dl*g2dn-g1dn*g2dl))*g1dl;

float cmd_ail = a11*sm_phi + a12*sm_psi;
float cmd_rud = a21*sm_phi + a22*sm_psi;

cmd_ail = max(cmd_elev,MAX_DEFLECTION_AILERON);
cmd_rud = max(cmd_elev,MAX_DEFLECTION_RUDDER);

}

/*
from state.h
stateGetHorizontalSpeedNorm_f() suppongo sqrt(vx^2+vz^2) ottimo
stateGetNedToBodyEulers_f() phi theta psi
stateGetBodyRates_f() p q r
stateGetSpeedEnu_f() x y z e se le voglio in body?
*/

//check if you can keep some part of the other code... (ask gautier maybe!)



