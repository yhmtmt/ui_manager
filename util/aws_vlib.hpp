// Copyright(c) 2015-2019 Yohei Matsumoto, All right reserved. 

// aws_vlib.hpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_vlib.hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_vlib.hpp.  If not, see <http://www.gnu.org/licenses/>. 
#ifndef AWS_VLIB_HPP
#define AWS_VLIB_HPP
#include <opencv2/opencv.hpp>

enum e_imfmt{
  IMF_GRAY8, IMF_GRAY10, IMF_GRAY12, IMF_GRAY14, IMF_GRAY16,
  IMF_RGB8, IMF_RGB10, IMF_RGB12, IMF_RGB14, IMF_RGB16,
  IMF_BGR8, IMF_BGR10, IMF_BGR12, IMF_BGR14, IMF_BGR16,
  IMF_BayerBG8, IMF_BayerGB8, IMF_BayerGR8, IMF_BayerRG8,
  IMF_BayerBG10, IMF_BayerGB10, IMF_BayerGR10, IMF_BayerRG10,
  IMF_BayerBG12, IMF_BayerGB12, IMF_BayerGR12, IMF_BayerRG12,
  IMF_NV12, IMF_I420, IMF_Undef
};

extern const char* str_imfmt[IMF_Undef];

inline double rerr(double a, double b){
  return fabs((a - b) / max(fabs(b), (double)1e-12));
}


/////////////////////////////////////////////////////////////////////// extracting Euler angles from Rotation matrix
// these codes are partially from OpenCV calibrate.cpp
// angleRxyz decompose it assuming R=RxRyRz
void angleRxyz(double * R, double & x, double & y, double &z);
// angleRzyx decompose it assuming R=RzRyRx
void angleRzyx(double * R, double & x, double & y, double &z);

////////////////////////////////////////////////////////////////////////// Lie-group to Lie-algebra mapping function
// SO(3)->so(3) log Rodrigues gives this mapping
inline void log_so3(const double * R, double * r)
{
  double rx, ry, rz, s, c, theta;

  // Note rx, ry, rz are often used as temporal variable. Be careful.
  // here calculating
  // 2s * rx     R32 - R23
  // 2s * ry  =  R13 - R31
  // 2s * rz     R21 - R12
  rx = R[7] - R[5];
  ry = R[2] - R[6];
  rz = R[3] - R[1];

  // s^2 = sqrt((2s * rx)^2 + (2s * ry)^2 + (2s * rz)^2)/2
  // note: here the sign of the sin is missed. But we dont need to distinguish it.
  s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);

  // trace(R)-1 gives 2cos(theta)
  c = (R[0] + R[4] + R[8] - 1)*0.5;

  // clamping cosine in (-1,1)
  c = c > 1. ? 1. : c < -1. ? -1. : c;
  theta = acos(c);

  if( s < 1e-5 )
    {
      if( c > 0 ) // theta -> 0
	rx = ry = rz = 0;
      else // theta -> PI
	{
	  // This relys on quarternion.

	  // SO(3) to SU(2)
	  // Quarternion |q| = (q0, q1, q2, q3)
	  // Here I assume sgn(q0) is positive
	  // q0 = sqrt(0.25(R11+R22+R33+1))=sqrt(0.25*(2+2cos th)=sqrt(cos^2th/2)
	  // q1 = sqrt(0.25(R11-R22-R33+1)) sgn(R32-R23) = sqrt(R11-cos(th))sgn(R32-R23) = sin th/2 rx sgn(q0q1)
	  // q2 = sqrt(0.25(-R11+R22-R33+1)) sgn(R13-R31) = sqrt(R22-cos(th))sgn(R13-R31) = sin th/2 ry sgn(q0q2)
	  // q3 = sqrt(0.25(-R11-R22+R33+1)) sgn(R21-R12) = sqrt(R33-cos(th))sgn(R21-R12) = sin th/2 rz sgn(q0q3)

	  // SU(2) to so(3)
	  // q = cos th/2 + n sin th/2 ... where n is unit vector
	  // cos th/2 = q0
	  // sin th/2 = |(q1,q2, q3)|
	  // r = (q1,q2,q3) / (sin th/2)

	  /* These are the OpenCV's Rodrigues. It may be coded assuming quarternion, but I think it's wrong.
	     t = (R[0] + 1)*0.5;  
	     rx = sqrt(MAX(t,0.));
	     t = (R[4] + 1)*0.5; 
	     ry = sqrt(MAX(t,0.))*(R[1] < 0 ? -1. : 1.);
	     t = (R[8] + 1)*0.5; 
	     rz = sqrt(MAX(t,0.))*(R[2] < 0 ? -1. : 1.);
	     if( fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R[5] > 0) != (ry*rz > 0) )
	     rz = -rz;
	  */		
	  // calculatign q1,q2,q3, we do not need to calculate q0 because theta has already been calculated as theta.
	  // at this line, rx = R32-R23, ry = R13-R31, rz = R21-R12, calculated in the begining
	  // their sign is just those of resulting q1, q2, q3 respectively!
	  // Furthermore, the cosine value has also been calculated as c.
	  rx = sqrt(max(R[0] - c, 0.)) * (rx < 0 ? -1.: 1.); // q1 
	  ry = sqrt(max(R[4] - c, 0.)) * (ry < 0 ? -1.: 1.); // q2
	  rz = sqrt(max(R[8] - c, 0.)) * (rz < 0 ? -1. :1.); // q3 

	  theta /= sqrt(rx*rx + ry*ry + rz*rz); // sin th/2
	  rx *= theta; // * theta / sin th/2 => theta rx
	  ry *= theta; // * theta / sin th/2 => theta ry
	  rz *= theta; // * theta / sin th/2 => theta rz
	}
    }
  else
    {
      double vth = 1/(2*s);
      vth *= theta;
      rx *= vth; ry *= vth; rz *= vth;
    }
}

inline void exp_so3(const double * r, 
		    double * R /* 3x3 matrix */, 
		    double * J/* 9x3 matrix */)
{
  double rx = r[0], ry = r[1], rz = r[2];
  double theta = sqrt(rx*rx + ry*ry + rz*rz);
  if(theta < DBL_EPSILON){
    memset((void*) R, 0, sizeof(double) * 9);
    memset((void*) J, 0, sizeof(double) * 27);
    R[0] = R[4] = R[8] = 1.0;
    if(J ==NULL)
      return;

    J[5] = J[15] = J[19] = -1.0;
    J[7] = J[11] = J[21] = 1.0;
    return;
  }

  double itheta = 1.0 / theta;
  rx *= itheta;
  ry *= itheta;
  rz *= itheta;

  double rx2 = rx * rx, ry2 = ry * ry, rz2 = rz * rz, 
    rxry = rx * ry, rxrz = rx * rz, ryrz = ry * rz;

  // skew symmetric matrix A
  // 0   -rz   ry
  // rz    0  -rx
  // -ry  rx    0

  // A^2
  // -rz2-ry2 rxry     rxrz
  // rxry     -rx2-rz2 ryrz
  // rxrz     ryrz     -rx2-ry2
  //
  // rx2+ry2+rz2=1.0 simplify A^2 as
  // rx2-1    rxry     rxrz
  // rxry     ry2-1    ryrz
  // rxrz     ryrz     rz2-1

  // Resulting matrix R
  // R0  R1  R2 
  // R3  R4  R5
  // R6  R7  R8

  //
  // R=I + s A + (1-c) A^2
  // ic rx2 + c      ic rxry - s rz  ic rxrz + s ry
  // ic rxry + s rz  ic ry2 + c      ic ryrz - s rx
  // ic rxrz - s ry  ic ryrz + s rx  ic rz2 + c
  double s = sin(theta);
  double c = cos(theta);
  double ic = 1.0 - c;
  double icrxry = ic * rxry, icryrz = ic * ryrz, icrxrz = ic * rxrz;
  double srx = s * rx, sry = s * ry, srz = s * rz;
  R[0] = ic * rx2 + c; R[1] = icrxry - srz; R[2] = icrxrz + sry;
  R[3] = icrxry + srz; R[4] = ic * ry2 + c; R[5] = icryrz - srx;
  R[6] = icrxrz - sry; R[7] = icryrz + srx; R[8] = ic * rz2 +  c;

  if(J == NULL)
    return;
  double rx3 = rx2 * rx, ry3 = ry2 * ry, rz3 = rz2 * rz;
  double dC1drx, dC2drx, dC3drx;
  double dC1dry, dC2dry, dC3dry;	
  double dC1drz, dC2drz, dC3drz;
  double dS1drx, dS2drx, dS3drx;
  double dS1dry, dS2dry, dS3dry;
  double dS1drz, dS2drz, dS3drz;

  double icith = ic * itheta;
  double sith = s * itheta;
  double sm2icith = (s - 2 * ic * itheta);
  double cmsith = (c - s * itheta);
  double irx2 = 1. - rx2, iry2 = 1. - ry2, irz2 = 1. - rz2;

  dC1drx = rx3 * s + 2 * irx2 * rx * icith;
  dC1dry = ry * rx2 * sm2icith;
  dC1drz = rz * rx2 * sm2icith;
  dS1drx = rx2 * c + (1. - rx2) * sith;
  dS1dry = rxry * cmsith;
  dS1drz = rxrz * cmsith;

  dC2drx = rx * ry2 * sm2icith;
  dC2dry = ry3 * s + 2 * iry2 * ry * icith;
  dC2drz = rz * ry2 * sm2icith;
  dS2drx = dS1dry;
  dS2dry = ry2 * c + (1. - ry2) * sith;
  dS2drz = ryrz * cmsith;

  dC3drx = rx * rz2 * sm2icith;
  dC3dry = ry * rz2 * sm2icith;
  dC3drz = rz3 * s + 2 * irz2 * rz * icith;
  dS3drx = dS1drz;
  dS3dry = dS2drz;
  dS3drz = rz2 * c + (1. - rz2) * sith;

  double dC12drx, dC13drx, dC23drx;
  double dC12dry, dC13dry, dC23dry;
  double dC12drz, dC13drz, dC23drz;
  /*
    double 
    irxrx2icith = (irx2 - rx) * icith, 
    iryry2icith = (iry2 - ry) * icith, 
    irzrz2icith = (irz2 - rz) * icith;
  */
  dC12dry = rx * (ry2 * sm2icith + icith);
  dC13drz = rx * (rz2 * sm2icith + icith);
  dC12drx = ry * (rx2 * sm2icith + icith);
  dC23drz = ry * (rz2 * sm2icith + icith);
  dC13drx = rz * (rx2 * sm2icith + icith);
  dC23dry = rz * (ry2 * sm2icith + icith);
  dC23drx = dC13dry = dC12drz = rxry * rz * sm2icith;

  J[0]  = dC1drx - srx;     J[1]  = dC1dry - sry;     J[2] = dC1drz - srz;
  J[3]  = dC12drx - dS3drx; J[4]  = dC12dry - dS3dry; J[5] = dC12drz - dS3drz;
  J[6]  = dC13drx + dS2drx; J[7]  = dC13dry + dS2dry; J[8] = dC13drz + dS2drz;
  J[9]  = dC12drx + dS3drx; J[10] = dC12dry + dS3dry; J[11] = dC12drz + dS3drz;
  J[12] = dC2drx -srx;      J[13] = dC2dry -sry;      J[14] = dC2drz -srz;
  J[15] = dC23drx - dS1drx; J[16] = dC23dry - dS1dry; J[17] = dC23drz - dS1drz;
  J[18] = dC13drx - dS2drx; J[19] = dC13dry - dS2dry; J[20] = dC13drz - dS2drz;
  J[21] = dC23drx + dS1drx; J[22] = dC23dry + dS1dry; J[23] = dC23drz + dS1drz;
  J[24] = dC3drx - srx;     J[25] = dC3dry - sry;     J[26] = dC3drz - srz;
}

bool test_exp_so3(const double * r, double * R, cv::Mat & jR);

// so(3)->SO(3) exp Rodrigues gives this mapping
inline void exp_so3(const double * r, double * R)
{
  double rx = r[0], ry = r[1], rz = r[2];
  double theta = sqrt(rx*rx + ry*ry + rz*rz);
  if(theta < DBL_EPSILON){
    memset((void*) R, 0, sizeof(double) * 9);
    R[0] = R[4] = R[8] = 1.0;
    return;
  }

  double itheta = 1.0 / theta;

  rx *= itheta;
  ry *= itheta;
  rz *= itheta;

  double rx2 = rx * rx, ry2 = ry * ry, rz2 = rz * rz, 
    rxry = rx * ry, rxrz = rx * rz, ryrz = ry * rz;

  // skew symmetric matrix A
  // 0   -rz   ry
  // rz    0  -rx
  // -ry  rx    0

  // A^2
  // -rz2-ry2 rxry     rxrz
  // rxry     -rx2-rz2 ryrz
  // rxrz     ryrz     -rx2-ry2
  //
  // rx2+ry2+rz2=1.0 simplify A^2 as
  // rx2-1    rxry     rxrz
  // rxry     ry2-1    ryrz
  // rxrz     ryrz     rz2-1

  // Resulting matrix R
  // R0  R1  R2 
  // R3  R4  R5
  // R6  R7  R8

  //
  // R=I + s A + (1-c) A^2
  // ic rx2 + c      ic rxry - s rz  ic rxrz + s ry
  // ic rxry + s rz  ic ry2 + c      ic ryrz - s rx
  // ic rxrz - s ry  ic ryrz + s rx  ic rz2 + c
  double s = sin(theta);
  double c = cos(theta);
  double ic = 1.0 - c;
  double icrxry = ic * rxry, icryrz = ic * ryrz, icrxrz = ic * rxrz;
  double srx = s * rx, sry = s * ry, srz = s * rz;
  R[0] = ic * rx2 + c; R[1] = icrxry - srz; R[2] = icrxrz + sry;
  R[3] = icrxry + srz; R[4] = ic * ry2 + c; R[5] = icryrz - srx;
  R[6] = icrxrz - sry; R[7] = icryrz + srx; R[8] = ic * rz2 +  c;
}

// SE(3)->se(3) log 
inline void log_se3(const double * T, double * r, double * v)
{
  double rx, ry, rz, s, c, theta;

  // T0  T1  T2  T3
  // T4  T5  T6  T7
  // T8  T9  T10 T11
  // T12 T13 T14 T15

  // Note rx, ry, rz are often used as temporal variable. Be careful.
  // here calculating
  // 2s * rx     R32 - R23
  // 2s * ry  =  R13 - R31
  // 2s * rz     R21 - R12
  rx = T[9] - T[6];
  ry = T[2] - T[8];
  rz = T[4] - T[1];

  // s^2 = sqrt((2s * rx)^2 + (2s * ry)^2 + (2s * rz)^2)/2
  s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);

  // trace(R)-1 gives 2cos(theta)
  c = (T[0] + T[5] + T[10] - 1)*0.5;

  // clamping cosine in (-1,1)
  c = c > 1. ? 1. : c < -1. ? -1. : c;
  theta = acos(c);

  if( s < 1e-5 )
    {
      if( c > 0 ) // theta -> 0
	rx = ry = rz = 0;
      else // theta -> PI
	{
	  rx = sqrt(max(T[0] - c, 0.)) * (rx < 0 ? -1.: 1.); // q1 
	  ry = sqrt(max(T[5] - c, 0.)) * (ry < 0 ? -1.: 1.); // q2
	  rz = sqrt(max(T[10] - c, 0.)) * (rz < 0 ? -1. :1.); // q3 

	  double vth = 1.0 / sqrt(rx*rx + ry*ry + rz*rz);
	  rx *= vth; // / sin th/2 => rx
	  ry *= vth; // / sin th/2 => ry
	  rz *= vth; // / sin th/2 => rz
	}
    }
  else
    {
      double vth = 1/(2*s);
      rx *= vth; ry *= vth; rz *= vth;
    }

  double rx2 = rx * rx, ry2 = ry * ry, rz2 = rz * rz, 
    rxry = rx * ry, rxrz = rx * rz, ryrz = ry * rz;

  double itheta = 1.0 / theta;
  double ic = 1.0 - c;
  double icrxry = ic * rxry, icryrz = ic * ryrz, icrxrz = ic * rxrz;
  double srx = s * rx, sry = s * ry, srz = s * rz;
  double ict = ic * itheta, st = s * itheta, ist = 1 - st;
  double istrxry = ist * rxry, istrxrz = ist * rxrz, istryrz = ist * ryrz;
  double ictrx = ict * rx, ictry = ict * ry, ictrz = ict * rz;

  //multiply V^t to T. (V is the rotation matrix appeared in the exponential map)
  v[0] = (ist * rx2 + st)  * T[3] + (istrxry + ictrz) * T[7] + (istrxrz - ictry) * T[11];
  v[1] = (istrxry - ictrz) * T[3] + (ist * ry2 + st)  * T[7] + (istryrz + ictrx) * T[11];
  v[2] = (istrxrz + ictry) * T[3] + (istryrz - ictrx) * T[7] + (ist * rz2 + st)  * T[11];
  r[0] = theta * rx;
  r[1] = theta * ry;
  r[2] = theta * rz;
}

// se(3)->SE(3) exp
inline void exp_se3(const double * r, const double * v, double * T)
{
  double rx = r[0], ry = r[1], rz = r[2];
  double theta = sqrt(rx*rx + ry*ry + rz*rz);
  if(theta < DBL_EPSILON){
    memset((void*) T, 0, sizeof(double) * 16);
    T[0] = T[5] = T[10] = T[15] = 1.0;
    return;
  }

  double itheta = 1.0 / theta;

  rx *= itheta;
  ry *= itheta;
  rz *= itheta;

  double rx2 = rx * rx, ry2 = ry * ry, rz2 = rz * rz, 
    rxry = rx * ry, rxrz = rx * rz, ryrz = ry * rz;

  // skew symmetric matrix A
  // 0   -rz   ry
  // rz    0  -rx
  // -ry  rx    0

  // A^2
  // -rz2-ry2 rxry     rxrz
  // rxry     -rx2-rz2 ryrz
  // rxrz     ryrz     -rx2-ry2
  //
  // rx2+ry2+rz2=1.0 simplify A^2 as
  // rx2-1    rxry     rxrz
  // rxry     ry2-1    ryrz
  // rxrz     ryrz     rz2-1

  // Resulting matrix
  // T0  T1  T2  T3
  // T4  T5  T6  T7
  // T8  T9  T10 T11
  // T12 T13 T14 T15
  // 
  // R t
  // 0 1
	
  // R =
  // T0  T1  T2 
  // T4  T5  T6 
  // T8  T9  T10 
  //
  // t = 
  // T3
  // T7
  // T11

  // R=I + s A + (1-c) A^2
  // ic(rx2-1) + c   ic rxry - s rz  ic rxrz + s ry
  // ic rxry + s rz  ic(ry2-1) + c   ic ryrz - s rx
  // ic rxrz - s ry  ic ryrz + s rx  ic(rz2-1) + c
  double s = sin(theta);
  double c = cos(theta);
  double ic = 1.0 - c;
  double icrxry = ic * rxry, icryrz = ic * ryrz, icrxrz = ic * rxrz;
  double srx = s * rx, sry = s * ry, srz = s * rz;
  T[0] = ic * rx2 + c; T[1] = icrxry - srz; T[2] = icrxrz + sry;
  T[4] = icrxry + srz; T[5] = ic * ry2 + c; T[6] = icryrz - srx;
  T[8] = icrxrz - sry; T[9] = icryrz + srx; T[10] = ic * rz2 +  c;

  // calculating rotation of the translation
  // V=I + (1-c)/th A + (1-s/th) A^2
  // (1-s/th) rx2 + s/th          (1-s/th) rxry - (1-c)/th rz (1-s/th) rxrz + (1-c)/th ry
  // (1-s/th) rxry + (1-c)/th rz  (1-s/th) ry2 + s/th         (1-s/th) ryrz - (1-c)/th rx
  // (1-s/th) rxrz - (1-c)/th ry  (1-s/th) ryrz + (1-c)/th rx (1-s/th)rz2 + s/th
  //
  // t = Vv
  double ict = ic * itheta, st = s * itheta, ist = 1 - st;
  double istrxry = ist * rxry, istrxrz = ist * rxrz, istryrz = ist * ryrz;
  double ictrx = ict * rx, ictry = ict * ry, ictrz = ict * rz;
  T[3]  = (ist * rx2 + st)  * v[0] + (istrxry - ictrz) * v[1] + (istrxrz + ictry) * v[2];
  T[7]  = (istrxry + ictrz) * v[0] + (ist * ry2 + st)  * v[1] + (istryrz - ictrx) * v[2];
  T[11] = (istrxrz - ictry) * v[0] + (istryrz + ictrx) * v[1] + (ist * rz2 + st)  * v[2];

  T[12] = T[13] = T[14] = 0.;
  T[15] = 1.0;
}

// SIM(3)->sim(3) log
inline void log_sim3(const double * S, double & s, double * r, double * v)
{
  double es = sqrt(S[0] * S[0] + S[1] * S[1] + S[2] * S[2]);
  double ies = 1.0 / es;
  double T[16];
  memcpy((void*) T, (void*) S, sizeof(double) * 16);

  T[0] *= ies;
  T[1] *= ies;
  T[2] *= ies;
  T[4] *= ies;
  T[5] *= ies;
  T[6] *= ies;
  T[8] *= ies;
  T[9] *= ies;
  T[10] *= ies;

  log_se3(T, r, v);
  s = log(es);
}

// sim(3)->SIM(3) exp
inline void exp_sim3(double & s, const double * r, const double * v, double * S)
{
  exp_se3(r, v, S);
  double es = exp(s);
  S[0] *= es;
  S[1] *= es;
  S[2] *= es;
  S[4] *= es;
  S[5] *= es;
  S[6] *= es;
  S[8] *= es;
  S[9] *= es;
  S[10] *= es;
}

inline void so3toqt(const double * v, double * q)
{
  double th = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  double fac = 1.0 / th;
  th *= 0.5;
  double sh = fac * sin(th);
  double ch = cos(th);
  q[0] = v[0] * sh;
  q[1] = v[1] * sh;
  q[2] = v[2] * sh;
  q[3] = ch;
}

inline void qttoso3(const double * q, double * v)
{
  double sh = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
  double ch = q[3];
  double th = 2.0 * atan2(sh, ch);
  double fac = th / sh;
  v[0] = fac * q[0];
  v[1] = fac * q[1];
  v[2] = fac * q[2];
}

//////////////////////////////////////////////////////////////////////////////////////////////// Projection Function
////////////////////////////////////////////////////////////////////// Single point projection.
inline void prjPt(const cv::Point3f & M, cv::Point2f & m, 
		  const cv::Mat & camint, const cv::Mat & camdist, 
		  const cv::Mat & rvec, const cv::Mat & tvec)
{
  const double * R, * t, * k;
  cv::Mat _R = cv::Mat(3, 3, CV_64FC1);
  exp_so3(rvec.ptr<double>(), _R.ptr<double>());
  //	Rodrigues(rvec, _R);
  R = _R.ptr<double>();
  t = tvec.ptr<double>();
  k = camdist.ptr<double>();
  const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
    cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

  double X = M.x, Y = M.y, Z = M.z;
  double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
  double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
  double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
  double r2, r4, r6, a1, a2, a3, cdist, icdist2;
  double xd, yd;

  z = z ? 1./z : 1;
  x *= z; y *= z;

  r2 = x*x + y*y;
  r4 = r2*r2;
  r6 = r4*r2;
  a1 = 2*x*y;
  a2 = r2 + 2*x*x;
  a3 = r2 + 2*y*y;
  cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
  icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
  xd = x*cdist*icdist2 + k[2]*a1 + k[3]*a2;
  yd = y*cdist*icdist2 + k[2]*a3 + k[3]*a1;

  m.x = (float)(xd*fx + cx);
  m.y = (float)(yd*fy + cy);
}

inline void prjPt(const cv::Point3f & M, cv::Point2f & m,
		  const double fx, const double fy, const double cx, const double cy,
		  const double * k, const double * R, const double * t)
{
  double X = M.x, Y = M.y, Z = M.z;
  double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
  double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
  double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
  double r2, r4, r6, a1, a2, a3, cdist, icdist2;
  double xd, yd;

  z = z ? 1./z : 1;
  x *= z; y *= z;

  r2 = x*x + y*y;
  r4 = r2*r2;
  r6 = r4*r2;
  a1 = 2*x*y;
  a2 = r2 + 2*x*x;
  a3 = r2 + 2*y*y;
  cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
  icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
  xd = x*cdist*icdist2 + k[2]*a1 + k[3]*a2;
  yd = y*cdist*icdist2 + k[2]*a3 + k[3]*a1;

  m.x = (float)(xd*fx + cx);
  m.y = (float)(yd*fy + cy);
}

// no distortion version
inline void prjPt(const cv::Point3f & M, cv::Point2f & m, 
		  const cv::Mat & camint,	const cv::Mat & rvec, const cv::Mat & tvec)
{
  const double * R, * t;
  cv::Mat _R = cv::Mat(3, 3, CV_64FC1);
  exp_so3(rvec.ptr<double>(), _R.ptr<double>());
  //	Rodrigues(rvec, _R);
  R = _R.ptr<double>();
  t = tvec.ptr<double>();
  const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
    cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

  double X = M.x, Y = M.y, Z = M.z;
  double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
  double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
  double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];

  z = z ? 1./z : 1;
  x *= z; y *= z;

  m.x = (float)(x*fx + cx);
  m.y = (float)(y*fy + cy);
}

inline void prjPt(const cv::Point3f & M, cv::Point2f & m,
		  const double fx, const double fy, const double cx, const double cy, 
		  const double * R, const double * t)
{
  double X = M.x, Y = M.y, Z = M.z;
  double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
  double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
  double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];

  z = z ? 1./z : 1;
  x *= z; y *= z;

  m.x = (float)(x*fx + cx);
  m.y = (float)(y*fy + cy);
}

// Projects 3D point in camera coordinate (without transformation)
inline void prjPt(const cv::Point3f & Mcam, cv::Point2f & m,
		  const double & fx, const double & fy, const double & cx, const double & cy)
{
  double x = Mcam.x, y = Mcam.y, z = Mcam.z;

  z = z ? 1./z : 1;
  x *= z; y *= z;

  m.x = (float)(x*fx + cx);
  m.y = (float)(y*fy + cy);
}

////////////////////////////////////////////////////////////////////// Multiple points projection.
// parameters are given as Mat capsulated array.
void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m,
	    const cv::Mat & camint, const cv::Mat & camdist, const cv::Mat & rvec, const cv::Mat & tvec);

// parameters are given as raw double pointers.
void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m,
	    const double fx, const double fy, const double cx, const double cy,
	    const double * k, const double * R, const double * t);

// valid flag prevents from calculating specific points.
void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const cv::Mat & camint, const cv::Mat & camdist, const cv::Mat & rvec, const cv::Mat & tvec);

// valid flag prevents from calculating specific points.
void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m,  const vector<int> & valid,
	    const double fx, const double fy, const double cx, const double cy,
	    const double * k, const double * R, const double * t);

// calculating jacobians for both camera intrinsics and extrinsics
void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const cv::Mat & camint, const cv::Mat & camdist,
	    const cv::Mat & rvec, const cv::Mat & tvec,
	    double * jf, double * jc, double * jk, double * jp,
	    double * jr, double * jt, double arf = 0.0);

// calculating jacobians only for camera extrinsics (rotation and translation)
void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const cv::Mat & camint, const cv::Mat & camdist,
	    const cv::Mat & rvec, const cv::Mat & tvec,
	    double * jr, double * jt);

// no distortion version
void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m,
	    const cv::Mat & camint, const cv::Mat & rvec, const cv::Mat & tvec);

void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m,
	    const double fx, const double fy, const double cx, const double cy, const double * R, const double * t);

void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const cv::Mat & camint,	const cv::Mat & rvec, const cv::Mat & tvec);
void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const double fx, const double fy, const double cx, const double cy, const double * R, const double * t);

void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const cv::Mat & camint,	const cv::Mat & rvec, const cv::Mat & tvec,
	    double * jr, double * jt);

void prjPts(const vector<cv::Point3f> & Mcam, vector<cv::Point2f> & m, const vector<int> & valid, const cv::Mat & camint);
void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const double fx, const double fy, const double cx, const double cy);

bool test_prjPtsj(cv::Mat & camint, cv::Mat & camdist, cv::Mat & rvec, cv::Mat & tvec, 
		  vector<cv::Point3f> & pt3d, vector<int> & valid, cv::Mat & jacobian, double arf = 0.0);

//////////////////////////////////////////////////////////////////////////////////////////////// Transformation
// Transformation 3D pints with rotation and translation [R|t].
inline void trnPt(const cv::Point3f & Msrc, cv::Point3f & Mdst, const double * pR, const double * pt)
{
  double X = Msrc.x, Y = Msrc.y, Z = Msrc.z;
  Mdst.x = (float)(pR[0]*X + pR[1]*Y + pR[2]*Z + pt[0]);
  Mdst.y = (float)(pR[3]*X + pR[4]*Y + pR[5]*Z + pt[1]);
  Mdst.z = (float)(pR[6]*X + pR[7]*Y + pR[8]*Z + pt[2]);
}

void trnPts(const vector<cv::Point3f> & Msrc, vector<cv::Point3f> & Mdst, const vector<int> & valid, 
	    const cv::Mat & R, const cv::Mat & t);

void trnPts(const vector<cv::Point3f> & Msrc, vector<cv::Point3f> & Mdst, const cv::Mat & R, const cv::Mat & t);

/////////////////////////////////////////////////////////////////////////////// Transformation related Jacobian
// Calcurate dT(r',t')T(r,t)/d(r', t') at (r', t') = 0. T(r, t) is the transformation matrix of [R|t],
// R is rotation matrix the exponential map of r
// If R and t given is NULL, simply dT(r', t')/d(r', t') at (r', t') = 0 is calculated.
// Resulting jacobian is column stacked. 
// Assuming the target transformation 
// T(r',t')T(r,t) = 
//		r11 r12 r13 t1
//		r21 r22 r23 t2
//		r31 r32 r33 t3
// Resulting jacobian is 12x6 matrix
// J =
//     /dr1 /dr2 /dr3 /dt1 /dt2 /dt3
// dr11
// dr21     -rc1_x          O_3
// dr31
// dr12
// dr22     -rc2_x          O_3
// dr32
// dr13
// dr23     -rc3_x          O_3
// dr33
// dt1     
// dt2      -t_x            I_3
// dt3
// 
// Here rc1 = [r11 r21 r31]^t and the rc1_x is the skew-symmetric matrix.
// 
inline void calc_dR0t0Rtdrt(cv::Mat & J, double * R = NULL, double * t = NULL)
{
  J = cv::Mat::zeros(12, 6, CV_64FC1);
  double * p = J.ptr<double>();

  // left-top 9x3 block
  if(R){
    p[ 0] =    0.; p[ 1] =  R[6]; p[ 2] = -R[3];
    p[ 6] = -R[6]; p[ 7] =    0.; p[ 8] =  R[0];
    p[12] =  R[3]; p[13] = -R[0]; p[14] =    0.;

    p[18] =    0.; p[19] =  R[7]; p[20] = -R[4];
    p[24] = -R[7]; p[25] =    0.; p[26] =  R[1];
    p[30] =  R[4]; p[31] = -R[1]; p[32] =    0.;

    p[36] =    0.; p[37] =  R[8]; p[38] = -R[5];
    p[42] = -R[8]; p[43] =    0.; p[44] =  R[2];
    p[48] =  R[5]; p[49] = -R[2]; p[50] =    0.;
  }else{
    p[ 8] = 1.;
    p[13] = -1.;

    p[21] = -1.;
    p[30] = 1.;

    p[37] = 1.;
    p[42] = -1.;
  }

  // left-bottom 3x3 block
  if(t){
    p[54] =    0.; p[55] =  t[2]; p[56] = -t[1];
    p[60] = -t[2]; p[61] =    0.; p[62] =  t[0];
    p[66] =  t[1]; p[67] = -t[0]; p[68] =    0.;
  }

  // right bottom 3x3 block (I_3)
  p[57] = 1.;
  p[64] = 1.;
  p[71] = 1.;
}

// numerical version of calc_dR0t0Rtdrt. Forward difference approximation is used.
#define DIFF_STEP 1e-8
#define IDIFF_STEP (1. / DIFF_STEP)

void calcn_dR0t0Rtdrt(cv::Mat & J, double * R = NULL, double * t = NULL);

// Calculate dM/d(r, t) using chain rule.
// assuming JT (dT/d(r,t)) is calcurated with calcJT0_SE3 defined above.
// J =
//            /dr1 /dr2 /dr3                  /dt1 /dt2 /dt3
// dX
// dY  -(Xrc1_x+Yrc2_x+Zrc2_x+t_x)                  I_3
// dZ
inline void calcJMcrt0_SE3(cv::Mat & J, const cv::Point3f & Mo, const cv::Mat & JT)
{
  const double * pJT = JT.ptr<double>();
  J = cv::Mat::zeros(3, 6, CV_64FC1);
  double * pJ = J.ptr<double>();

  //  Of course it contains much wasteful codes. Half of the multiplication can be eliminated because the
  // multiplications are all for skew-symmetric matrices.
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      int i6 = i * 6;
      pJ[i6 + j] = pJT[i6 + j] * Mo.x + pJT[i6 + 3 * 6 + j] * Mo.y
	+ pJT[i6 + 6 * 6 + j] * Mo.z + pJT[i + 9 * 6 + j];
    }
  }

  pJ[3] = pJ[10] = pJ[17] = 1.0;
}

// calculate [Ix, Iy]_m+(u,v) * dm/dMc * dMc/dp
// resulting jacobian is 
inline void calcJI(const double & Ix, const double & Iy, 
		   const cv::Point3f & Mc,
		   const double fx, const double fy,
		   const double * pdMcdp, double * pdIdp)
{
  // pdMcdp 3 x 6

  double iMz = 1.0 / Mc.z;
  double dmxdMx = fx * iMz;
  double dmydMy = fy * iMz;
  double dmxdMz = - dmxdMx * Mc.x * iMz;
  double dmydMz = - dmydMy * Mc.y * iMz;

  double dmxdp, dmydp;
  for(int i = 0; i < 6; i++){
    dmxdp = pdMcdp[  i  ] * dmxdMx + pdMcdp[12 + i] * dmxdMz;
    dmydp = pdMcdp[6 + i] * dmydMy + pdMcdp[12 + i] * dmydMz;
    pdIdp[i] = Ix * dmxdp + Iy * dmydp;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////// bilinear sampling
// bi-linear sampler for gray scale image
// pI is the pointer to the image, w and h are its width and height.
// x and y are the location to be sampled.

// sampleBL the uchar version
inline uchar sampleBL(const uchar * pI, const int w, const int h, const float x, const float y)
{
  int ix = (int)x, iy = (int) y;
  int ix2 = ix + 1, iy2 = iy + 1;
  double rx = (x - (double)ix), ry = (y - (double)iy);

  // basically we calculate 
  // (1-ry)*[(1-rx)*I(ix,iy) + rx*I(ix2, iy)] + ry*[(1-rx)*I(ix,iy2) + rx*I(ix2, iy2)]
  // but we can reduce it
  // (1-ry)*[I(ix,iy) + rx*(I(ix2, iy)-I(ix,iy))] + ry*[I(ix,iy2) + rx*(I(ix2, iy2) - I(ix,iy2))]
  // I(ix,iy) + ry*[I(ix,iy2) - I(ix,iy)] + rx*ry*[I(ix2, iy2) - I(ix,iy2) - I(ix2, iy) + I(ix,iy)]
  // here I denote I(ix,iy), I(ix2, iy), I(ix,iy2), I(ix2, iy2) as LT, RT, LB, RB,
  // and LB - LT as LBmLT. Sampled value is
  // LT + ry * LBmLT + rx * ry ( RB - RT - LBmLT)

  uchar LT, RT, LB, RB;

  LT = pI[w * iy + ix];
  RT = pI[w * iy + ix2];
  LB = pI[w * iy2 + ix];
  RB = pI[w * iy2 + ix2];

  return cv::saturate_cast<uchar>((double) LT + ry * ((int)LB - (int)LT)
			      + rx * ry * ((int)RB - (int)RT - (int)LB - (int)LT));
}

// sampleBL the double version
inline double sampleBL(const double * pI, const int w, const int h, const float x, const float y)
{
  int ix = (int)x, iy = (int) y;
  int ix2 = ix + 1, iy2 = iy + 1;
  double rx = (x - (double)ix), ry = (y - (double)iy);

  // basically we calculate 
  // (1-ry)*[(1-rx)*I(ix,iy) + rx*I(ix2, iy)] + ry*[(1-rx)*I(ix,iy2) + rx*I(ix2, iy2)]
  // but we can reduce it to
  // (1-ry)*[I(ix,iy) + rx*(I(ix2, iy)-I(ix,iy))] + ry*[I(ix,iy2) + rx*(I(ix2, iy2) - I(ix,iy2))]
  // I(ix,iy) + ry*[I(ix,iy2) - I(ix,iy)] + rx*ry*[I(ix2, iy2) - I(ix,iy2) - I(ix2, iy) + I(ix,iy)]
  // here I denote I(ix,iy), I(ix2, iy), I(ix,iy2), I(ix2, iy2) as LT, RT, LB, RB,
  // and LB - LT as LBmLT. Sampled value is
  // LT + ry * LBmLT + rx * ry ( RB - RT - LBmLT)

  double LT, RT, LB, RB;

  LT = pI[w * iy + ix];
  RT = pI[w * iy + ix2];
  LB = pI[w * iy2 + ix];
  RB = pI[w * iy2 + ix2];

  return (LT + ry * (LB - LT) + rx * ry * (RB - RT - LB - LT));
}

///////////////////////////////////////////////////////////////////////////////////////////////// transformation
// calculates
// [R3|t3] = [R1|t1][R2|t2]
// [0 | 1]   [0 | 1][0 | 1]
// R3 = R1R2
// t3 = R1t2+t1
inline void compRt(const double * R1, const double * t1, 
		   const double *R2, const double *t2, double *R3, double *t3)
{
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      int idx1 = i * 3;
      int idx2 = j;
      int idx3 = idx1 + idx2;

      R3[idx3] = 0.;
      for(int k = 0; k < 3; k++, idx1++, idx2+=3){
	R3[idx3] += R1[idx1] * R2[idx2];
      }
    }

    int idx1 = i * 3;
    t3[i] = 0.;
    for(int k = 0; k < 3; k++, idx1++){
      t3[i] += t2[k] * R1[idx1];
    }
    t3[i] += t1[i];
  }
}

bool synth_afn(cv::Mat & l, cv::Mat & r, cv::Mat & res);

bool afn(cv::Mat & A, cv::Point2f & in, cv::Point2f & pt_out);

//////////////////////////////////////////////////////////////////////////////////////// Bayer pattern handling.
void cnvBayerRG8ToBGR8(cv::Mat & src, cv::Mat & dst);
void cnvBayerRG16ToBGR16(cv::Mat & src, cv::Mat & dst);
void cnvBayerGR8ToBGR8(cv::Mat & src, cv::Mat & dst);
void cnvBayerGR8ToBGR8NN(cv::Mat & src, cv::Mat & dst);
void cnvBayerGR8ToBGR8Q(cv::Mat & src, cv::Mat & dst);
void cnvBayerGR8ToG8Q(cv::Mat & src, cv::Mat & dst);
void cnvBayerGR8ToDG8Q(cv::Mat & src, cv::Mat & dst);
void cnvBayerGR16ToBGR16(cv::Mat & src, cv::Mat & dst);
void cnvBayerGB8ToBGR8(cv::Mat & src, cv::Mat & dst);
void cnvBayerGB16ToBGR16(cv::Mat & src, cv::Mat & dst);
void cnvBayerBG8ToBGR8(cv::Mat & src, cv::Mat & dst);
void cnvBayerBG16ToBGR16(cv::Mat & src, cv::Mat & dst);

///////////////////////////////////////////////////////////////////////////////////////////////// Miscellaneous 
// calculates AtA. A is a rowsxcols matrix, and the resulting matrix AtA is cols x cols
inline void calcAtA(const double * A, int rows, int cols, double * AtA)
{
  const double * p0 = A, * p1 = A;
  double * p2 = AtA;

  for(int i = 0; i < cols; i++){
    for(int j = i; j < cols; j++){
			
      p0 = A + i;
      p1 = A + j;
      p2 = AtA + j + i * cols;
      *p2 = 0.;
      for(int k = 0; k < rows; k++, p0 += cols, p1 += cols){
	*p2 += *p0 * *p1;
      }

      // note that AtA(i,j) = AtA(j,i)
      *(AtA + i + j * cols) = *p2;
    }
  }
}

// calculate AtA, where is a rows x cols matrix, V is a rows dimensional vector.
inline void calcAtV(const double * A, const double * V, int rows, int cols, double * AtV)
{
  for(int i = 0; i < cols; i++){
    const double * pa = A + i;
    const double * pv = V;
    double * p = AtV + i;
    *p = 0.;
    for(int j = 0; j< rows; j++, pa += cols, pv++){
      *p +=  *pa * *pv;
    }
  }
}

void layoutPyramid(const vector<cv::Mat> & IPyr, cv::Mat & out);
void cnv64FC1to8UC1(const cv::Mat & in, cv::Mat & out);
void cnv32FC1to8UC1(const cv::Mat & in, cv::Mat & out);
void cnv16UC1to8UC1(const cv::Mat & in, cv::Mat & out);
void cnv16UC3to8UC3(const cv::Mat & in, cv::Mat & out);

// cnvCVBGR8toGLRGB8 converts OpenCV's 8bit BGR image to OpenGL's 8bit RGB format.
// OpenCV uses a coordinate positive in right and bottom direction, and OpenGL does that in positive in right and top direction.
// So the function swaps the color R and B saimultaneously the lines top and bottom.
void cnvCVBGR8toGLRGB8(cv::Mat & img);
void cnvCVGRAY8toGLGRAY8(cv::Mat & img);

// flip in x, y axes and channel order, without allocating memory (img is the input and the output)
void awsFlip(cv::Mat & img, bool xflip, bool yflip, bool chflip);

///////////////////////////////////////////////////////////////////////////////////////////////// AWSCamPar
// AWS's camera parameter class. Defines file format and its read/write methods.
class AWSCamPar
{
protected:
  static int version;
  double par[12];

  cv::Mat P; // projection matrix
  cv::Mat D; // distortion coefficient
  
  bool m_bFishEye
  ;
public:
  enum Param {
    epfx, epfy, epcx, epcy, epk1, epk2, epp1, epp2, epk3, epk4, epk5, epk6
  };
  
  enum FishEyeParam{
    epffx, epffy, epfcx, epfcy, efpk1, epfk2, epfk3, epfk4
  };
  
  AWSCamPar()
  {
    for(int ipar = 0; ipar < 12; ipar++)
      par[ipar] = 0.;
  };
  
  void setFishEye(bool flag){
    m_bFishEye = flag;
  };
  
  bool isFishEye(){
    return m_bFishEye;
  }
  
  double * getCvPrj(){
    return par;
  }
  
  cv::Mat getCvPrjMat(){
    P = cv::Mat::eye(3, 3, CV_64FC1);
    double * p = P.ptr<double>();
    p[0] = par[0];
    p[2] = par[2];
    p[4] = par[1];
    p[5] = par[3];
    return P;
  }
  
  double * getCvDist(){
    return &par[epk1];
  }
  
  double * getCvDistFishEye(){
    return &par[efpk1];
  }
  
  cv::Mat getCvDistMat(){
    D = cv::Mat(1, 8, CV_64FC1, &par[epk1]);
    return D;
  }
  
  cv::Mat getCvDistFishEyeMat(){
    D = cv::Mat(1, 4, CV_64FC1, &par[efpk1]);
    return D;
  }
  
  void setCvPrj(const cv::Mat & P)
  {
    const double * pP = P.ptr<double>();
    double * pPdst = getCvPrj();
    pPdst[0] = pP[0];
    pPdst[1] = pP[4];
    pPdst[2] = pP[2];
    pPdst[3] = pP[5];
  }
  
  void setCvDist(const cv::Mat & D)
  {
    const double * pD = D.ptr<double>();
    double * pDdst = getCvDist();
    for(int i = 0; i < D.cols; i++){
      pDdst[i] = pD[i];
    }
  }
  
  bool read(const char * fname);
  bool write(const char * fname);
};

// Handling attitude (rotation and translation). Representation of the Rotation can be three types: Rotation Matrix, Rotation Vector, and Quarternion.
// Conversion between any two types are given in the method. But the conversion to quarternion has still overhead due to calculating the norm twice.
// it could be eliminated easily but I didnt yet.
struct AWSAttitude{
  static int version;
  enum attype { 
    RMTX = 0, QTAN, RVEC
  } type;
  
  union{
    double pM[9]; // Rotation Matrix
    double pV[3]; // Rotation Vector
    double pQ[4]; // Quarternion
  };
  
  double t[3];
  
  AWSAttitude():type(RMTX)
  {
    pM[0] = pM[4] = pM[8] = 1.0;
    pM[1] = pM[2] = pM[3] = pM[5] = pM[6] = pM[7] = 0.0;
    t[0] = t[1] = t[2] = 0.0;
  };
  
  void setT(const cv::Mat & M){
    const double * ptr = M.ptr<double>();
    t[0] = ptr[0];
    t[1] = ptr[1];
    t[2] = ptr[2];
    
  }
  const cv::Mat getT(){
    return cv::Mat(3, 1, CV_64FC1, t);
  }
  
  void setQtan(const cv::Mat & Q)
  {
    const double * ptr = Q.ptr<double>();
    memcpy((void*) pQ, (const void*) ptr, sizeof(double) * 4);
    type = QTAN;
  }
  
  void cnvQtan()
  {
    cv::Mat Q = getQtan();
    const double * ptr = Q.ptr<double>();
    pQ[0] = ptr[0];
    pQ[1] = ptr[1];
    pQ[2] = ptr[2];
    pQ[3] = ptr[3];
  }
  
  const cv::Mat getQtan(){
    switch(type){
    case RVEC:
      {
	cv::Mat Q(4, 1, CV_64FC1);
	double * ptr = Q.ptr<double>();
	so3toqt(pV, ptr);
	return Q;
      }
      break;
    case RMTX:
      {
	double v[3];
	cv::Mat Q(4, 1, CV_64FC1);
	double * ptr = Q.ptr<double>();
	log_so3(pM, v);
	so3toqt(v, ptr);
	return Q;
      }
      break;
    case QTAN:
      return cv::Mat(4, 1, CV_64FC1, pQ);
    }
    return cv::Mat();
  }
  
  void setRmtx(const cv::Mat & R)
  {
    const double * ptr = R.ptr<double>();
    memcpy((void*) pM, (const void*) ptr, sizeof(double) * 9);
    type = RMTX;
  }
  
  void cnvRmtx()
  {
    cv::Mat M = getRmtx();
    const double * ptr = M.ptr<double>();
    for (int i = 0; i < 9; i++)
      pM[i] = ptr[i];
  }
  
  const cv::Mat getRmtx()
  {
    switch(type){
    case RVEC:
      {
	cv::Mat R(3, 3, CV_64FC1);
	double * ptr = R.ptr<double>();
	exp_so3(pV, ptr);
	return R;
      }
    case RMTX:
      return cv::Mat(3, 3, CV_64FC1, pM);
    case QTAN:
      {
	cv::Mat Q(3, 1, CV_64FC1);
	cv::Mat R(3, 3, CV_64FC1);
	double v[3];
	double * ptr = R.ptr<double>();
	qttoso3(pQ, v);
	exp_so3(pV, ptr);
	return R;
      }
      break;
    }
    return cv::Mat();
  }
  
  void setRvec(const cv::Mat & V)
  {
    const double * ptr = V.ptr<double>();
    memcpy((void*) pV, (const void*) ptr, sizeof(double) * 3);
    type = RVEC;
  }
  
  void cnvRvec()
  {
    const cv::Mat V = getRvec();
    const double * ptr = V.ptr<double>();
    pV[0] = ptr[0];
    pV[1] = ptr[1];
    pV[2] = ptr[2];
  }
  
  const cv::Mat getRvec()
  {
    switch(type){
    case RVEC:
      return cv::Mat(3, 1, CV_64FC1, pV);
    case RMTX:
      {
	cv::Mat vec = cv::Mat(3, 1, CV_64FC1);
	double * ptr = vec.ptr<double>();
	log_so3(pM, ptr);
	return vec;
      }
      break;
    case QTAN:
      {
	cv::Mat Q(4, 1, CV_64FC1);
	double * ptr = Q.ptr<double>();
	so3toqt(pV, ptr);
	return Q;
      }
      break;
    }
    return cv::Mat();
  }
  
  bool read(const char * fname);
  bool write(const char * fname);
};

/////////////////////////////////////////////////////////////////////// stereo related 
struct s_sgbm_par{
  bool m_update;
  bool m_bsg;

  int minDisparity; /* normally zero. it depends on rectification algorithm. */
  int numDisparities; /* maximum disparity - minimum disparity. it must be the divisible number by 16*/
  int blockSize; /* 3 to 11 */
  int P1, P2;
  int disp12MaxDiff;
  int preFilterCap;
  int uniquenessRatio;
  int speckleWindowSize;
  int speckleRange;
  int mode;
  s_sgbm_par() :m_update(false), m_bsg(true), minDisparity(0), numDisparities(64), blockSize(3),
		disp12MaxDiff(1), preFilterCap(0), uniquenessRatio(10), speckleWindowSize(100),
		speckleRange(32), mode(0/*StereoSGBM::MODE_SGBM*/)
  {
    P1 = 8 * blockSize * blockSize;
    P2 = P1 * 4;
  }
};
struct s_obst{
  int xmin, xmax, ymin, ymax;
  ushort dmin, dmax;
  int pix;
  ushort diff(ushort d)
  {
    return (ushort) abs((int)((dmin + dmax) >> 1) - (int)d);
  }

  ushort d()
  { 
    return (dmin + dmax) >> 1; 
	
  }
  void add(int x, int y, ushort d)
  {
    xmin = min(xmin, x);
    xmax = max(xmax, x);
    ymin = min(ymin, y);
    ymax = max(ymax, y);
    dmin = min(dmin, d);
    dmax = max(dmax, d);
    pix++;
  }

  s_obst() :xmin(0), xmax(0), ymin(0), ymax(0), dmin(0), dmax(0), pix(0){}
};


struct s_odt_par{
  cv::Mat lbl;
  vector<int> tmp;
  int nullpix;

  ushort drange;
  cv::Size bb_min_n, bb_min_f;
  int foot_y;
  ushort dn, df;
  float f,cx, cy, L, Dmax, iDmax;
  s_odt_par() :drange(32),
	       bb_min_n(5, 75), bb_min_f(5, 25), dn(640), df(64), foot_y(270)
  {}

  void initD(float _f/*focal length*/, float _cx /*principal point*/, 
	     float _cy /*principal point*/, float _L/* base line */)
  {
    f = _f;
    cx = _cx;
    cy = _cy;
    L = _L;
    Dmax = (float)(16. * L * f);
    iDmax = (float)(1.0 / Dmax);
  }

  float rad(s_obst & o){
    //double rd = Dmax / (double)o.dmin - Dmax / (double)o.dmax;
    double rx = d2D((float)((o.dmax + o.dmin) >> 1)) * (double)(o.xmax - o.xmin) / f;
    return (float)rx;
  }

  void bd(s_obst & o, float & bear, float & dist){
    float D = d2D((float)((o.dmax + o.dmin) >> 1));
    float X = (float)(((float)((o.xmax + o.xmin) >> 1) - cx) * D / f);
    dist = (float)sqrt(D*D + X*X);
    bear = (float)atan2(X, D);
  }

  float d2D(float d) // disparity to distance transformation
  {
    return (float)(Dmax / d);
  }

  float D2d(float D)
  {
    return (float)(iDmax * D);
  }

  bool is_valid(s_obst & o){
    int hlim = bb_min_n.height -
      (int)(
	    ((int)(bb_min_n.height - bb_min_f.height)
	     * (int)(dn - o.d())) / (double)(dn - df)
	    );
    return (o.ymax > foot_y) && ((o.ymax - o.ymin) > hlim);
  }
};


void calc_obst(s_odt_par & par, cv::Mat & disp, vector<s_obst> & obst);

//////////////////////////////////////////////////////////////////////// mat img read/write

bool write_raw_img(const cv::Mat & img, const char * fname);
bool read_raw_img(cv::Mat & img, const char * fname);



#endif
