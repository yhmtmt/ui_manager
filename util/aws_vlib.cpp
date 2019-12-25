// Copyright(c) 2015-2019 Yohei Matsumoto, All right reserved. 

// aws_vlib.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_vlib.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_vlib.cpp.  If not, see <http://www.gnu.org/licenses/>.
#include <iostream>
#include <vector>
using namespace std;
#include "aws_vlib.hpp"

const char * str_imfmt[IMF_Undef] = 
{
  "GRAY8", "GRAY10", "GRAY12", "GRAY14", "GRAY16",
  "RGB8", "RGB10", "RGB12", "RGB14", "RGB16",
  "BGR8", "BGR10", "BGR12", "BGR14", "BGR16",
  "BayerBG8", "BayerGB8", "BayerGR8", "BayerRG8",  
  "BayerBG10", "BayerGB10", "BayerGR10", "BayerRG10",
  "BayerBG12", "BayerGB12", "BayerGR12", "BayerRG12",
  "NV12", "I420"
};

///////////////////////////////////////////////////////////////////////////////////////////////// AWSCamPar
int AWSCamPar::version = 0;

bool AWSCamPar::read(const char * fname)
{
  cv::FileStorage fs(fname, cv::FileStorage::READ);
  if(!fs.isOpened()){
    cerr << "Failed to open " << fname << endl;
    return false;
  }

  cv::FileNode fn;

  fn = fs["AWSCamPar"];
  if(fn.empty()){
    cerr << "File " << fname << " does not have AWSCamPar" << endl;
    return false;
  }
  int file_version;
  fn >> file_version;

  if(file_version != version){
    cerr << "File " << fname << " is version " << file_version 
	 << ". Current version is " << version << "." << endl;
    return false;
  }

  fn = fs["FishEye"];
  if(fn.empty()){
    cerr << "File " << fname << " does not have FishEye flag." << endl;
  }
  fn >> m_bFishEye;

  fn = fs["Params"];
  if(fn.empty()){
    cerr << "File " << fname << "does not have Params." << endl;
    return false;
  }

  if(m_bFishEye){
    cv::Mat tmp(1, epfk4 + 1, CV_64FC1, par);
    fn >> tmp;
  }else{
    cv::Mat tmp(1, epk6 + 1, CV_64FC1, par);
    fn >> tmp;
  }

  return true;
}

bool AWSCamPar::write(const char * fname)
{
  cv::FileStorage fs(fname, cv::FileStorage::WRITE);
  if(!fs.isOpened()){
    cerr << "Failed to open " << fname << endl;
    return false;
  }

  // Version information 
  fs << "AWSCamPar" << version;
  fs << "FishEye" << m_bFishEye;
  if(m_bFishEye)
    fs << "Params" << cv::Mat(1, epfk4 + 1, CV_64FC1, par);
  else
    fs << "Params" << cv::Mat(1, epk6 + 1, CV_64FC1, par);

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////// AWSAttitude
int AWSAttitude::version = 0;
bool AWSAttitude::read(const char * name)
{
  try{
    cv::FileStorage fs(name, cv::FileStorage::READ);
    if(!fs.isOpened()){
      cerr << "Failed to open file " << name << endl;
      cv::FileNode fn;

      fn = fs["AWSAttitude"];
      if(fn.empty()){
	cerr << "AWSAttitude is not found." << endl;
	return false;
      }
      int version_tmp;
      fn >> version_tmp;
      if(version != version_tmp){
	cerr << "AWSAttitude's version is not matched. Current version is " << version << "." << endl;
	return false;
      }


      fn = fs["type"];
      if(fn.empty()){
	cerr << "type is not found." << endl;
	return false;
      }
      int itype;
      fn >> itype;
      type = (attype) itype;

      fn = fs["Rotation"];
      if(fn.empty()){
	cerr << "Rotation is not found." << endl;
	return false;
      }
      cv::Mat M;
      fn >> M;
      switch(type){
      case RMTX:
	if(M.rows == M.cols && M.rows == 3){
	  setRmtx(M);
	}else{
	  cerr << "Rotation should be 3x3 matrix." << endl;
	  return false;
	}
	break;
      case RVEC:
	if(M.rows == 3 && M.cols == 1){
	  setRvec(M);
	}else{
	  cerr << "Rotation should be 3x1 vector." << endl;
	  return false;
	}
	break;
      case QTAN:
	if(M.rows = 4 && M.cols == 1){
	  setQtan(M);
	}else{
	  cerr << "Rotation should be 4x1 vector." << endl;
	  return false;
	}
      }
		
      fn = fs["Translation"];
      if(fn.empty()){
	cerr << "Translation is not found." << endl;
	return false;
      }

      fn >> M;
      if(M.rows == 3 && M.cols == 1)
	setT(M);	
      else{
	cerr << "Translation should be 3x1 vector." << endl;
	return false;
      }
    }
  }catch(...){
    cerr << "Failed to read " << name << endl;
    return false;
  }
  return true;
}

bool AWSAttitude::write(const char * name)
{
  try{
    cv::FileStorage fs(name, cv::FileStorage::WRITE);
    fs << "AWSAttitude" << version;
    fs << "type" << (int) type;
    fs << "Rotation";
	
    switch(type){
    case RMTX:
      fs << getRmtx();
      break;
    case RVEC:
      fs << getRvec();
      break;
    case QTAN:
      fs << getQtan();
      break;
    }
    fs << "Translation" << getT();
  }catch(...){
    cerr << "Failed to write " << name << endl;
    return false;
  }
  return true;
}



/////////////////////////////////////////////////////////////////////// extracting Euler angles from Rotation matrix
void angleRxyz(double * R, double & x, double & y, double &z)
{
  // R
  // r11 r12 r13
  // r21 r22 r23
  // r31 r32 r33
  //
  // R0  R1  R2 
  // R3  R4  R5
  // R6  R7  R8
  // left multiplication decompose Rz^t Ry^t Rx^t I = R(because our rotation order is RxRyRz x)
  double r, cx, sx, cy, sy, cz, sz;
  double Rtmp[9];

  // Find Givens rotation Rx^t to make r23->0
  // Rx^t
  // 1  0  0
  // 0  c  s
  // 0 -s  c
  //
  // r = sqrt(r23^2+r33^2)
  // s = -r23/r
  // c = r33/r
  r = 1./sqrt(R[5]*R[5] + R[8]*R[8]);
  sx = -R[5]  * r;
  cx = R[8] * r;

  // then Rx^tR=>R
  Rtmp[0] = R[0];
  Rtmp[1] = R[1];
  Rtmp[2] = R[2];

  Rtmp[3] = cx * R[3] + sx * R[6];
  Rtmp[4] = cx * R[4] + sx * R[7];
  Rtmp[5] = cx * R[5] + sx * R[8];

  Rtmp[6] = -sx * R[3] + cx * R[6];
  Rtmp[7] = -sx * R[4] + cx * R[7];
  Rtmp[8] = -sx * R[5] + cx * R[8];

  // Find Givens rotation Ry^t to make r13->0
  // Ry^t
  // c  0 -s
  // 0  1  0
  // s  0  c
  //
  // r= sqrt(r13^2+r33^2)
  // s = r13/r
  // c = r33/r
  r = 1./sqrt(Rtmp[2] * Rtmp[2] + Rtmp[8] * Rtmp[8]);
  sy = Rtmp[2] * r;
  cy = Rtmp[8] * r;

  // then Ry^tR=>R
  R[0] = cy * Rtmp[0] - sy * Rtmp[6];
  R[1] = cy * Rtmp[1] - sy * Rtmp[7];
  R[2] = cy * Rtmp[2] - sy * Rtmp[8];

  R[3] = Rtmp[3];
  R[4] = Rtmp[4];
  R[5] = Rtmp[5];

  R[6] = sy * Rtmp[0] + cy * Rtmp[6];
  R[7] = sy * Rtmp[1] + cy * Rtmp[7];
  R[8] = sy * Rtmp[2] + cy * Rtmp[8];

  // Find Givens rotation Rz^t to make r12->0
  // Rz^t
  // c  s  0
  //-s  c  0
  // 0  0  1
  // 
  // r = sqrt(r12^2 + r22^2)
  // s = -r12/r
  // c = r22/r
  r = 1./sqrt(R[1]*R[1] + R[4]*R[4]);
  sz = -R[1] * r;
  cz  = R[4] * r;

  // then Rz^tR=>R
  Rtmp[0] = cz * R[0] + sz * R[3];
  Rtmp[1] = cz * R[1] + sz * R[4];
  Rtmp[2] = cz * R[2] + sz * R[5];

  Rtmp[3] = -sz * R[0] + cz * R[3];
  Rtmp[4] = -sz * R[1] + cz * R[4];
  Rtmp[5] = -sz * R[2] + cz * R[5];

  Rtmp[6] = R[6];
  Rtmp[7] = R[7];
  Rtmp[8] = R[8];

  // here R should be an identity matrix. if the diagonal elements are not positive, reverse the rotation angle by PI rad	// resolve 180 degree ambiguity
  // diag(R)[1] < 0 && diag(R)[2] < 0 -> multiply -1 to Rx^t's c/s values, and transpose Ry^t, Rz^t
  //  means cx = -Rx^t(1,1), sx = -Rx^t(1,2), cy = Ry^t(0,0), sy = Ry^t(0,2), cz = Rz^t(0,0), sz = Rz^t(1,0)
  // diag(R)[0] < 0 && diag(R)[2] < 0 -> multiply -1 to Ry^t's c/s values, and transpose Rz^t
  //  means cx = Rx^t(1,1), sx = Rx^t(1,2), cy = -Ry^t(0,0), sy = -Ry^t(2,0), cz = Rz^t(0,0), sz = Rz^t(1,0)
  // diag(R)[0] < 0 && diag(R)[1] < 0 -> multiply -1 to Rz^t's c_s values
  //  means cx = Rx^t(1,1), sx = Rx^t(1,2), cy = Ry^t(0,0), sy = Ry^t(2,0), cz = -Rz^t(0,0), sz = -Rz^t(0,1)
  // diag(R) are positive - no need to rotate
  //  means cx = Rx^t(1,1), sx = Rx^t(1,2), cy = Ry^t(0,0), sy = Ry^t(2,0), cz = Rz^t(0,0), sz = Rz^t(0,1)
  if(Rtmp[0] < 0){
    if(Rtmp[4] < 0){
      // z rotation + PI
      sz = -sz;
      cz = -sz;
    }else{
      // y rotation + PI
      sy = -sy;
      cy = -cy;

      // transpose z rotation
      sz = -sz;
    }
  }else{
    if(Rtmp[4] < 0){
      // x rotation + PI
      sx = -sx;
      sy = -sy;

      // transpose y rotation
      sy = -sy;
      // transpose z rotation
      sz = -sz;
    }
  }

  x = atan2(sx, cx);
  y = atan2(sy, cy);
  z = atan2(sz, cz);
}

void angleRzyx(double * R, double & x, double & y, double &z)
{
  // R
  // r11 r12 r13
  // r21 r22 r23
  // r31 r32 r33
  //
  // R0  R1  R2 
  // R3  R4  R5
  // R6  R7  R8
  // left multiplication decompose Rz^t Ry^t Rx^t I = R(because our rotation order is RxRyRz x)
  double r, cx, sx, cy, sy, cz, sz;
  double Rtmp[9];

  // Find Givens rotation Rz^t to make r12->0
  // Rz^t
  // c  s  0
  //-s  c  0
  // 0  0  1
  // 
  // r = sqrt(r12^2 + r22^2)
  // s = -r12/r
  // c = r22/r
  r = 1./sqrt(R[1]*R[1] + R[4]*R[4]);
  sz = -R[1] * r;
  cz  = R[4] * r;

  // then Rz^tR=>R
  Rtmp[0] = cz * R[0] + sz * R[3];
  Rtmp[1] = cz * R[1] + sz * R[4];
  Rtmp[2] = cz * R[2] + sz * R[5];

  Rtmp[3] = -sz * R[0] + cz * R[3];
  Rtmp[4] = -sz * R[1] + cz * R[4];
  Rtmp[5] = -sz * R[2] + cz * R[5];

  Rtmp[6] = R[6];
  Rtmp[7] = R[7];
  Rtmp[8] = R[8];


  // Find Givens rotation Ry^t to make r13->0
  // Ry^t
  // c  0 -s
  // 0  1  0
  // s  0  c
  //
  // r= sqrt(r13^2+r33^2)
  // s = r13/r
  // c = r33/r
  r = 1./sqrt(Rtmp[2] * Rtmp[2] + Rtmp[8] * Rtmp[8]);
  sy = Rtmp[2] * r;
  cy = Rtmp[8] * r;

  // then Ry^tR=>R
  R[0] = cy * Rtmp[0] - sy * Rtmp[6];
  R[1] = cy * Rtmp[1] - sy * Rtmp[7];
  R[2] = cy * Rtmp[2] - sy * Rtmp[8];

  R[3] = Rtmp[3];
  R[4] = Rtmp[4];
  R[5] = Rtmp[5];

  R[6] = sy * Rtmp[0] + cy * Rtmp[6];
  R[7] = sy * Rtmp[1] + cy * Rtmp[7];
  R[8] = sy * Rtmp[2] + cy * Rtmp[8];

  // Find Givens rotation Rx^t to make r23->0
  // Rx^t
  // 1  0  0
  // 0  c  s
  // 0 -s  c
  //
  // r = sqrt(r23^2+r33^2)
  // s = -r23/r
  // c = r33/r
  r = 1./sqrt(R[5]*R[5] + R[8]*R[8]);
  sx = -R[5]  * r;
  cx = R[8] * r;

  // then Rx^tR=>R
  Rtmp[0] = R[0];
  Rtmp[1] = R[1];
  Rtmp[2] = R[2];

  Rtmp[3] = cx * R[3] + sx * R[6];
  Rtmp[4] = cx * R[4] + sx * R[7];
  Rtmp[5] = cx * R[5] + sx * R[8];

  Rtmp[6] = -sx * R[3] + cx * R[6];
  Rtmp[7] = -sx * R[4] + cx * R[7];
  Rtmp[8] = -sx * R[5] + cx * R[8];

  // here R should be an identity matrix. if the diagonal elements are not positive, reverse the rotation angle by PI rad	// resolve 180 degree ambiguity
  // diag(R)[1] < 0 && diag(R)[2] < 0 -> multiply -1 to Rx^t's c/s values, and transpose Ry^t, Rz^t
  //  means cx = -Rx^t(1,1), sx = -Rx^t(1,2), cy = Ry^t(0,0), sy = Ry^t(0,2), cz = Rz^t(0,0), sz = Rz^t(1,0)
  // diag(R)[0] < 0 && diag(R)[2] < 0 -> multiply -1 to Ry^t's c/s values, and transpose Rz^t
  //  means cx = Rx^t(1,1), sx = Rx^t(1,2), cy = -Ry^t(0,0), sy = -Ry^t(2,0), cz = Rz^t(0,0), sz = Rz^t(1,0)
  // diag(R)[0] < 0 && diag(R)[1] < 0 -> multiply -1 to Rz^t's c_s values
  //  means cx = Rx^t(1,1), sx = Rx^t(1,2), cy = Ry^t(0,0), sy = Ry^t(2,0), cz = -Rz^t(0,0), sz = -Rz^t(0,1)
  // diag(R) are positive - no need to rotate
  //  means cx = Rx^t(1,1), sx = Rx^t(1,2), cy = Ry^t(0,0), sy = Ry^t(2,0), cz = Rz^t(0,0), sz = Rz^t(0,1)
  if(Rtmp[0] < 0){
    if(Rtmp[4] < 0){
      // z rotation + PI
      sz = -sz;
      cz = -sz;
    }else{
      // y rotation + PI
      sy = -sy;
      cy = -cy;

      // transpose z rotation
      sz = -sz;
    }
  }else{
    if(Rtmp[4] < 0){
      // x rotation + PI
      sx = -sx;
      sy = -sy;

      // transpose y rotation
      sy = -sy;
      // transpose z rotation
      sz = -sz;
    }
  }

  x = atan2(sx, cx);
  y = atan2(sy, cy);
  z = atan2(sz, cz);
}

////////////////////////////////////////////////////////////////////////// Lie-group to Lie-algebra mapping function
bool test_exp_so3(const double * r, double * Rcv, double * jRcv)
{
  // R and jRcv are the OpenCV's rotation matrix and jacobian.
  double jR[27];
  double errjR[27];
  double R[9];
  double errR[9];
  exp_so3(r, R, jR);
  memset((void*)errjR, 0, sizeof(errjR));
  memset((void*)errR, 0, sizeof(errR));

  // jacobian is [dR/drx, dR/dry, dR/drz]
  // Note: OpenCV's jacobian assumes stacked row vector
  // exp_so3's jacobian assumes stacked column vector (if evaluated at zero, both are the same.)
  for(int i = 0; i < 9; i++){
    errR[i] = rerr(R[i], Rcv[i]);
  }

  for (int i = 0; i < 9; i++){
    for(int j = 0; j < 3; j++){
      errjR[i*3 + j] = rerr(jR[i*3 + j], jRcv[j*9+i]);
    }
  }

  bool res = true;
  for(int i = 0; i < 27; i++){
    if(errjR[i] > 0.1){
      cerr << "Jacobian[" << i << "] exp_so3 is erroneous." << endl;
      res = false;
    }
    jRcv[i] = jR[i];
  }

  for(int j = 0; j < 9; j++){
    if(errR[j] > 0.001){
      cerr << "R[" << j << "] is erroneous." << endl;
      res = false;
    }
  }

  return res;
}

//////////////////////////////////////////////////////////////////////////////////////////////// Projection Function
bool test_prjPtsj(cv::Mat & camint, cv::Mat & camdist, cv::Mat & rvec, cv::Mat & tvec, 
		  vector<cv::Point3f> & pt3d, vector<int> & valid, cv::Mat & jacobian, double arf)
{
  int neq = (int) pt3d.size() * 2;
  vector<cv::Point2f> pt2d(pt3d.size());
  double * jf, * jc, * jk, * jp, * jr, * jt;
  jf = new double [neq * 2];
  jc = new double [neq * 2];
  jk = new double [neq * 6];
  jp = new double [neq * 2];
  jr = new double [neq * 3];
  jt = new double [neq * 3];

  prjPts(pt3d, pt2d, valid, camint, camdist, rvec, tvec,
	 jf, jc, jk, jp, jr, jt, arf);
  // comparation with OpenCV's jacobian
  // OpenCV's jacobian has cols | rotation | translation | focal length | principal point | distortion coefficient |
  //                            |     3    |      3      |       2      |        2        |            8           |
  //                                                                                      |  2   |  2  |    4      |
  //                                                                                      | k1 k2 p1 p2 k3 k4 k5 k6
  int row_step = sizeof(double) * 18;
  double * jcv = jacobian.ptr<double>();
  double err[18] = {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};

  double * _jf = jf, * _jc = jc, * _jk = jk, * _jp = jp, * _jr = jr, * _jt = jt;

  bool res = true;

  for(int i = 0; i < neq; i++){
    if(!valid[i/2]){
      _jf += (arf == 0.0 ? 2 : 1);
      _jc += 2;
      _jk += 6;
      _jp += 2;
      _jr += 3;
      _jt += 3;
      jcv += 18;
      continue;
    }

    // focal length
    if(arf == 0.0){
      err[6] = rerr(_jf[0], jcv[6]);
      err[7] = rerr(_jf[1], jcv[7]);
    }else{
      err[6] = err[7] = rerr(_jf[0], jcv[7]);
    }
    // principal point
    err[8] = rerr(_jc[0], jcv[8]);
    err[9] = rerr(_jc[1], jcv[9]);

    // distortion coefficient
    err[10] = rerr(_jk[0], jcv[10]);
    err[11] = rerr(_jk[1], jcv[11]);
    err[12] = rerr(_jp[0], jcv[12]);
    err[13] = rerr(_jp[1], jcv[13]);
    err[14] = rerr(_jk[2], jcv[14]);
    err[15] = rerr(_jk[3], jcv[15]);
    err[16] = rerr(_jk[4], jcv[16]);
    err[17] = rerr(_jk[5], jcv[17]);

    err[0] = rerr(_jr[0], jcv[0]);
    err[1] = rerr(_jr[1], jcv[1]);
    err[2] = rerr(_jr[2], jcv[2]);

    err[3] = rerr(_jt[0], jcv[3]);
    err[4] = rerr(_jt[1], jcv[4]);
    err[5] = rerr(_jt[2], jcv[5]);

    for(int j = 0; j < 18; j++){
      if(err[j] > 0.1){
	cerr << "Jacobian parameter " << j << " in prjPts is erroneous." << endl;
	res = false;
      }
    }
    _jf += 2;
    _jc += 2;
    _jk += 6;
    _jp += 2;
    _jr += 3;
    _jt += 3;
    jcv += 18;
  }

  delete[] jf;
  delete[] jc;
  delete[] jk;
  delete[] jp;
  delete[] jr;
  delete[] jt;

  return res;
}


void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m,
	    const cv::Mat & camint, const cv::Mat & camdist, const cv::Mat & rvec, const cv::Mat & tvec)
{
  const double * R, * t, * k;
  cv::Mat _R;
  Rodrigues(rvec, _R);
  R = _R.ptr<double>();
  t = tvec.ptr<double>();
  k = camdist.ptr<double>();
  const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
    cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

  m.resize(M.size());

  for(int i = 0; i < M.size(); i++){
    prjPt(M[i], m[i], fx, fy, cx, cy, k, R, t);
  }
}

void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m,
	    const double fx, const double fy, const double cx, const double cy,
	    const double * k, const double * R, const double * t)
{
  m.resize(M.size());

  for(int i = 0; i < M.size(); i++){
    prjPt(M[i], m[i], fx, fy, cx, cy, k, R, t);
  }
}


void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const cv::Mat & camint, const cv::Mat & camdist, const cv::Mat & rvec, const cv::Mat & tvec)
{
  const double * R, * t, * k;
  cv::Mat _R;
  Rodrigues(rvec, _R);
  R = _R.ptr<double>();
  t = tvec.ptr<double>();
  k = camdist.ptr<double>();
  const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
    cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

  m.resize(M.size());

  for(int i = 0; i < M.size(); i++){
    if(!valid[i])
      continue;

    prjPt(M[i], m[i], fx, fy, cx, cy, k, R, t);
  }
}

void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const double fx, const double fy, const double cx, const double cy,
	    const double * k, const double * R, const double * t)
{
  m.resize(M.size());

  for(int i = 0; i < M.size(); i++){
    if(!valid[i])
      continue;
    prjPt(M[i], m[i], fx, fy, cx, cy, k, R, t);
  }
}

void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const cv::Mat & camint, const cv::Mat & camdist,
	    const cv::Mat & rvec, const cv::Mat & tvec,
	    double * jf, double * jc, double * jk, double * jp,
	    double * jr, double * jt, double arf)
{
  // if arf equals to 0.0, the jacobian for jf is calculated for fx and fy distinctively. (jf is 2Nx2 array)
  // Otherwise, jf is caluclated related only to fy. (jf is 2Nx1 array)

  const double * t, * k;

  //	double R[9], jR[27];
  cv::Mat _R, _jR;
  Rodrigues(rvec, _R, _jR);
  double *R = _R.ptr<double>(), *jR = _jR.ptr<double>();

  if(!test_exp_so3(rvec.ptr<double>(), _R.ptr<double>(), _jR.ptr<double>())){
    cerr << "exp_so3 may be erroneous." << endl;
  }

  t = tvec.ptr<double>();
  k = camdist.ptr<double>();
  const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
    cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

  if(M.size() != m.size())
    m.resize(M.size());

  for(int i = 0; i < M.size(); i++){
    if(!valid[i]){
      jf += (arf == 0.0 ? 4 : 2);
      jc += 4;
      jk += 12;
      jp += 4;
      jr += 6;
      jt += 6;
      continue;
    }

    double X = M[i].x, Y = M[i].y, Z = M[i].z;
    double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
    double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
    double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
    double r2, r4, r6, a1, a2, a3, cdist, icdist2, D, D2;
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
    D = cdist * icdist2;
    D2 = D * icdist2;
    xd = x * D + k[2]*a1 + k[3]*a2;
    yd = y * D + k[2]*a3 + k[3]*a1;

    m[i].x = (float)(xd*fx + cx);
    m[i].y = (float)(yd*fy + cy);

    // calculate jacboian for fx, fy ( this version is free aspect ratio)
    if(arf == 0.0){
      jf[0] = xd; jf[1] = 0;
      jf += 2;
      jf[0] = 0; jf[1] = yd;
      jf += 2;
    }else{
      jf[0] = xd * arf;
      jf += 1;
      jf[0] = yd;
      jf += 1;
    }

    // calculate jacobian for cx, cy
    jc[0] = 1; jc[1] = 0;
    jc += 2;
    jc[0] = 0; jc[1] = 1;
    jc += 2;

    // calculate jacobian for radial distortion coefficient
    jk[0] = fx * x * r2 * icdist2; 
    jk[1] = fx * x * r4 * icdist2; 
    jk[2] = fx * x * r6 * icdist2;
    jk[3] = -fx * x * r2 * D2;
    jk[4] = -fx * x * r4 * D2;
    jk[5] = -fx * x * r6 * D2;
    jk += 6;
    jk[0] = fy * y * r2 * icdist2; 
    jk[1] = fy * y * r4 * icdist2; 
    jk[2] = fy * y * r6 * icdist2;
    jk[3] = -fy * y * r2 * D2;
    jk[4] = -fy * y * r4 * D2;
    jk[5] = -fy * y * r6 * D2;
    jk += 6;

    // calculate jacobian for tangential distortion coefficient
    jp[0] = fx * a1; jp[1] = fx * a2;
    jp += 2;
    jp[0] = fy * a3; jp[1] = fy * a1;
    jp += 2;

    double dDdl2 = (3 * k[4] * r4 + 2 * k[1] * r2 + k[0]) * icdist2 
      - (3 * k[7] * r4 + 2 * k[6] * r2 + k[5]) * D2;
    double dl2dxp = 2*x;
    double dl2dyp = 2*y;

    double dxdz = -x * z; // here x and y has already been mutiplied with 1/z. And note that z is actually 1/z here.
    double dydz = -y * z;

    // calculate jacobian for translation
    double dxdxp = fx * (D + x * dDdl2 * dl2dxp + 2 * k[2] * y + k[3] * (dl2dxp + 4 * x));
    double dxdyp = fx * (x * dDdl2 * dl2dyp + 2 * k[2] * x + k[3] * dl2dyp);
    double dydxp = fy * (y * dDdl2 * dl2dxp + 2 * k[3] * y + k[2] * dl2dxp);
    double dydyp = fy * (D + y * dDdl2 * dl2dxp + 2 * k[3] * x + k[2] * (dl2dyp + 4 * y));
    jt[0] = dxdxp * z; jt[1] = dxdyp * z; jt[2] = dxdxp * dxdz + dxdyp * dydz;
    jt[3] = dydxp * z; jt[4] = dydyp * z; jt[5] = dydxp * dxdz + dydyp * dydz;

    // calculate jacobian for rotation 
    // [ dm/jt = dm/dM ] [ dM/jr ]
    // [dx/jt][ dX/dr ]
    // [dy/jt][ dY/dr ]
    //        [ dZ/dr ]

    double dXdr[3] = {
      X * jR[0] + Y * jR[3] + Z * jR[6], 
      X * jR[1] + Y * jR[4] + Z * jR[7], 
      X * jR[2] + Y * jR[5] + Z * jR[8]
    };

    double dYdr[3] = {
      X * jR[9] +  Y * jR[12] + Z * jR[15], 
      X * jR[10] + Y * jR[13] + Z * jR[16], 
      X * jR[11] + Y * jR[14] + Z * jR[17]
    };

    double dZdr[3] = {
      X * jR[18] + Y * jR[21] + Z * jR[24], 
      X * jR[19] + Y * jR[22] + Z * jR[25], 
      X * jR[20] + Y * jR[23] + Z * jR[26]
    };
    /*
      for( int j = 0; j < 3; j++ )
      {
      double dxdr = z*(dXdr[j] - x*dZdr[j]);
      double dydr = z*(dYdr[j] - y*dZdr[j]);
      double dr2dr = 2*x*dxdr + 2*y*dydr;
      double dcdist_dr = k[0]*dr2dr + 2*k[1]*r2*dr2dr + 3*k[4]*r4*dr2dr;
      double dicdist2_dr = -icdist2*icdist2*(k[5]*dr2dr + 2*k[6]*r2*dr2dr + 3*k[7]*r4*dr2dr);
      double da1dr = 2*(x*dydr + y*dxdr);
      double dmxdr = fx*(dxdr*cdist*icdist2 + x*dcdist_dr*icdist2 + x*cdist*dicdist2_dr +
      k[2]*da1dr + k[3]*(dr2dr + 2*x*dxdr));
      double dmydr = fy*(dydr*cdist*icdist2 + y*dcdist_dr*icdist2 + y*cdist*dicdist2_dr +
      k[2]*(dr2dr + 2*y*dydr) + k[3]*da1dr);
      jr[j] = dmxdr;
      jr[j+3] = dmydr;
      }
      /*
      double dxdrx, dydrx, dxdry, dydry, dxdrz, dydrz;
      dxdrx = z * (dXdr[0] - x * dZdr[0]);
      dxdry = z * (dXdr[1] - x * dZdr[1]);
      dxdrz = z * (dXdr[2] - x * dZdr[2]);

      dydrx = z * (dYdr[0] - y * dZdr[0]);
      dydry = z * (dYdr[1] - y * dZdr[1]);
      dydrz = z * (dYdr[2] - y * dZdr[2]);

      double dl2drx, dl2dry, dl2drz;
      dl2drx = dl2dxp * dxdrx + dl2dyp * dydrx;
      dl2dry = dl2dxp * dxdry + dl2dyp * dydry;
      dl2drz = dl2dxp * dxdrz + dl2dyp * dydrz;

      jr[0] = fx * (
      D * dxdrx 
      + x * dDdl2 * dl2drx 
      + 2 * k[2] * (x * dxdrx + y * dydrx) 
      + k[3] * (dl2drx + 2 * x * dxdrx));
      jr[3] = fy * (
      D * dxdrx 
      + x * dDdl2 * dl2drx 
      + 2 * k[3] * (x * dxdrx + y * dydrx) 
      + k[2] * (dl2drx + 2 * y * dydrx));
      jr[1] = fx * (
      D * dxdry 
      + x * dDdl2 * dl2dry 
      + 2 * k[2] * (x * dxdry + y * dydry) 
      + k[3] * (dl2dry + 2 * x * dxdry));
      jr[4] = fy * (
      D * dxdry 
      + x * dDdl2 * dl2dry 
      + 2 * k[3] * (x * dxdry + y * dydry) 
      + k[2] * (dl2dry + 2 * y * dydry));
      jr[2] = fx * (
      D * dxdrz 
      + x * dDdl2 * dl2drz 
      + 2 * k[2] * (x * dxdrz + y * dydrz) 
      + k[3] * (dl2drz + 2 * x * dxdrz));
      jr[5] = fy * (
      D * dxdrz 
      + x * dDdl2 * dl2drz 
      + 2 * k[3] * (x * dxdrz + y * dydrz) 
      + k[2] * (dl2drz + 2 * y * dydrz));
    */
		
    jr[0] = jt[0] * dXdr[0] + jt[1] * dYdr[0] + jt[2] * dZdr[0];
    jr[1] = jt[0] * dXdr[1] + jt[1] * dYdr[1] + jt[2] * dZdr[1];
    jr[2] = jt[0] * dXdr[2] + jt[1] * dYdr[2] + jt[2] * dZdr[2];

    jr[3] = jt[3] * dXdr[0] + jt[4] * dYdr[0] + jt[5] * dZdr[0];
    jr[4] = jt[3] * dXdr[1] + jt[4] * dYdr[1] + jt[5] * dZdr[1];
    jr[5] = jt[3] * dXdr[2] + jt[4] * dYdr[2] + jt[5] * dZdr[2];
		
    jr += 6;
    jt += 6;
  }
};


void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const cv::Mat & camint, const cv::Mat & camdist,
	    const cv::Mat & rvec, const cv::Mat & tvec,
	    double * jr, double * jt)
{
  const double * t, * k;

  double R[9], jR[27];
  //Rodrigues(rvec, _R, _jR);
  exp_so3(rvec.ptr<double>(), R, jR);

  t = tvec.ptr<double>();
  k = camdist.ptr<double>();
  const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
    cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

  if(M.size() != m.size())
    m.resize(M.size());

  for(int i = 0; i < M.size(); i++){
    if(!valid[i]){
      jr += 6;
      jt += 6;
      continue;
    }

    double X = M[i].x, Y = M[i].y, Z = M[i].z;
    double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
    double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
    double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
    double r2, r4, r6, a1, a2, a3, cdist, icdist2, D, D2;
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
    D = cdist * icdist2;
    D2 = D * icdist2;
    xd = x * D + k[2]*a1 + k[3]*a2;
    yd = y * D + k[2]*a3 + k[3]*a1;

    m[i].x = (float)(xd*fx + cx);
    m[i].y = (float)(yd*fy + cy);

    double dDdl2 = (3 * k[4] * r4 + 2 * k[1] * r2 + k[0]) * icdist2 
      - (3 * k[7] * r4 + 2 * k[6] * r2 + k[5]) * D2;
    double dl2dxp = 2*x;
    double dl2dyp = 2*y;

    double dxdz = -x * z; // here x and y has already been mutiplied with 1/z. And note that z is actuall 1/z here.
    double dydz = -y * z;

    // calculate jacobian for translation
    double dxdxp = fx * (D + x * dDdl2 * dl2dxp + 2 * k[2] * y + k[3] * (dl2dxp + 4 * x));
    double dxdyp = fx * (x * dDdl2 * dl2dyp + 2 * k[2] * x + k[3] * dl2dyp);
    double dydxp = fy * (y * dDdl2 * dl2dxp + 2 * k[3] * y + k[2] * dl2dxp);
    double dydyp = fy * (D + y * dDdl2 * dl2dxp + 2 * k[3] * x + k[2] * (dl2dyp + 4 * y));
    jt[0] = dxdxp * z; jt[1] = dxdyp * z; jt[2] = dxdxp * dxdz + dxdyp * dydz;
    jt[3] = dydxp * z; jt[4] = dydyp * z; jt[5] = dydxp * dxdz + dydyp * dydz;

    // calculate jacobian for rotation 
    double dXdr[3] = {
      X * jR[0] + Y * jR[3] + Z * jR[6], 
      X * jR[1] + Y * jR[4] + Z * jR[7], 
      X * jR[2] + Y * jR[5] + Z * jR[8]
    };

    double dYdr[3] = {
      X * jR[9] +  Y * jR[12] + Z * jR[15], 
      X * jR[10] + Y * jR[13] + Z * jR[16], 
      X * jR[11] + Y * jR[14] + Z * jR[17]
    };

    double dZdr[3] = {
      X * jR[18] + Y * jR[21] + Z * jR[24], 
      X * jR[19] + Y * jR[22] + Z * jR[25], 
      X * jR[20] + Y * jR[23] + Z * jR[26]
    };

    jr[0] = jt[0] * dXdr[0] + jt[1] * dYdr[0] + jt[2] * dZdr[0];
    jr[1] = jt[0] * dXdr[1] + jt[1] * dYdr[1] + jt[2] * dZdr[1];
    jr[2] = jt[0] * dXdr[2] + jt[1] * dYdr[2] + jt[2] * dZdr[2];

    jr[3] = jt[3] * dXdr[0] + jt[4] * dYdr[0] + jt[5] * dZdr[0];
    jr[4] = jt[3] * dXdr[1] + jt[4] * dYdr[1] + jt[5] * dZdr[1];
    jr[5] = jt[3] * dXdr[2] + jt[4] * dYdr[2] + jt[5] * dZdr[2];
    jr += 6;
    jt += 6;
  }
};


void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m,
	    const cv::Mat & camint, const cv::Mat & rvec, const cv::Mat & tvec)
{
  const double * R, * t;
  cv::Mat _R;
  Rodrigues(rvec, _R);
  R = _R.ptr<double>();
  t = tvec.ptr<double>();
  const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
    cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

  m.resize(M.size());

  for(int i = 0; i < M.size(); i++){
    prjPt(M[i], m[i], fx, fy, cx, cy, R, t);
  }
}


void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m,
	    const double fx, const double fy, const double cx, const double cy, const double * R, const double * t)
{
  m.resize(M.size());

  for(int i = 0; i < M.size(); i++){
    prjPt(M[i], m[i], fx, fy, cx, cy, R, t);
  }
}

void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const cv::Mat & camint,	const cv::Mat & rvec, const cv::Mat & tvec)
{
  const double * R, * t;
  cv::Mat _R;
  Rodrigues(rvec, _R);
  R = _R.ptr<double>();
  t = tvec.ptr<double>();
  const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
    cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

  m.resize(M.size());

  for(int i = 0; i < M.size(); i++){
    if(!valid[i])
      continue;

    prjPt(M[i], m[i], fx, fy, cx, cy, R, t);
  }
}

void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const double fx, const double fy, const double cx, const double cy, const double * R, const double * t)
{
  m.resize(M.size());

  for(int i = 0; i < M.size(); i++){
    if(!valid[i])
      continue;

    prjPt(M[i], m[i], fx, fy, cx, cy, R, t);
  }
}

void prjPts(const vector<cv::Point3f> & M, vector<cv::Point2f> & m, const vector<int> & valid,
	    const cv::Mat & camint,	const cv::Mat & rvec, const cv::Mat & tvec,
	    double * jr, double * jt)
{
  const double * t;

  double R[9], jR[27];
  //Rodrigues(rvec, _R, _jR);
  exp_so3(rvec.ptr<double>(), R, jR);

  t = tvec.ptr<double>();
  const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
    cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

  if(M.size() != m.size())
    m.resize(M.size());

  for(int i = 0; i < M.size(); i++){
    if(!valid[i]){
      jr += 6;
      jt += 6;
      continue;
    }

    double X = M[i].x, Y = M[i].y, Z = M[i].z;
    double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
    double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
    double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];

    z = z ? 1./z : 1;
    x *= z; y *= z;
		
    m[i].x = (float)(x*fx + cx);
    m[i].y = (float)(y*fy + cy);

    jt[0] = fx * z; jt[1] = 0; jt[2] = -fx * x * z;
    jt[3] = 0; jt[4] = fy * z; jt[5] = -fy * y * z;

    // calculate jacobian for rotation 
    double dXdr[3] = {
      X * jR[0] + Y * jR[3] + Z * jR[6], 
      X * jR[1] + Y * jR[4] + Z * jR[7], 
      X * jR[2] + Y * jR[5] + Z * jR[8]
    };

    double dYdr[3] = {
      X * jR[9] +  Y * jR[12] + Z * jR[15], 
      X * jR[10] + Y * jR[13] + Z * jR[16], 
      X * jR[11] + Y * jR[14] + Z * jR[17]
    };

    double dZdr[3] = {
      X * jR[18] + Y * jR[21] + Z * jR[24], 
      X * jR[19] + Y * jR[22] + Z * jR[25], 
      X * jR[20] + Y * jR[23] + Z * jR[26]
    };

    jr[0] = jt[0] * dXdr[0] + jt[1] * dYdr[0] + jt[2] * dZdr[0];
    jr[1] = jt[0] * dXdr[1] + jt[1] * dYdr[1] + jt[2] * dZdr[1];
    jr[2] = jt[0] * dXdr[2] + jt[1] * dYdr[2] + jt[2] * dZdr[2];

    jr[3] = jt[3] * dXdr[0] + jt[4] * dYdr[0] + jt[5] * dZdr[0];
    jr[4] = jt[3] * dXdr[1] + jt[4] * dYdr[1] + jt[5] * dZdr[1];
    jr[5] = jt[3] * dXdr[2] + jt[4] * dYdr[2] + jt[5] * dZdr[2];
    jr += 6;
    jt += 6;
  }
}

void trnPts(const vector<cv::Point3f> & Msrc, vector<cv::Point3f> & Mdst, const vector<int> & valid, 
	    const cv::Mat & R, const cv::Mat & t)
{
  const double * pR = R.ptr<double>(), * pt = t.ptr<double>();
  if(Msrc.size() != Mdst.size())
    Mdst.resize(Msrc.size());

  for(int i = 0; i < Msrc.size(); i++){
    if(!valid[i])
      continue;

    double X = Msrc[i].x, Y = Msrc[i].y, Z = Msrc[i].z;
    Mdst[i].x = (float)(pR[0]*X + pR[1]*Y + pR[2]*Z + pt[0]);
    Mdst[i].y = (float)(pR[3]*X + pR[4]*Y + pR[5]*Z + pt[1]);
    Mdst[i].z = (float)(pR[6]*X + pR[7]*Y + pR[8]*Z + pt[2]);
  }
}

void trnPts(const vector<cv::Point3f> & Msrc, vector<cv::Point3f> & Mdst,
	    const cv::Mat & R, const cv::Mat & t)
{
  const double * pR = R.ptr<double>(), * pt = t.ptr<double>();
  if(Msrc.size() != Mdst.size())
    Mdst.resize(Msrc.size());

  for(int i = 0; i < Msrc.size(); i++){
    double X = Msrc[i].x, Y = Msrc[i].y, Z = Msrc[i].z;
    Mdst[i].x = (float)(pR[0]*X + pR[1]*Y + pR[2]*Z + pt[0]);
    Mdst[i].y = (float)(pR[3]*X + pR[4]*Y + pR[5]*Z + pt[1]);
    Mdst[i].z = (float)(pR[6]*X + pR[7]*Y + pR[8]*Z + pt[2]);
  }
}

void prjPts(const vector<cv::Point3f> & Mcam, vector<cv::Point2f> & m, const vector<int> & valid, const cv::Mat & camint)
{
  const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
    cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

  for(int i = 0; i < Mcam.size(); i++){
    if(!valid[i])
      continue;

    double x = Mcam[i].x;
    double y = Mcam[i].y;
    double z = Mcam[i].z;
    z = z ? 1./z : 1;
    x *= z; y *= z;
		
    m[i].x = (float)(x * fx + cx);
    m[i].y = (float)(y * fy + cy);
  }
}

void prjPts(const vector<cv::Point3f> & Mcam, vector<cv::Point2f> & m, const vector<int> & valid, 
	    const double fx, const double fy, const double cx, const double cy)
{
  for(int i = 0; i < Mcam.size(); i++){
    if(!valid[i])
      continue;

    double x = Mcam[i].x;
    double y = Mcam[i].y;
    double z = Mcam[i].z;
    z = z ? 1./z : 1;
    x *= z; y *= z;
		
    m[i].x = (float)(x * fx + cx);
    m[i].y = (float)(y * fy + cy);
  }
}

//////////////////////////////////////////////////////////////////////////////////////// Bayer pattern handling.
// R G
// G B
void cnvBayerRG8ToBGR8(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_8UC1)
    return;

  dst = cv::Mat(src.rows - 2, src.cols - 2, CV_8UC3);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
  int next_skip = step_size - src.cols + 2;;

  unsigned char * psrc0, * psrc1, * psrc2, * pdst;
  psrc0 = (unsigned char*) src.data + 1;
  psrc1 = psrc0 + step_size;
  psrc2 = psrc1 + step_size;
  pdst = (unsigned char*) dst.data;

  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ // even y
      //even x
      //blue
      // y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]+(unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 2);
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short)psrc0[-1]+(unsigned short)psrc0[+1]+(unsigned short)psrc2[-1]+(unsigned short)psrc2[+1]) >> 2);
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]) >> 1);
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 1);
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    y++;
    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;

    for(int x = 0; x < dst.cols; x++){ // odd y
      //even x
      //blue
      // y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 1);
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]) >> 1);
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short)psrc0[-1]+(unsigned short)psrc0[+1]+(unsigned short)psrc2[-1]+(unsigned short)psrc2[+1]) >> 2);
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]+(unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 2);
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;
  }
}

void cnvBayerRG16ToBGR16(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_16UC1)
    return;

  dst = cv::Mat(src.rows - 2, src.cols - 2, CV_16UC3);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
  int next_skip = step_size - src.cols + 2;

  unsigned short * psrc0, * psrc1, * psrc2, * pdst;
  psrc0 = (unsigned short*) src.data + 1;
  psrc1 = psrc0 + step_size;
  psrc2 = psrc1 + step_size;
  pdst = (unsigned short*) dst.data;

  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ // even y
      //even x
      //blue
      // y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]+(unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 2);
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short)psrc0[-1]+(unsigned short)psrc0[+1]+(unsigned short)psrc2[-1]+(unsigned short)psrc2[+1]) >> 2);
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]) >> 1);
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 1);
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    y++;
    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;

    for(int x = 0; x < dst.cols; x++){ // odd y
      //even x
      //blue
      // y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 1);
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]) >> 1);
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short) psrc0[-1]+(unsigned short) psrc0[+1]+(unsigned short) psrc2[-1]+(unsigned short) psrc2[+1]) >> 2);
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]+(unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 2);
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;
  }
}


// G R
// B G
void cnvBayerGR8ToBGR8(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_8UC1)
    return;

  dst = cv::Mat(src.rows - 2, src.cols - 2, CV_8UC3);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
  int next_skip = step_size - src.cols + 2;

  unsigned char * psrc0, * psrc1, * psrc2, * pdst;
  psrc0 = (unsigned char*) src.data + 1;
  psrc1 = psrc0 + step_size;
  psrc2 = psrc1 + step_size;
  pdst = (unsigned char*) dst.data;

  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ // even y
      //even x
      //blue
      // y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]) >> 1);
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 1);
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]+(unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 2);
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short)psrc0[-1]+(unsigned short)psrc0[+1]+(unsigned short)psrc2[-1]+(unsigned short)psrc2[+1]) >> 2);
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    y++;
    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;

    for(int x = 0; x < dst.cols; x++){ // odd y
      //even x
      //blue
      // y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short)psrc0[-1]+(unsigned short)psrc0[+1]+(unsigned short)psrc2[-1]+(unsigned short)psrc2[+1]) >> 2);
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]+(unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 2);
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc0[0]+(unsigned short)psrc2[0]) >> 1);
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (unsigned char)(((unsigned short)psrc1[-1]+(unsigned short)psrc1[+1]) >> 1);
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;
  }
}

// nearest neighbour
void cnvBayerGR8ToBGR8NN(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_8UC1)
    return;

  dst = cv::Mat(src.rows - 2, src.cols - 2, CV_8UC3);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
  int next_skip = step_size - src.cols + 2;

  unsigned char * psrc0, * psrc1, * pdst;
  psrc0 = (unsigned char*) src.data;
  psrc1 = psrc0 + step_size;
  pdst = (unsigned char*) dst.data;

  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ // even y
      // even x
      // src0: G R
      // src1: B G
      // Blue 
      *pdst = *psrc1;
      psrc1++;
      pdst++;

      // Green 
      *pdst = (unsigned char)(((unsigned short)*psrc0 + (unsigned short)*psrc1) >> 1);
      psrc0++;
      pdst++;

      // Red
      *pdst = *(psrc0);
      pdst++;

      x++;
      // odd x
      // src1: G R
      // src2: B G
      // Blue
      *pdst = *(psrc1 + 1);
      pdst++;

      // Green
      *pdst = (unsigned char)(((unsigned short)*psrc1 + (unsigned short)*(psrc0+1)) >> 1);
      pdst++;

      // Red
      *pdst = *psrc0;
      pdst++;

      psrc0++;
      psrc1++;
    }
    y++;
    psrc0 += next_skip;
    psrc1 += next_skip;

    for(int x = 0; x < dst.cols; x++){ // odd y
      // even x
      // src0: B G B G ...
      // src1: G R G R ...
      // Blue 
      *pdst = *psrc0;
      pdst++;

      // Green 
      psrc0++;
      *pdst = (unsigned char)(((unsigned short)*psrc1 + (unsigned short)*psrc0) >> 1);
      pdst++;

      // Red
      psrc1++;
      *pdst = *psrc1;
      *pdst++;

      x++;
      // odd x
      // src0: G B
      // src1: R G
      *pdst = *(psrc0 + 1);
      pdst++;

      // Green 
      *pdst = (unsigned char)(((unsigned short)*psrc0 + (unsigned short)*(psrc1 + 1)) >> 1);
      pdst++;

      // Red
      *pdst = *psrc1;
      *pdst++;

      psrc0++;
      psrc1++;
    }
    psrc0 += next_skip;
    psrc1 += next_skip;
  }
}

// quarter  
void cnvBayerGR8ToBGR8Q(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_8UC1)
    return;

  dst = cv::Mat(src.rows >> 1, src.cols >> 1, CV_8UC3);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
  int next_skip = step_size - src.cols;

  unsigned char * psrc0, * psrc1, * pdst;
  psrc0 = (unsigned char*) src.data;
  psrc1 = psrc0 + step_size;
  pdst = (unsigned char*) dst.data;

  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ 
      // src0: G R
      // src1: B G

      // Blue 
      *pdst = *psrc1;
      psrc1++;
      pdst++;

      // Green 
      *pdst = (unsigned char)(((unsigned short)*psrc0 + (unsigned short)*psrc1) >> 1);
      psrc0++;
      pdst++;

      // Red
      *pdst = *(psrc0);
      pdst++;

      psrc0++;
      psrc1++;
    }

    psrc0 = psrc1 + next_skip;
    psrc1 = psrc0 + step_size;
  }
}

// only green channel, quarter 
void cnvBayerGR8ToG8Q(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_8UC1)
    return;

  dst = cv::Mat(src.rows >> 1, src.cols >> 1, CV_8UC1);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
  int next_skip = step_size - src.cols;

  unsigned char * psrc0, * psrc1, * pdst;
  psrc0 = (unsigned char*) src.data;
  psrc1 = psrc0 + step_size + 1;
  pdst = (unsigned char*) dst.data;

  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ 
      // src0: G R
      // src1: B G
      // Green 
      *pdst = (unsigned char)(((unsigned short)*psrc0 + (unsigned short)*psrc1) >> 1);
      psrc0 +=2;
      psrc1 +=2;
      pdst++;
    }

    psrc0 = psrc1 + next_skip - 1;
    psrc1 = psrc0 + step_size + 1;
  }
}

// calculates the difference at B cells, and L1 norm is stored.
void cnvBayerGR8ToDG8Q(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_8UC1)
    return;

  dst = cv::Mat((src.rows >> 1) - 1, (src.cols >> 1) - 1, CV_8UC1);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
  int next_skip = step_size - src.cols + 2;

  unsigned char * psrc0, * psrc1, *psrc2, * pdst;
  psrc0 = (unsigned char*) src.data;
  psrc1 = psrc0 + step_size;
  psrc2 = psrc1 + step_size;
  psrc0 += 2;
  psrc1 += 1;
  psrc2 += 2;
  pdst = (unsigned char*) dst.data;

  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ 			
      // src0: G R G R
      // src1: B G B G
      // src2: G R G R

      // Green 
      short dx = (short)*psrc1;
      psrc1 += 2;
      dx = abs((short)*psrc1 - dx);
      short dy = abs((short)*psrc2 - (short)*psrc0);
      *pdst = (unsigned char)((dy + dx) >> 1);

      psrc0 += 2;
      psrc2 += 2;
      pdst++;
    }

    psrc0 = psrc1 + next_skip + 1;
    psrc1 = psrc0 + step_size - 1;
    psrc2 = psrc1 + step_size + 1;
  }
}

void cnvBayerGR16ToBGR16(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_16UC1)
    return;

  dst = cv::Mat(src.rows - 2, src.cols - 2, CV_16UC3);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
  int next_skip = step_size - src.cols + 2;

  unsigned short * psrc0, * psrc1, * psrc2, * pdst;
  psrc0 = (unsigned short*) src.data + 1;
  psrc1 = psrc0 + step_size;
  psrc2 = psrc1 + step_size;
  pdst = (unsigned short*) dst.data;

  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ // even y
      //even x
      //blue
      // y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (psrc1[-1]+psrc1[+1]) >> 1;
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (psrc0[0]+psrc2[0]) >> 1;
      pdst++;


      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    y++;
    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;

    for(int x = 0; x < dst.cols; x++){ // odd y
      //even x
      //blue
      // y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (psrc0[0]+psrc2[0]) >> 1;
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (psrc1[-1]+psrc1[+1]) >> 1;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;
  }
}

// B G
// G R
void cnvBayerBG8ToBGR8(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_8UC1)
    return;

  dst = cv::Mat(src.rows - 2, src.cols - 2, CV_8UC3);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
  int next_skip = step_size - src.cols + 2;

  unsigned char * psrc0, * psrc1, * psrc2, * pdst;
  psrc0 = (unsigned char*) src.data + 1;
  psrc1 = psrc0 + step_size;
  psrc2 = psrc1 + step_size;
  pdst = (unsigned char*) dst.data;

  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ // even y
      //even x
      //blue
      // y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (psrc0[0]+psrc2[0]) >> 1;
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (psrc1[-1]+psrc1[+1]) >> 1;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    y++;
    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;

    for(int x = 0; x < dst.cols; x++){ // odd y
      //even x
      //blue
      // y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (psrc1[-1]+psrc1[+1]) >> 1;
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (psrc0[0]+psrc2[0]) >> 1;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;
  }
}

void cnvBayerBG16ToBGR16(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_16UC1)
    return;

  dst = cv::Mat(src.rows - 2, src.cols - 2, CV_16UC3);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
  int next_skip = step_size - src.cols + 2;

  unsigned short * psrc0, * psrc1, * psrc2, * pdst;
  psrc0 = (unsigned short*) src.data + 1;
  psrc1 = psrc0 + step_size;
  psrc2 = psrc1 + step_size;
  pdst = (unsigned short*) dst.data;
  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ // even y
      //even x
      //blue
      // y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (psrc0[0]+psrc2[0]) >> 1;
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (psrc1[-1]+psrc1[+1]) >> 1;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    y++;
    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;

    for(int x = 0; x < dst.cols; x++){ // odd y
      //even x
      //blue
      // y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (psrc1[-1]+psrc1[+1]) >> 1;
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (psrc0[0]+psrc2[0]) >> 1;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;
  }
}

// G B
// R G
void cnvBayerGB8ToBGR8(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_8UC1)
    return;

  dst = cv::Mat(src.rows - 2, src.cols - 2, CV_8UC3);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
  int next_skip = step_size - src.cols + 2;

  unsigned char * psrc0, * psrc1, * psrc2, * pdst;
  psrc0 = (unsigned char*) src.data + 1;
  psrc1 = psrc0 + step_size;
  psrc2 = psrc1 + step_size;
  pdst = (unsigned char*) dst.data;

  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ // even y
      //even x
      //blue
      // y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (psrc0[0]+psrc2[0]) >> 1;
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (psrc1[-1]+psrc1[+1]) >> 1;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    y++;
    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;

    for(int x = 0; x < dst.cols; x++){ // odd y
      //even x
      //blue
      // y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (psrc1[-1]+psrc1[+1]) >> 1;
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (psrc0[0]+psrc2[0]) >> 1;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;
  }
}

void cnvBayerGB16ToBGR16(cv::Mat & src, cv::Mat & dst)
{
  if(src.type() != CV_16UC1)
    return;

  dst = cv::Mat(src.rows - 2, src.cols - 2, CV_16UC3);

  int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
  int next_skip = step_size - src.cols + 2;

  unsigned short * psrc0, * psrc1, * psrc2, * pdst;
  psrc0 = (unsigned short*) src.data + 1;
  psrc1 = psrc0 + step_size;
  psrc2 = psrc1 + step_size;
  pdst = (unsigned short*) dst.data;

  for(int y = 0; y < dst.rows; y++){
    for(int x = 0; x < dst.cols; x++){ // even y
      //even x
      //blue
      // y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (psrc0[0]+psrc2[0]) >> 1;
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (psrc1[-1]+psrc1[+1]) >> 1;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
      pdst++;
      //green
      // y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
      pdst++;
      //red
      // y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    y++;
    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;

    for(int x = 0; x < dst.cols; x++){ // odd y
      //even x
      //blue
      // y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
      *pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
      *pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
      x++;

      // odd x
      //blue
      // y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
      *pdst = (psrc1[-1]+psrc1[+1]) >> 1;
      pdst++;
      //green
      // y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
      *pdst = psrc1[0];
      pdst++;
      //red
      // y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
      *pdst = (psrc0[0]+psrc2[0]) >> 1;
      pdst++;

      psrc0++;
      psrc1++;
      psrc2++;
    }

    psrc0 += next_skip;
    psrc1 += next_skip;
    psrc2 += next_skip;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////// Transform related jacobian
void calcn_dR0t0Rtdrt(cv::Mat & J, double * R, double * t)
{
  J = cv::Mat::zeros(12, 6, CV_64FC1);
  double * p = J.ptr<double>();

  // First, the values at (r, t) = (0, 0) should be calculated.
  cv::Mat r1 = cv::Mat::zeros(3, 1, CV_64FC1);
  cv::Mat R1 = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Mat t1 = cv::Mat::zeros(3, 1, CV_64FC1);

  cv::Mat R3_0 = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Mat t3_0 = cv::Mat::zeros(3, 1, CV_64FC1);

  exp_so3(r1.ptr<double>(), R1.ptr<double>());
  compRt(R1.ptr<double>(), t1.ptr<double>(), R, t, R3_0.ptr<double>(), t3_0.ptr<double>());

  // Second, for each parameter, numerical difference is calculated.
  cv::Mat R3_d = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Mat t3_d = cv::Mat::zeros(3, 1, CV_64FC1);

  // derivative in r_i
  for(int i = 0; i < 3; i++){
    r1.ptr<double>()[i] = DIFF_STEP;
    exp_so3(r1.ptr<double>(), R1.ptr<double>());		
    compRt(R1.ptr<double>(), t1.ptr<double>(), R, t, R3_d.ptr<double>(), t3_d.ptr<double>());
		
    R3_d -= R3_0;
    t3_d -= t3_0;
		
    int idx = i;
    double * pR = R3_d.ptr<double>();
    for(int j = 0; j < 3; j++){
      for(int k = 0; k < 3; k++, idx += 6)
	p[idx] = pR[k * 3 + j] * IDIFF_STEP;
    }

    double * pt = t3_d.ptr<double>();
    for(int j = 0; j < 3; j++, idx += 6){
      p[idx] = pt[j] * IDIFF_STEP;
    }

    r1.ptr<double>()[i] = 0;
  }

  // derivative in t_i
  for(int i = 0; i < 3; i++){
    memcpy((void*)R3_d.ptr<double>(), (void*)R, sizeof(double) * 9);
    memcpy(t3_d.ptr<double>(), (void*)t, sizeof(double) * 3);

    t3_d.ptr<double>()[i] += DIFF_STEP;

    R3_d -= R3_0; 
    t3_d -= t3_0;

    // Actually, we don't need to calculate R3_d. Because it should always be zero here.
    int idx = i + 3;
    double * pR = R3_d.ptr<double>();
    for(int j = 0; j < 3; j++){
      for(int k = 0; k < 3; k++, idx += 6)
	p[idx] = pR[k * 3 + j] * IDIFF_STEP;
    }

    // Actually, we don't need to calculate it, because it should always be an identity matrix.
    double * pt = t3_d.ptr<double>();
    for(int j = 0; j < 3; j++, idx += 6){
      p[idx] = pt[j] * IDIFF_STEP;
    }	
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////// Miscellaneous 
// Synthesize two affine trasformations
// res = l * r 
bool synth_afn(cv::Mat & l, cv::Mat & r, cv::Mat & res)
{
  if(l.cols != 3 || r.cols != 3)
    return false;
  if(l.rows != 2 || r.rows != 2)
    return false;
  if(l.type() != r.type())
    return false;

  double * lptr, * rptr[2], * resptr;
  int irow, icol;
  res = cv::Mat::zeros(2, 3, CV_64FC1);

  rptr[0] = r.ptr<double>(0);
  rptr[1] = r.ptr<double>(1);
  for(irow = 0; irow < 2; irow++){
    lptr = l.ptr<double>(irow);
    resptr = res.ptr<double>(irow);
    for(icol = 0; icol < 3; icol++)
      resptr[icol] += lptr[0] * rptr[0][icol] + lptr[1] * rptr[1][icol];

    resptr[2] += lptr[2];
  }
  return true;
}

bool afn(cv::Mat & A, cv::Point2f & pt_in, cv::Point2f & pt_out)
{
  if(A.cols != 3 || A.rows != 2)
    return false;

  double *ptr;

  ptr = A.ptr<double>(0);
  pt_out.x = (float)(pt_in.x * ptr[0] + pt_in.y * ptr[1] + ptr[2]);
  ptr = A.ptr<double>(1);
  pt_out.y = (float)(pt_in.x * ptr[0] + pt_in.y * ptr[1] + ptr[2]);
  return true;
}

void layoutPyramid(const vector<cv::Mat> & IPyr, cv::Mat & out)
{
  int lvs = (int) IPyr.size();
  out = cv::Mat::zeros(IPyr[0].rows, IPyr[0].cols * 2 + lvs, IPyr[0].type());
  int icol = 0;
  for(int ilv = 0; ilv < lvs; ilv++){
    IPyr[ilv].copyTo(out(cv::Range(0, IPyr[ilv].rows), cv::Range(icol, icol + IPyr[ilv].cols)));
    icol += IPyr[ilv].cols;
  }
}

void cnv64FC1to8UC1(const cv::Mat & in, cv::Mat & out)
{
  if(in.type() != CV_64FC1){
    out = cv::Mat();
    return;
  }

  cv::MatConstIterator_<double> itr = in.begin<double>();
  cv::MatConstIterator_<double> itr_end = in.end<double>();

  double vmin = DBL_MAX, vmax = 0;

  for(;itr != itr_end; itr++){
    vmin = min(*itr, vmin);
    vmax = max(*itr, vmax);
  }

  double ivabs = 255.0 / (vmax - vmin);

  out = cv::Mat::zeros(in.rows, in.cols, CV_8UC1);

  cv::MatIterator_<uchar> itru = out.begin<uchar>();
  itr = in.begin<double>();
  for(;itr != itr_end; itr++, itru++){
    *itru = cv::saturate_cast<uchar>(ivabs* (*itr - vmin));
  }
}

void cnv32FC1to8UC1(const cv::Mat & in, cv::Mat & out)
{
  if(in.type() != CV_32FC1){
    out = cv::Mat();
    return;
  }

  cv::MatConstIterator_<float> itr = in.begin<float>();
  cv::MatConstIterator_<float> itr_end = in.end<float>();

  float vmin = FLT_MAX, vmax = 0;

  for(;itr != itr_end; itr++){
    vmin = min(*itr, vmin);
    vmax = max(*itr, vmax);
  }

  float ivabs = (float)(255.0 / (vmax - vmin));

  out = cv::Mat::zeros(in.rows, in.cols, CV_8UC1);

  cv::MatIterator_<uchar> itru = out.begin<uchar>();
  itr = in.begin<float>();
  for(;itr != itr_end; itr++, itru++){
    float v = (float)(ivabs* (*itr - vmin));
    *itru = cv::saturate_cast<uchar>(v);
  }
}

void cnv16UC1to8UC1(const cv::Mat & in, cv::Mat & out)
{
  if(in.type() != CV_16UC1){
    out = cv::Mat();
    return;
  }

  cv::MatConstIterator_<ushort> itr = in.begin<ushort>();
  cv::MatConstIterator_<ushort> itr_end = in.end<ushort>();

  ushort vmin = USHRT_MAX, vmax = 0;

  for(;itr != itr_end; itr++){
    vmin = min(*itr, vmin);
    vmax = max(*itr, vmax);
  }

  float ivabs = (float)(255.0 / (float) (vmax - vmin));

  out = cv::Mat::zeros(in.rows, in.cols, CV_8UC1);

  cv::MatIterator_<uchar> itru = out.begin<uchar>();
  itr = in.begin<ushort>();
  for(;itr != itr_end; itr++, itru++){
    float v = (float)(ivabs* (*itr - vmin));
    *itru = cv::saturate_cast<uchar>(v);
  }
}

void cnv16UC3to8UC3(const cv::Mat & in, cv::Mat & out)
{	
  if(in.type() != CV_16UC3){
    out = cv::Mat();
    return;
  }

  cv::MatConstIterator_<cv::Vec3w> itr = in.begin<cv::Vec3w>();
  cv::MatConstIterator_<cv::Vec3w> itr_end = in.end<cv::Vec3w>();

  ushort vmin[3] = {USHRT_MAX, USHRT_MAX, USHRT_MAX},
    vmax[3] = {0, 0, 0};

  for(;itr != itr_end; itr++){
    vmin[0] = min((*itr)[0], vmin[0]);
    vmax[0] = max((*itr)[0], vmax[0]);
    vmin[1] = min((*itr)[1], vmin[1]);
    vmax[1] = max((*itr)[1], vmax[1]);
    vmin[2] = min((*itr)[2], vmin[2]);
    vmax[2] = max((*itr)[2], vmax[2]);
  }

  float ivabs[3] = {
    (float)(255.0 / (float) (vmax[0] - vmin[0])),
    (float)(255.0 / (float) (vmax[1] - vmin[1])),
    (float)(255.0 / (float) (vmax[2] - vmin[2]))
  };

  out = cv::Mat::zeros(in.rows, in.cols, CV_8UC3);

  cv::MatIterator_<cv::Vec3b> itru = out.begin<cv::Vec3b>();
  itr = in.begin<cv::Vec3w>();
  for(;itr != itr_end; itr++, itru++){
    float v;
    v = (float)(ivabs[0]* ((*itr)[0] - vmin[0]));
    (*itru)[0] = cv::saturate_cast<uchar>(v);
    v = (float)(ivabs[1]* ((*itr)[1] - vmin[1]));
    (*itru)[1] = cv::saturate_cast<uchar>(v);
    v = (float)(ivabs[2]* ((*itr)[2] - vmin[2]));
    (*itru)[2] = cv::saturate_cast<uchar>(v);
  }
}

void cnvCVBGR8toGLRGB8(cv::Mat & img)
{
  uchar * ptr0, * ptr1;

  int cols = img.cols;
	
  for(int r0 = 0, r1 = img.rows - 1; r0 < r1; r0++, r1--)
    {
      ptr0 = img.ptr<uchar>(r0);
      ptr1 = img.ptr<uchar>(r1);
      for(int c = 0; c < cols; c++, ptr0+=3, ptr1+=3){
	uchar tmp;
	tmp = ptr0[0];
	ptr0[0] = ptr1[2];
	ptr1[2] = tmp;

	tmp = ptr0[1];
	ptr0[1] = ptr1[1];
	ptr1[1] = tmp;

	tmp = ptr0[2];
	ptr0[2] = ptr1[0];
	ptr1[0] = tmp;
      }
    }
}

inline void swapAndFlipElem(uchar * p1, uchar * p2, int chs, int szch)
{
  uchar t;
  p2 += szch * (chs - 1);
  for(int ich = 0; ich < chs && p1 != p2; ich++){
    for(int ibyte = 0; ibyte < szch; ibyte++){
      t = *p1;
      *p1 = *p2;
      *p2 = t;
    }
    p1 += szch;
    p2 -= szch;
  }
}

inline void swapElem(uchar * p1, uchar * p2, int szele)
{
  uchar t;
  for(int iele = 0; iele < szele; iele++, p1++, p2++){
    t = *p1;
    *p1 = *p2;
    *p2 = t;
  }
}

void awsFlip(cv::Mat & img, bool xflip, bool yflip, bool chflip)
{
  int chs = img.channels();
  int dpth = img.depth();
  int cols = img.cols;
  int hcols = cols >> 1;
  int rows = img.rows;
  int hrows = rows >> 1;
  int szele = (int) img.step[1];
  int szrow = (int) img.step[0];
  int nxrow = szrow - szele * cols;
  int szch = szele / chs;
  uchar * ptr0, * ptr1;

  //	if(cols & 0x1 || rows & 0x1)
  //		cerr << "awsFlip assumes that the image has even pixels in both vertical and horizontal direction." << endl;

  if(xflip){
    if(yflip){
      ptr0 = img.data;
      ptr1 = img.data + (rows - 1) * szrow + (cols - 1) * szele;
      if(chflip){
	for(int i = 0; i < hrows; i++){
	  for(int j = 0; j < cols; j++){						
	    swapAndFlipElem(ptr0, ptr1, chs, szch);
	    ptr0 += szele;
	    ptr1 -= szele;
	  }
	  ptr0 += nxrow;
	  ptr1 -= nxrow;
	}
      }else{
	for(int i = 0; i < hrows; i++){
	  for(int j = 0; j < cols; j++){
	    swapElem(ptr0, ptr1, szele);
	    ptr0 += szele;
	    ptr1 -= szele;
	  }
	  ptr0 += nxrow;
	  ptr1 -= nxrow;
	}
      }
    }else{
      ptr0 = img.data;
      ptr1 = img.data + (cols - 1) * szele;
      if(chflip){
	for(int i = 0; i < rows; i++){
	  for(int j = 0; j < hcols; j++){
	    swapAndFlipElem(ptr0, ptr1, chs, szch);						
	    ptr0 += szele;
	    ptr1 -= szele;
	  }
	  ptr0 += (szrow >> 1);
	  ptr1 = ptr0 + (cols - 1) * szele; 
	}
      }else{
	for(int i = 0; i < rows; i++){
	  for(int j = 0; j < hcols; j++){
	    swapElem(ptr0, ptr1, szele);
	    ptr0 += szele;
	    ptr1 -= szele;
	  }
	  ptr0 += (szrow >> 1);
	  ptr1 = ptr0 + (cols - 1) * szele; 
	}
      }
    }
  }else{
    if(yflip){
      ptr0 = img.data;
      ptr1 = img.data + (rows - 1) * szrow;
      if(chflip){
	for(int i = 0; i < hrows; i++){
	  for(int j = 0; j < cols; j++){
	    swapAndFlipElem(ptr0, ptr1, chs, szch);
	    ptr0 += szele;
	    ptr1 += szele;
	  }
	  ptr0 += nxrow;
	  ptr1 -= (szrow << 1);
	}
      }else{
	for(int i = 0; i < hrows; i++){
	  for(int j = 0; j < cols; j++){
	    swapElem(ptr0, ptr1, szele);
	    ptr0 += szele;
	    ptr1 += szele;
	  }
	  ptr0 += nxrow;
	  ptr1 -= (szrow << 1);
	}
      }
    }else{
      ptr0 = img.data;
      if(chflip){
	for(int i = 0; i < rows; i++){
	  for(int j = 0; j < cols; j++){
	    swapAndFlipElem(ptr0, ptr0, chs, szch);
	    ptr0 += szele;
	  }
	  ptr0 += nxrow;
	}
      }
    }
  }
}

void cnvCVGRAY8toGLGRAY8(cv::Mat & img)
{
  cv::Mat gl_img;
  gl_img.create(img.size(), CV_8UC3);
  uchar * cv_ptr, * gl_ptr;

  gl_ptr = gl_img.data;

  for(int i = 0; i < img.rows; ++i){
    cv_ptr = img.ptr<uchar>(i);
    for(int j = 0; j < img.cols; ++j){
      for(int c = 0; c < gl_img.channels(); ++c){
	gl_ptr[i * gl_img.step + j * gl_img.elemSize() + c] = cv_ptr[j];
      }      
    }
  }
  
  img = gl_img;
}


//////////////////////////////////////////////////////// stereo related 
void calc_obst(s_odt_par & par, cv::Mat & disp, vector<s_obst> & obst)
{
  // 1. labeling for region with same and continuous disparity.
  // 2. threasholding for height of the top edge pixels of the regions, and determines (x,d) tuple of the obstacle 

  par.lbl = cv::Mat::zeros(disp.rows, disp.cols, CV_16UC1);
  par.tmp.clear();
  par.tmp.push_back(-1);
  obst.clear();
  obst.push_back(s_obst());

  ushort * plbl, *plbl_prev;
  ushort * pdisp_prev, *pdisp;
  int nrgn = 0;
  // labeling phase
  for (int y = 1; y < disp.rows; y++){
    plbl_prev = par.lbl.ptr<ushort>(y - 1);
    plbl = par.lbl.ptr<ushort>(y);
    pdisp_prev = disp.ptr<ushort>(y - 1);
    pdisp = disp.ptr<ushort>(y);
    for (int x = 0; x < disp.cols - 1; x++){
      if (*pdisp && *pdisp <= par.dn && *pdisp >= par.df){
	int irgn, drgn;
	{
	  // determine the label of this pixel
	  // XXX
	  // XPY <- P's label is defined by X's label and disparity.
	  // YYY

	  // The difference of the disparity between X and P should be
	  // less than m_rgn_drange. If there are many labels satisfy 
	  // the condition, a label with minimum disparity is selected.

	  int d, dmin = INT_MAX;
	  int irgn_min = -1;

	  irgn = *(plbl_prev - 1);
	  if (irgn){
	    if ((d = obst[irgn].diff(*pdisp)) < dmin){
	      dmin = d;
	      irgn_min = irgn;
	    }
	  }

	  irgn = *(plbl_prev);
	  if (irgn){
	    if ((d = obst[irgn].diff(*pdisp)) < dmin){
	      dmin = d;
	      irgn_min = irgn;
	    }
	  }

	  irgn = *(plbl_prev + 1);
	  if (irgn){
	    if ((d = obst[irgn].diff(*pdisp)) < dmin){
	      dmin = d;
	      irgn_min = irgn;
	    }
	  }

	  irgn = *(plbl - 1);
	  if (irgn){
	    if ((d = obst[irgn].diff(*pdisp)) < dmin){
	      dmin = d;
	      irgn_min = irgn;
	    }
	  }

	  irgn = irgn_min;
	  drgn = dmin;
	}

	if (irgn == -1 || drgn > par.drange){
	  irgn = (int)obst.size();
	  s_obst o;
	  o.dmax = o.dmin = *pdisp;
	  o.xmax = o.xmin = x;
	  o.ymax = o.ymin = y;
	  o.pix = 1;
	  obst.push_back(o);
	  par.tmp.push_back(irgn);
	}
	else{
	  obst[irgn].add(x, y, *pdisp);
	}
	*plbl = irgn;
      }
      else{
	*pdisp = 0;
	obst[0].add(x, y, 0);
      }

      plbl_prev++;
      plbl++;
      pdisp_prev++;
      pdisp++;
    }
  }

  par.nullpix = obst[0].pix;
  int iobst = 0;
  for (int irgn = 1; irgn < obst.size(); irgn++)
    {
      s_obst & o = obst[irgn];
      /*
	bool merged = false;
	for (int iobst2 = 0; iobst2 < iobst; iobst2++){
	s_obst & o2 = obst[iobst2];
	if (o.dmax < o2.dmin || o.dmin > o2.dmax ||
	o.xmax < o2.xmin || o.xmin > o2.xmax ||
	o.ymax < o2.ymin || o.ymin > o2.ymax){
	continue;
	}

	ushort dmin = min(o.dmin, o2.dmin);
	ushort dmax = max(o.dmax, o2.dmax);
	if ((dmax - dmin) >> 1 > par.drange)
	continue;

	o2.dmax = dmax;
	o2.dmin = dmin;
	o2.xmax = max(o.xmax, o2.xmax);
	o2.xmin = min(o.xmin, o2.xmin);
	o2.ymax = max(o.ymax, o2.ymax);
	o2.ymin = min(o.ymin, o2.ymin);
	merged = true;
	break;
	}
		
	if (merged){
	continue;
	}
      */
      if (par.is_valid(o)){
	obst[iobst] = obst[irgn];
	iobst++;
      }
    }
  obst.resize(iobst);

}

//////////////////////////////////////////////////////////////////////////////////////// mat img read/write
bool write_raw_img(const cv::Mat & img, const char * fname)
{
  FILE * pf = fopen(fname, "wb");
  if(pf){
    int r, c, type, size;
    r = img.rows;
    c = img.cols;
    type = img.type();
    size = (int)(r * c * img.channels() * img.elemSize());
    fwrite((void*)&type, sizeof(int), 1, pf);
    fwrite((void*)&r, sizeof(int), 1, pf);
    fwrite((void*)&c, sizeof(int), 1, pf);
    fwrite((void*)&size, sizeof(int), 1, pf);
    fwrite((void*)img.data, sizeof(char), size, pf);
  }else{
    cerr << "Failed to write " << fname << "." << endl;
    return false;
  }
  fclose(pf);
  return true;
}

bool read_raw_img(cv::Mat & img, const char * fname)
{
  FILE * pf = fopen(fname, "rb");
  if(pf){
    int r, c, type, size;
    size_t res;
    r = c = type = size = 0;
    res = fread((void*)&type, sizeof(int), 1, pf);
    if(!res)
      goto failed;
    res = fread((void*)&r, sizeof(int), 1, pf);
    if(!res)
      goto failed;
    res = fread((void*)&c, sizeof(int), 1, pf);
    if(!res)
      goto failed;
    res =fread((void*)&size, sizeof(int), 1, pf);
    if(!res)
      goto failed;
    img.create(r, c, type);
    res = fread((void*)img.data, sizeof(char), size, pf);
    if(!res)
      goto failed;
  }else{
    cerr << "Failed to read " << fname << "." << endl;
    return false;
  }

  return true;
 failed:
  cerr << "Failedto read " << fname << "." << endl;
  return false;
}
