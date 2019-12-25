#ifndef F_GLFW_WINDOW_HPP
#define F_GLFW_WINDOW_HPP
// Copyright(c) 2015-2019 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_glfw_window.hpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_glfw_window.hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_glfw_window.hpp  If not, see <http://www.gnu.org/licenses/>. 

#include "filter_base.hpp"
#include "util/aws_glib.hpp"

inline void crossProduct(const float x1, const float y1, const float z1, 
			 const float x2, const float y2, const float z2, 
			 float & nx, float & ny, float & nz)
{
  nx = (float)(y1 * z2 - y2 * z1);
  ny = (float)(z1 * x2 - z2 * x1);
  nz = (float)(x1 * y2 - x2 * y1);
}

inline void normalize(float & nx, float & ny, float & nz)
{
  float inorm = (float) (1.0 / sqrt(nx * nx + ny * ny + nz * nz));
  nx *= inorm;
  ny *= inorm;
  nz *= inorm;
}

inline void setEyeGl4x4(GLfloat * m)
{
  memset((void*)m, 0, sizeof(GLfloat) * 16);
  m[0] = m[5] = m[10] = m[15] = 1.0;
}

inline void reformRtAsGl4x4(cv::Mat & R, cv::Mat & t, GLfloat * m)
{
  // note that m is the OpenGL memory layout (say column major order)
  double * pR = R.ptr<double>();
  double * pt = t.ptr<double>();
  int i, j;

  for(i = 0; i < 3; i++){
    for(j = 0; j < 3; j++){
      m[j * 4 + i] = (float) pR[i * 3 + j];
    }
    m[j * 4 + i] = 0;
  }

  for(int i = 0; i < 3; i++){
    m[12 + i] = (float) pt[i];
  }

  m[15] = 1.0;
}

inline void set_identity4x4(float * m) 
{
  for (int i = 0; i < 16; i++){
    m[i] = 0.f;
  }

  m[0] = m[5] = m[10] = m[15] = 1.0f;
}

inline void mult4x4(float * a, float * b) // a = axb
{
  float m[16];
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      m[j * 4 + i] = 0.0f;
      for (int k = 0; k < 4; ++k) {
	m[j * 4 + i] += (float)(a[k * 4 + i] * b[j * 4 + k]);
      }
    }
  }
  memcpy(a, m, 16 * sizeof(float));
}

inline void set_tran(float * m, float x, float y, float z)
{
  set_identity4x4(m);
  m[12] = x;
  m[13] = y;
  m[14] = z;
}

inline void set_pm(float * m, float fov, float ratio, float nearP, float farP)
{
  float f = (float)(1.0f / tan(fov * (PI / 360.0)));

  set_identity4x4(m);

  m[0] =(float)(f / ratio);
  m[1 * 4 + 1] = f;
  m[2 * 4 + 2] = (float)((farP + nearP) / (nearP - farP));
  m[3 * 4 + 2] = (float)((2.0f * farP * nearP) / (nearP - farP));
  m[2 * 4 + 3] = (float)(-1.0f);
  m[3 * 4 + 3] = 0.0f;
}

inline void xproduct(float * a, float * b, float * res)
{
  res[0] = (float)(a[1] * b[2] - b[1] * a[2]);
  res[1] = (float)(a[2] * b[0] - b[2] * a[0]);
  res[2] = (float)(a[0] * b[1] - b[0] * a[1]);
}

inline void norm(float * a)
{
  float imag = (float)(1.0 / sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]));

  a[0] *= imag;
  a[1] *= imag;
  a[2] *= imag;
}

inline void set_vm(float * m, float posX, float posY, float posZ,
		   float lookAtX, float lookAtY, float lookAtZ) 
{
  float dir[3], right[3], up[3];

  up[0] = 0.0f;   up[1] = 1.0f;   up[2] = 0.0f;

  dir[0] = (lookAtX - posX);
  dir[1] = (lookAtY - posY);
  dir[2] = (lookAtZ - posZ);
  norm(dir);

  xproduct(dir, up, right);
  norm(right);

  xproduct(right, dir, up);
  norm(up);

  float aux[16];

  m[0] = right[0];
  m[4] = right[1];
  m[8] = right[2];
  m[12] = 0.0f;

  m[1] = up[0];
  m[5] = up[1];
  m[9] = up[2];
  m[13] = 0.0f;

  m[2] = -dir[0];
  m[6] = -dir[1];
  m[10] = -dir[2];
  m[14] = 0.0f;

  m[3] = 0.0f;
  m[7] = 0.0f;
  m[11] = 0.0f;
  m[15] = 1.0f;

  set_tran(aux, -posX, -posY, -posZ);

  mult4x4(m, aux);
}

void printShaderInfoLog(GLuint obj);
void printProgramInfoLog(GLuint obj);

struct cmpglfwin{
  bool operator () (const GLFWwindow * a, const GLFWwindow * b) const{
    return reinterpret_cast<unsigned long long>(a) 
      < reinterpret_cast<unsigned long long>(b);
  }
};

class f_glfw_window;

typedef map<GLFWwindow *, f_glfw_window *, cmpglfwin> MapGLFWin;

class f_glfw_window: public f_base
{
protected:
  static bool is_init_glut;
  
  static MapGLFWin m_map_glfwin;
  GLFWwindow * m_pwin;
  int m_depth_bits;
  bool m_bfull;
  int m_display;

  GLFWwindow * pwin(){
    return m_pwin;
  }
  
  cv::Size m_sz_win;
  
  virtual bool init_run();
  virtual void destroy_run();
  
  virtual void _key_callback(int key, int scancode, int action, int mods)
  {
  }
  
  static void key_callback(GLFWwindow * pwindow, int key, int scancode, int action, int mods)
  {
    MapGLFWin::iterator itr = m_map_glfwin.find(pwindow);
    f_glfw_window * ptr = itr->second;
    ptr->_key_callback(key, scancode, action, mods);
  }
  
  virtual void _cursor_position_callback(double xpos, double ypos)
  {
  }
  
  static void cursor_position_callback(GLFWwindow* pwindow, double xpos, double ypos)
  {
    MapGLFWin::iterator itr = m_map_glfwin.find(pwindow);
    f_glfw_window * ptr = itr->second;
    ptr->_cursor_position_callback(xpos, ypos);
  }
  
  virtual void _mouse_button_callback(int button, int action, int mods)
  {
  }
  
  static void mouse_button_callback(GLFWwindow* pwindow, int button, int action, int mods)
  {
    MapGLFWin::iterator itr = m_map_glfwin.find(pwindow);
    f_glfw_window * ptr = itr->second;
    ptr->_mouse_button_callback(button, action, mods);
  }
  
  void _scroll_callback(double xoffset, double yoffset)
  {
  }
  
  static void scroll_callback(GLFWwindow* pwindow, double xoffset, double yoffset)
  {
    MapGLFWin::iterator itr = m_map_glfwin.find(pwindow);
    f_glfw_window * ptr = itr->second;
    ptr->_scroll_callback(xoffset, yoffset);
  }
  
  static void framebuffer_size_callback(GLFWwindow * pwindow, int width, int height)
  {
    MapGLFWin::iterator itr = m_map_glfwin.find(pwindow);
    f_glfw_window * ptr = itr->second;
    ptr->m_sz_win = cv::Size(width, height);
  }
public:
  f_glfw_window(const char * name);
  virtual ~f_glfw_window();
  
  virtual bool is_main_thread()
  {
    return true;
  }
  
  virtual bool proc();
  
  // glfw callbacks
  static void err_cb(int e, const char * dsc)
  {
    fputs(dsc, stderr);
  }
};

#endif
