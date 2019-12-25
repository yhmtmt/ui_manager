// Copyright(c) 2015-2019 Yohei Matsumoto, All right reserved. 

// f_glfw_window.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_glfw_window.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_glfw_window.  If not, see <http://www.gnu.org/licenses/>. 
#include "f_glfw_window.hpp"

//////////////////////////////////////////////////////////////////////////////////////////// f_glfw_window
bool f_glfw_window::is_init_glut = false;
MapGLFWin f_glfw_window::m_map_glfwin;

f_glfw_window::f_glfw_window(const char * name) :f_base(name), m_sz_win(640, 480), m_depth_bits(16), m_bfull(false), m_display(0)
{
  m_pwin = NULL;
  register_fpar("width", &m_sz_win.width, "Width of the window.");
  register_fpar("height", &m_sz_win.height, "Height of the window.");
  register_fpar("depth_bits", &m_depth_bits, "Bit depth of depth buffer.");
  register_fpar("full", &m_bfull, "Full screen mode is enabled.");
  register_fpar("display", &m_display, "Display number used for full screen mode");
}

f_glfw_window::~f_glfw_window()
{
}

bool f_glfw_window::init_run()
{
  if(!glfwInit()){
    cerr << "Failed to initialize GLFW." << endl;
    return false;
  }

  glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

  glfwWindowHint(GLFW_DEPTH_BITS, m_depth_bits);

  GLFWmonitor * pmonitor = NULL;
  if (m_bfull)
    {
      int count = 0;
      GLFWmonitor ** pmonitors = glfwGetMonitors(&count);
      if (m_display < count)
	pmonitor = pmonitors[m_display];
      else
	pmonitor = pmonitors[0];
	  
      int min_diff = INT_MAX;
      int ivm_best = 0;
      /*
	const GLFWvidmode * pvidmods = glfwGetVideoModes(pmonitor, &count);
	for (int ivm = 0; ivm < count; ivm++){
	int diff = abs(pvidmods[ivm].width - m_sz_win.width) + abs(pvidmods[ivm].height - m_sz_win.height);
	if (diff < min_diff){
	min_diff = diff;
	ivm_best = ivm;
	}
	}
	GLFWvidmode vidmod;  
	vidmod = pvidmods[ivm_best];

	m_sz_win.width = vidmod.width;
	m_sz_win.height = vidmod.height;
	glfwWindowHint(GLFW_RED_BITS, vidmod.redBits);
	glfwWindowHint(GLFW_GREEN_BITS, vidmod.greenBits);
	glfwWindowHint(GLFW_BLUE_BITS, vidmod.blueBits);
	glfwWindowHint(GLFW_REFRESH_RATE, vidmod.refreshRate);

      */
      const GLFWvidmode * pvidmod = glfwGetVideoMode(pmonitor);

      m_sz_win.width = pvidmod->width;
      m_sz_win.height = pvidmod->height;
      glfwWindowHint(GLFW_RED_BITS, pvidmod->redBits);
      glfwWindowHint(GLFW_GREEN_BITS, pvidmod->greenBits);
      glfwWindowHint(GLFW_BLUE_BITS, pvidmod->blueBits);
      glfwWindowHint(GLFW_REFRESH_RATE, pvidmod->refreshRate);
    }
  
  m_pwin = glfwCreateWindow(m_sz_win.width, m_sz_win.height, m_name, pmonitor, NULL);
  if (!pwin())
    {
      cerr << "Failed to create GLFW window." << endl;
      cerr << "Window name = " << m_name << " " << m_sz_win.width << " x " << m_sz_win.height << endl;
      glfwTerminate();
      return false;
    }
  
  m_map_glfwin.insert(pair<GLFWwindow*, f_glfw_window*>(m_pwin, this));
  
  glfwMakeContextCurrent(pwin());
  
  glfwSetKeyCallback(pwin(), key_callback);
  glfwSetCursorPosCallback(pwin(), cursor_position_callback);
  glfwSetMouseButtonCallback(pwin(), mouse_button_callback);
  glfwSetScrollCallback(pwin(), scroll_callback);
  glfwSetErrorCallback(err_cb);
  
  glfwSetWindowTitle(pwin(), m_name);
  
  GLenum err;
  if((err = glewInit()) != GLEW_OK){
    cerr << "Failed to initialize GLEW." << endl;
    cerr << "\tMessage: " << glewGetString(err) << endl;
    return false;
  }

  if(!is_init_glut){
    int argc = 0;
    glutInit(&argc, NULL);
    is_init_glut = true;
  }
  
  return true;	
}

void f_glfw_window::destroy_run()
{ 
  if(!m_pwin)
    return;
  MapGLFWin::iterator itr = m_map_glfwin.find(m_pwin);
  m_map_glfwin.erase(itr);
  
  glfwDestroyWindow(m_pwin);
  glfwTerminate();

  m_pwin = NULL;
}

bool f_glfw_window::proc()
{
  if(glfwWindowShouldClose(pwin()))
    return false;

  //glfwMakeContextCurrent(pwin());
	
  // rendering codes >>>>>

  // <<<<< rendering codes

  glfwSwapBuffers(pwin());

  glfwPollEvents();

  return true;
}

