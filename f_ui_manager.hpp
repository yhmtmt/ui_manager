// Copyright(c) 2016-2020 Yohei Matsumoto, All right reserved. 

// f_ui_manager.hpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ui_manager.hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ui_manager.hpp.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef F_UI_MANAGER_HPP
#define F_UI_MANAGER_HPP

#include "filter_base.hpp"
#include "aws_map.hpp"

#include "f_glfw_window.hpp"

#include "ch_state.hpp"
#include "ch_aws1_ctrl.hpp"
#include "ch_aws1_sys.hpp"
#include "ch_map.hpp"
#include "ch_wp.hpp"
#include "ch_obj.hpp"
#include "ch_radar.hpp"

#define MAX_RT_FILES 10

#include "util/c_ui_box.hpp"
#include "util/c_map_obj.hpp"

class f_ui_manager: public f_glfw_window
{
public:
  enum e_obj_type{
    ot_wp, ot_cl, ot_ais, ot_mark, ot_nul
  };
  
  struct s_obj{
    e_obj_type type;
    int handle;
    s_obj():type(ot_nul), handle(-1){}
  };
  
  enum e_mouse_state{
    ms_add_wp, ms_drag, ms_normal
  } mouse_state;
  
private:
  bool m_verb; // verbose mode flag (for debug)
  
  //////////////////////////////////////////////////////// Channel Declaration  
  ch_state * m_state;			// required
  ch_eng_state * m_engstate;            // optional
  ch_aws1_sys * m_ch_sys;		// is not used
  ch_aws1_ctrl_inst * m_ch_ctrl_inst;	// optional
  ch_aws1_ctrl_stat * m_ch_ctrl_stat;   // optional
  ch_wp * m_ch_wp;	 	        // optional,
                                        //ref. update_route_cfg_box(),
                                        //     update_route(), add_waypoint()
  ch_map * m_ch_map;			// optional, ref. update_map()
  ch_ais_obj * m_ch_ais_obj;		// optional, ref. update_ais_obj()
  ch_aws1_ap_inst * m_ch_ap_inst;	// optional,
                                        // ref. update_ctrl_mode_box(),
                                        //      handle_ctrl_csr()
  ch_radar_image * m_ch_radar_image;    // radar image channel
  ch_radar_ctrl * m_ch_radar_ctrl;      // radar control channel
  ch_radar_state * m_ch_radar_state;    // radar state channel
  
  ////////////////////////////////////////////////////// shader related members
  
  char fvs[1024];    // File path to the vertex shader program 
  char ffs[1024];    // File path to the fragment shader program
  char ftex[1024];   // File path to the texture image of the system font
  char ftexinf[1024];// File path to the texture information of the system font
  
  GLuint p;          // shader program

  // location variables accessible in the shader programs
  GLuint loc_mode             /* rendering mode flag. See shader program.*/;
  GLuint loc_inv_sz_half_scrn /* inverse half screen size */;
  GLuint loc_gcolor           /* object color */;
  GLuint loc_gcolorb          /* object background color */;
  GLuint loc_pos2d            /* 2d point */;
  GLuint loc_t2d              /* 2d translation */;
  GLuint loc_scale2d          /* 2d scaling */;
  GLuint loc_Mmvp             /* transform matrix */;
  GLuint loc_Mm               /* object rotation/translation matrix */;
  GLuint loc_Lpar             /* Light direction */;
  GLuint loc_sampler          /* Texture sampler */;
  GLuint loc_depth2d;         /* Depth value used in 2d rendering */
  GLuint loc_position;        /* 3d vertex */
  GLuint loc_normal;          /* 3d normal vector */
  GLuint loc_texcoord;        /* texture coordinate */
  
  float inv_sz_half_scrn[2];  // Inverse of window width and height
                              // passed to shader program
  float iRE, height_cam_ec, dhorizon_cam, dhorizon_arc, zhorizon, th_horizon;
  bool setup_shader();
  
  /////////////////////////////////////////// graphics elements. 
  // The elements can be used by changing position, scale, and rotation
  // for each graphics artifacts.
  c_gl_2d_obj orect          /* 2d rectangle */;
  c_gl_2d_obj otri           /* 2d triangle */;
  c_gl_2d_obj ocirc          /* 2d circle */;
  c_gl_2d_obj omap_mask      /* map mode mask */;  
  c_gl_text_obj otxt         /* Text */;
  c_gl_2d_line_obj oline     /* 2d line */;
  c_gl_line_obj oline3d      /* 3d line */;
  c_gl_line_obj oline3d_map; /* 3d lin for map*/
  c_gl_radar oradar;         /* radar image renderer */
  
  cv::Mat m_cam;               // main camera image (now empty)
  long long m_tcam, m_frm_cam; // time and frame number m_cam grabbed
  e_imfmt m_fmt_cam;           // image format of main camera
  
  // Convert given OpenCV Mat image data to OpenGL bitmap image data layout.
  void cnv_img_to_view(cv::Mat & img, float av, cv::Size & sz,
		       bool flipx, bool flipy);

  int hmap_mask[2];
  bool init_map_mask();
  
  void render_gl_objs(c_view_mode_box * pvm_box); // renders all elements above declared.

  // ui boxes
  c_aws_ui_box_manager uim; // manages four ui-boxes.
  
  // check update in 4 ui boxes and call proper update_*_box function below.
  void handle_updated_ui_box(c_view_mode_box * pvm_box,
			     c_ctrl_mode_box * pcm_box,
			     c_map_cfg_box * pmc_box,
			     c_route_cfg_box * prc_box);
 
  void update_view_mode_box(c_view_mode_box * pvm_box);
  void update_ctrl_mode_box(c_ctrl_mode_box * pcm_box);
  void update_map_cfg_box(c_map_cfg_box * pmc_box);
  void update_route_cfg_box(c_route_cfg_box * prc_box,
			    e_mouse_state mouse_state_new);
  
  // updating own_ship appearance for map mode, and 3d projection matrix
  // for fpv mode.
  void update_ui_params(c_view_mode_box * pvm_box,
			const double xown, const double yown, const double zown,
			const float vx, const float vy, const float yaw);

  c_indicator ind; // indicates boat state
  // Update boat state indicator
  void update_indicator(const float cog, const float sog, 
			const float roll, const float pitch, const float yaw,
			const float rpm, const float trim, const int poil,
			const float toil, const float temp, const float valt,
			const float frate, const unsigned int teng,
			const int pclnt, const int pfl, const unsigned char ld,
			const unsigned char tq, const StatEng1 steng1,
			const StatEng2 steng2, const float depth);
  
  //////////////////////////////////////////// map objects
  float sz_mark;		   // Basic size of each object.
  vector<bool> visible_obj;        // Visible objects (copied from map_cfg_box),
                                   // the index is e_obj_type

  c_map_waypoint_obj owp;          // waypoint object
  int num_max_wps;		   // maximum number of waypoints

  c_map_coast_line_obj coast_line; // coast line object
  bool bupdate_map;		   // flag to notify reload the map 
  glm::vec3 pt_prev_map_update;	   // position caused previous map update.
                                   //(if the distance from the point has been
                                   // larger than map range, bupdate_map is
                                   // asserted.) 
  void update_map();
  
  c_map_ais_obj oais;		   // ais object
  int num_max_ais;	           // maximum number of ais objects ui can draw.

  void update_ais_objs();
  void update_route(c_route_cfg_box * prc_box);

  c_own_ship own_ship;             // own ship object
  c_cursor ocsr;		   // cursor 
 
  enum e_button{
    ebtn_lock_map_own_ship, ebtn_lock_cam_dir_hdg, ebtn_js_ctrl, ebtn_nul
  };
  e_button btn_pushed, btn_released;
  c_aws_ui_button btn_lock_map_own_ship, btn_lock_cam_dir_hdg, btn_js_ctrl;
  e_button get_col_button();
  bool handle_btn_pushed();
  bool handle_btn_released();
  void update_button(c_view_mode_box * pvm_box);

  ////////////////////////////////////////// Control instruction and status
  // Control source is categorized into two: one is manual, another is autopilot.
  // Manual control is done via joystic by instructing the control value to the m_ch_ctrl_inst channel,
  // and autopilot control is done by instructing the autopilot command to the m_ch_ap_inst channel.

  s_aws1_ctrl_inst m_inst; // the control instruction value
  void snd_ctrl_inst(); // send m_inst to m_ch_ctrl_inst

  s_aws1_ctrl_stat m_stat; // the control state value 
  void rcv_ctrl_stat(); // recieve m_stat from m_ch_ctrl_stat.

  // AWS1's manual control mode, crz: Cruise mode (for usual crusing), ctl: Control mode (for precise control), csr: Cursor mode (AWS1 follows mouse cursor)
  enum e_ctrl_mode{
    cm_crz, cm_ctl, cm_csr, cm_ap, cm_stb, cm_undef
  } ctrl_mode;

  static const char * str_ctrl_mode[cm_undef];
  
  enum e_crz_cmd{
    crz_stp,
    crz_ds_ah, crz_sl_ah, crz_hf_ah, crz_fl_ah, crz_nf,
    crz_ds_as, crz_sl_as, crz_hf_as, crz_fl_as, 
    crz_mds,
    crz_p10,crz_p20,crz_hap,
    crz_s10,crz_s20, crz_has,
    crz_undef
  } crz_cm;
  static const char * str_crz_cmd[crz_undef];
  static const char * str_crz_cmd_exp[crz_undef];
  unsigned char crz_cmd_val[crz_undef];
  
  enum e_stb_cmd{
    stb_stp,
    stb_ds_ah, stb_sl_ah, stb_hf_ah, stb_fl_ah, stb_nf,
    stb_ds_as, stb_sl_as, stb_hf_as, stb_fl_as, 
    stb_undef
  } stb_cm; // if not used, stb_undef is used 
  static const char * str_stb_cmd[stb_undef];
  static const char * str_stb_cmd_exp[stb_undef];
  short stb_cmd_val[stb_undef]; 
  float stb_cog_tgt; // if not set, FLT_MAX is used
  
  float sog_max, rev_max;
  float cog_tgt, sog_tgt, rev_tgt;
  
  float m_rud_f, m_eng_f;
  void handle_ctrl_crz(); // cruise mode: sticks are used to increase/decrease engine throttle and rudder angle 
  void handle_ctrl_ctl(); // control mode: positions of sticks are the throttle values. 
  void handle_ctrl_csr(); // cursor mode: follows cursor position 

  void handle_ctrl_stb(); // stabilized mode: instruct cog and rpm

  void ctrl_cog_tgt();
  void ctrl_sog_tgt();
  void ctrl_rev_tgt();
  ///////////////////////////////////////////////// joypad handlers (used for manual control)
  s_jc_u3613m m_js; // joystick wrapper. (Now i only support jc_u3613) 
  bool bjs;	    // joystick control enable flag
                    // (switched from fset, touch panel, and js's start button)
  int m_js_id;	    // joystick id (glfw's ordering)
  const char * m_js_name; // joystick name (glfw's naming)

  // mouse related members
  glm::dvec2 pt_mouse, pt_mouse_drag_begin, pt_mouse_blh;
  glm::dvec3 pt_mouse_ecef, pt_mouse_enu;
  int mouse_button, mouse_action, mouse_mods;
  s_obj obj_mouse_on;

  // calc_mouse_enu_and_ecef_pos calculates global position the mouse
  // pointer is pointing on. 
  void calc_mouse_enu_and_ecef_pos(e_ui_mode vm, double * Rown,
				   const float lat, const float lon, 
				   const float xown, const float yown,
				   const float zown, const float yaw);

  // these event handlers are called when the mouse event isnot handled in
  // ui boxes. 
  void handle_base_mouse_event(c_view_mode_box * pvm_box,
			       c_ctrl_mode_box * pcm_box,
			       c_map_cfg_box * pmc_box,
			       c_route_cfg_box * prc_box);
  
  void handle_mouse_lbtn_push(c_view_mode_box * pvm_box,
			      c_ctrl_mode_box * pcm_box,
			      c_map_cfg_box * pmc_box,
			      c_route_cfg_box * prc_box);
  void handle_mouse_lbtn_release(c_view_mode_box * pvm_box,
				 c_ctrl_mode_box * pcm_box,
				 c_map_cfg_box * pmc_box,
				 c_route_cfg_box * prc_box);
  void handle_mouse_mv(c_view_mode_box * pvm_box,
		       c_ctrl_mode_box * pcm_box,
		       c_map_cfg_box * pmc_box,
		       c_route_cfg_box * prc_box);
  void handle_mouse_drag(c_view_mode_box * pvm_box, s_obj & obj_tmp);
  void clear_mouse_event()
  {
    mouse_button = -1;
    mouse_action = -1;
    mouse_mods = -1;
  }

  void clear_mouse_state(c_route_cfg_box * prc_box)
  {
    if (mouse_state == ms_add_wp) {
      prc_box->command_processed(c_route_cfg_box::wp_add);
    }
    mouse_state = ms_normal;
  }
  
  void add_waypoint(c_route_cfg_box * prc_box);
  void drag_waypoint();
  void drag_cam_dir();
  void drag_map();
  void det_obj_collision();

  // map related members
  bool bmap_center_free;       // Asserted if the own_ship is not on the
                               // map center
  double Rmap[9];              // ecef to enu coordinate rotation matrix
  glm::vec2 pt_map_center_blh; // Current map center in BLH coordinate
  glm::vec3 pt_map_center_ecef;// Current map center in ECEF coordinate

  unsigned int map_range_base; // stride of the map range update.
  unsigned int map_range;      // map radius visible in ui 
  float meter_per_pix;         // length of viewport pixel in meter
  float pix_per_meter;         // inverse of meter_per_pix (1/meter_per_pix)

  // calculate largest radius of a circle placeable in viewport. 
  unsigned int get_max_circle_radius()
  {
    if(m_sz_win.height < m_sz_win.width){
      return m_sz_win.height >> 1;
    }
    return m_sz_win.width >> 1;
  }
  
  // Called when map range is changed, meter_per_pix and pix_per_meter is
  // recalculated.
  void recalc_range()
  {
    bupdate_map = true;
    meter_per_pix = (float)((float)map_range
			    / (float)(get_max_circle_radius()));
    pix_per_meter = (float)(1.0 / meter_per_pix);
  }

  ////////////////////////////////////////////////////// FPV mode related member
  float fov_cam_x, fov_cam_y; // Field of view (if given, calucated from fcam) 
  float fcam, ifcam;          // Focal length and the inverse of the camera
                              // assumed in FPV mode.
  float height_cam;           // camera height 
  float dir_cam_hdg;          // on-board camera direction relative to bow line
  float dir_cam_hdg_drag;     // only used in dragging on-board cam direction 

  glm::mat4 pm;   // projection matrix calculated with fov_cam_y
  glm::mat4 vm;   // Viewing matrix calculated with camera height
  glm::mat4 mm;   // rotation and translation matrix
  glm::mat4 pvm;  // multiplication, pm * pv * mm, is transfered to shader
  glm::vec3 light;// light direction (parallel light assumed)

  //////////////////////////////////////// video/screen capture related members
  cv::VideoWriter m_vw;        // video writer 
  bool m_bsvw;                 // screen video write
  bool m_bss;                  // screen shot
  cv::Mat m_simg;	       // screen img
  char m_path_storage[1024];   // path to the storage for screen shot
  void print_screen();
  
  // glfw event handler 
  virtual void _cursor_position_callback(double xpos, double ypos);
  virtual void _mouse_button_callback(int button, int action, int mods);
  virtual void _key_callback(int key, int scancode, int action, int mods);

  // helper for init_run()
  
public:
  f_ui_manager(const char * name);
  virtual ~f_ui_manager();

  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();

  // If LT+LB+RT+RB is detected, the system forces the controls to be nutral state. Called by default.
  void ui_force_ctrl_stop(c_ctrl_mode_box * pcm_box);
  void js_force_ctrl_stop(c_ctrl_mode_box * pcm_box);
  bool m_quit;
};
#endif
