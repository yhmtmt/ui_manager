// Copyright(c) 2016-2020 Yohei Matsumoto, All right reserved. 

// f_ui_manager.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ui_manager.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ui_manager.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_ui_manager.hpp"

DEFINE_FILTER(f_ui_manager);

f_ui_manager::f_ui_manager(const char * name) :
  f_glfw_window(name),
  m_state(NULL), m_engstate(NULL), m_ch_sys(NULL),
  m_ch_ctrl_out(nullptr), m_ch_ctrl_in(nullptr),
  m_ch_wp(NULL), m_ch_map(NULL), m_ch_ais_obj(NULL), 
  m_ch_radar_image(NULL), m_ch_radar_ctrl(NULL), m_ch_radar_state(NULL),
  m_js_id(0), bjs(false), m_bsvw(false), m_bss(false),
  fov_cam_x(100.0f), fcam(0), height_cam(2.0f), dir_cam_hdg(0.f),
  dir_cam_hdg_drag(0.f), num_max_wps(100), num_max_ais(100),
  bupdate_map(true), pt_prev_map_update(0, 0, 0),
  map_range(4000), map_range_base(1000), sz_mark(10.0f), mouse_state(ms_normal),
  bmap_center_free(false), btn_pushed(ebtn_nul), btn_released(ebtn_nul),
  stb_cog_tgt(FLT_MAX),
  m_rud_f(127.), m_eng_f(127.),
  engine_max(255), engine_min(0), engine_nutral(127),
  engine_backward(0), engine_forward(0),
  rudder_max(255), rudder_mid(127), rudder_min(0), 
  cog_tgt(0.f), sog_tgt(3.0f), rev_tgt(700),  sog_max(23),  rev_max(5600),
  msg_builder(1024), log_ctrl_flag(true), replay(false), ctrl_builder(1024)
{
  m_path_storage[0] = '.';m_path_storage[1] = '\0';
 
  register_fpar("ch_state", (ch_base**)&m_state,
		typeid(ch_state).name(), "State channel");
  register_fpar("ch_sys", (ch_base**)&m_ch_sys,
		typeid(ch_aws1_sys).name(), "System property channel");
  register_fpar("ch_engstate", (ch_base**)&m_engstate,
		typeid(ch_eng_state).name(), "Engine Status channel");
  
  register_fpar("ch_ctrl_out", (ch_base**)&m_ch_ctrl_out,
		typeid(ch_ctrl_data).name(), "Control data out (to autopilot)");
  register_fpar("ch_ctrl_in", (ch_base**)&m_ch_ctrl_in,
		typeid(ch_ctrl_data).name(), "Control data in (from autopilot)");
  register_fpar("ch_wp", (ch_base**)&m_ch_wp,
		typeid(ch_wp).name(), "Waypoint channel");
  register_fpar("ch_map", (ch_base**)&m_ch_map,
		typeid(ch_map).name(), "Map channel");
  register_fpar("ch_ais_obj", (ch_base**)&m_ch_ais_obj,
		typeid(ch_ais_obj).name(), "AIS object channel");

  register_fpar("ch_radar_image", (ch_base**)&m_ch_radar_image,
		typeid(ch_radar_image).name(), "Radar image");
  register_fpar("ch_radar_ctrl", (ch_base**)&m_ch_radar_ctrl,
		typeid(ch_radar_ctrl).name(), "Radar Control");
  register_fpar("ch_radar_state", (ch_base**)&m_ch_radar_state,
		typeid(ch_radar_state).name());
  
  fvs[0] = ffs[0] = '\0';
  register_fpar("fvs", fvs, 1024, "File path to the vertex shader program.");
  register_fpar("ffs", ffs, 1024, "File path to the fragment shader program.");
  register_fpar("ffnttex", ftex, 1024, "File path to the text texture.");
  register_fpar("ffnttexinf", ftexinf, 1024, "File path to the text texture information corresponding to ffnttex.");

  register_fpar("storage", m_path_storage, 1024, "Path to the storage device");

  register_fpar("eng", &m_eng_f, "Main engine instruction value");
  register_fpar("rud", &m_rud_f, "Rudder instruction value");

  register_fpar("verb", &m_verb, "Debug mode.");
  
  register_fpar("js", &m_js_id, "Joystick id");
  register_fpar("bjs", &bjs, "Joystick enable flag.");

  register_fpar("nwps", &num_max_wps, "Maximum number of waypoints.");
  register_fpar("nais", &num_max_ais, "Maximum number of ais objects.");
  register_fpar("sz_mark", &sz_mark, "Radius of the waypoint marker.");

  register_fpar("fov", &fov_cam_x, "Field of view in FPV mode.");
  register_fpar("fcam", &fcam, "Focal length of the camera assumed in FPV mode in pixel(calculated from fov if not given).");

  register_fpar("height_cam", &height_cam, "Camera height in FPV mode");
  register_fpar("dir_cam_hdg", &dir_cam_hdg, "Camera direction relative to the ship heading.");

  register_fpar("free_map_center", &bmap_center_free, "Free map center.");
  register_fpar("map_range", &map_range, "Range of the map (Radius in meter).");

  register_fpar("ss", &m_bss, "Screen shot now.");
  register_fpar("svw", &m_bsvw, "Screen video write.");
  register_fpar("log_ctrl", &log_ctrl_flag, "Control logging flag.");
  register_fpar("replay", &replay, "Replay flag");
  // for aws1
  eng_cmd_val[eng_fl_as] = 33;
  eng_cmd_val[eng_hf_as] = 43;
  eng_cmd_val[eng_sl_as] = 53;
  eng_cmd_val[eng_ds_as] =102; 
  eng_cmd_val[eng_stp] = 127;   // neutral
  eng_cmd_val[eng_ds_ah] = 152; // 700rpm
  eng_cmd_val[eng_sl_ah] = 200; //1000rpm
  eng_cmd_val[eng_hf_ah] = 210; //2500rpm
  eng_cmd_val[eng_fl_ah] = 220; //4500rpm
  eng_cmd_val[eng_nf] = 225;    //5200rpm
  
  rud_cmd_val[rud_hap] = 0;
  rud_cmd_val[rud_p20] = 47;
  rud_cmd_val[rud_p10] = 87;
  rud_cmd_val[rud_mds] = 127;
  rud_cmd_val[rud_s10] = 140;
  rud_cmd_val[rud_s20] = 180;
  rud_cmd_val[rud_has] = 255;

  rev_cmd_val[rev_fl_as] = -2000;
  rev_cmd_val[rev_hf_as] =  -1500;
  rev_cmd_val[rev_sl_as] = -1000;
  rev_cmd_val[rev_ds_as] =-700; 
  rev_cmd_val[rev_stp] = 0; 
  rev_cmd_val[rev_ds_ah] = 700;
  rev_cmd_val[rev_sl_ah] = 1200;
  rev_cmd_val[rev_hf_ah] = 2500;
  rev_cmd_val[rev_fl_ah] = 4000;
  rev_cmd_val[rev_nf] = 5500;  
  
  cog_cmd_val[cog_p20] = -20;
  cog_cmd_val[cog_p10] = -10;
  cog_cmd_val[cog_p5] = -5;
  cog_cmd_val[cog_0] = 0;
  cog_cmd_val[cog_s5] = 5;
  cog_cmd_val[cog_s10] = 10;
  cog_cmd_val[cog_s20] = 20;

  sog_cmd_val[sog_fl_as] = -8;
  sog_cmd_val[sog_hf_as] = -6;
  sog_cmd_val[sog_sl_as] = -4;
  sog_cmd_val[sog_ds_as] = -2; 
  sog_cmd_val[sog_stp] = 0; 
  sog_cmd_val[sog_ds_ah] = 2;
  sog_cmd_val[sog_sl_ah] = 5;
  sog_cmd_val[sog_hf_ah] = 10;
  sog_cmd_val[sog_fl_ah] = 15;
  sog_cmd_val[sog_nf] = 20;  
  
  register_fpar("sog_max", &sog_max, "Maximum allowed SOG in kts");
  register_fpar("rev_max", &rev_max, "Maximum allowed REV in rpm");

  register_fpar("cmd_id", (int*)(&cmd.id), (int)UC_NULL, str_ui_command, "Command id");
  register_fpar("ival0", &cmd.ival0, "Integer command argument 0 (64bit)");
  register_fpar("ival1", &cmd.ival1, "Integer command argument 1 (64bit)");
  register_fpar("ival2", &cmd.ival2, "Integer command argument 2 (64bit)");
  register_fpar("ival3", &cmd.ival3, "Integer command argument 3 (64bit)");
  register_fpar("fval0", &cmd.fval0, "Float command argument 0 (64bit)");
  register_fpar("fval1", &cmd.fval1, "Float command argument 1 (64bit)");
  register_fpar("fval2", &cmd.fval2, "Float command argument 2 (64bit)");
  register_fpar("fval3", &cmd.fval3, "Float command argument 3 (64bit)");
  register_fpar("path_quit_script", path_quit_script, 1024,
		"Path to the quit script.");
  register_fpar("quit_script", quit_script, 1024,
		"File name of the quit script");
}



f_ui_manager::~f_ui_manager()
{
}

bool f_ui_manager::init_map_mask()
{
    
  // initializing mask for map mode    
#define MAP_MASK_CIRCLE_DIV 18
#define NUM_MAP_MASK_PTS (MAP_MASK_CIRCLE_DIV+1+4)
#define NUM_MAP_MASK_TRIS (MAP_MASK_CIRCLE_DIV+3)
#define NUM_MAP_MASK_IDX (NUM_MAP_MASK_TRIS*3)
    
  struct s_pts{
    float x, y;      
  } pts[NUM_MAP_MASK_PTS];
  
  unsigned short idx[NUM_MAP_MASK_IDX];
  double ths = PI / (double)MAP_MASK_CIRCLE_DIV;
  double r = (double) get_max_circle_radius();
  pts[NUM_MAP_MASK_PTS - 4].x = (float)(m_sz_win.width >> 1);
  pts[NUM_MAP_MASK_PTS - 4].y = (float)(m_sz_win.height >> 1);
  pts[NUM_MAP_MASK_PTS - 3].x = (float) (m_sz_win.width >> 1);
  pts[NUM_MAP_MASK_PTS - 3].y = -(float) (m_sz_win.height >> 1);
  pts[NUM_MAP_MASK_PTS - 2].x = 0.f;
  pts[NUM_MAP_MASK_PTS - 2].y = pts[NUM_MAP_MASK_PTS - 4].y;
  pts[NUM_MAP_MASK_PTS - 1].x = 0.f;
  pts[NUM_MAP_MASK_PTS - 1].y = -pts[NUM_MAP_MASK_PTS - 4].y;
  
  
  for(int i = 0; i < MAP_MASK_CIRCLE_DIV + 1; i++){
    double th = ths * (double)i;
    double c = cos(th);
    double s = sin(th);
    pts[i].x = (float)(r * s);
    pts[i].y = (float)(r * c);
  }
  
  for (int i = 0; i < MAP_MASK_CIRCLE_DIV; i++) {
    if (i < MAP_MASK_CIRCLE_DIV / 2)
      idx[i * 3] = NUM_MAP_MASK_PTS - 4;
    else
      idx[i * 3] = NUM_MAP_MASK_PTS - 3;
    idx[i * 3 + 1] = i;
    idx[i * 3 + 2] = i + 1;
  }
  
  idx[(NUM_MAP_MASK_TRIS - 3) * 3] = NUM_MAP_MASK_PTS - 4;
  idx[(NUM_MAP_MASK_TRIS - 3) * 3 + 1] = NUM_MAP_MASK_PTS - 2;
  idx[(NUM_MAP_MASK_TRIS - 3) * 3 + 2] = 0;
  
  idx[(NUM_MAP_MASK_TRIS - 2) * 3] = NUM_MAP_MASK_PTS - 4;
  idx[(NUM_MAP_MASK_TRIS - 2) * 3 + 1] = MAP_MASK_CIRCLE_DIV / 2;
  idx[(NUM_MAP_MASK_TRIS - 2) * 3 + 2] = NUM_MAP_MASK_PTS - 3;
  
  idx[(NUM_MAP_MASK_TRIS - 1) * 3] = NUM_MAP_MASK_PTS - 3;
  idx[(NUM_MAP_MASK_TRIS - 1) * 3 + 1] = MAP_MASK_CIRCLE_DIV;
  idx[(NUM_MAP_MASK_TRIS - 1) * 3 + 2] = NUM_MAP_MASK_PTS - 1;
  
  if(!omap_mask.init(loc_mode, loc_t2d, loc_scale2d, loc_pos2d, loc_gcolor, loc_depth2d,
		     NUM_MAP_MASK_PTS, (float*)pts, NUM_MAP_MASK_IDX, idx, 2)){
    cerr << "Failed to initialize map mask." << endl;
    return false;
  }
  
  glm::vec4 clrb(0.0f, 0.0f, 0.0f, 1.0f);
  glm::vec2 pos(0.f,0.f);
  hmap_mask[0] = omap_mask.add(clrb, pos, 0.f, 1.f);
  hmap_mask[1] = omap_mask.add(clrb, pos, PI, 1.f);
  for(int i = 0; i < 2; i++){
    omap_mask.config_border(hmap_mask[i], false, 1.0f);
    omap_mask.config_depth(hmap_mask[i], 9);
  }
  return true;
}

bool f_ui_manager::init_run()
{
  {
    string file_prefix = get_name();
    file_prefix += "_ctrl";
    if(!log_ctrl.init(f_base::get_data_path(), file_prefix, replay)){
      spdlog::error("[{}] Failed to open log file in {}.", get_name(), f_base::get_data_path());
      return false;
    }
  }
  
  if (!m_state)
    {
      cerr << "In filter " << m_name << ", ";
      cerr << "State channel is not connected." << endl;
      return false;
    }

  m_state->set_gps_ant_pos(Eigen::Vector3d(0, 0, -1.));
  
  
  if (!m_ch_map)
    {
      cerr << "In filter " << m_name << ", ";
      cerr << "Map channel is not connected." << endl;
    }
  
  if (!m_ch_ais_obj)
    {
      cerr << "In filter " << m_name << ", ";
      cerr << "AIS object channel is not connected." << endl;
    }
    
  ctrl_src = ControlSource_UI;
  m_rud_f = 127.5;
  m_eng_f = 127.5;
  
  if(!f_glfw_window::init_run())
    return false;

  if(m_js.init(m_js_id)){
    cout << "Joystick " << m_js.name << " found." << endl;
  }
  
  ///////////////////////////////// Preparing graphics resources///////////////////////////
  if (!setup_shader()){
    return false;
  }
  
  inv_sz_half_scrn[0] = (float)(2. / (float)m_sz_win.width);
  inv_sz_half_scrn[1] = (float)(2. / (float)m_sz_win.height);
  if (fcam == 0.0f){
    fcam = (float)((float)(m_sz_win.width >> 1) / tan(fov_cam_x * (0.5 *  PI / 180.0f)));
    ifcam = (float)(1.0 / fcam);
  }
  else{ // fov is overriden
    ifcam = (float)(1.0 / fcam);
    fov_cam_x = (float)(2.0 * atan((m_sz_win.width >> 1) * ifcam) * 180.0f / PI);
  }
  fov_cam_y = (float)(2.0 * atan((m_sz_win.height >> 1) * ifcam) * 180.0f / PI);
  
  // calculating horizon related parameters 
  iRE = (float)(1.0 / RE);
  height_cam_ec = (float)(RE + height_cam);
  dhorizon_cam = (float)sqrt(-RE * RE + height_cam_ec * height_cam_ec);
  th_horizon = (float) acos(RE / height_cam_ec); // angle of the arc to the horizon
  dhorizon_arc = (float)(RE * th_horizon);
  zhorizon = dhorizon_cam * height_cam_ec * iRE;
  
  recalc_range();

  {
    glm::vec2 lb(0, 0);
    glm::vec2 sz(1, 1);
    if (!orect.init_rectangle(loc_mode, loc_t2d, loc_scale2d, loc_pos2d, loc_gcolor, loc_depth2d, lb, sz, 256))
      return false;
    if (!otri.init_circle(loc_mode, loc_t2d, loc_scale2d, loc_pos2d, loc_gcolor, loc_depth2d, 3, 1, 1, 256))
      return false;
    if (!ocirc.init_circle(loc_mode, loc_t2d, loc_scale2d, loc_pos2d, loc_gcolor, loc_depth2d, 10, 1, 1, 256))
      return false;
    if (!otxt.init(ftex, ftexinf, loc_mode, loc_t2d,  loc_scale2d, loc_pos2d, loc_texcoord, loc_sampler, loc_gcolor, loc_gcolorb, loc_depth2d, 65535))
      return false;
    if (!oline.init(loc_mode, loc_t2d, loc_scale2d, loc_pos2d, loc_gcolor, loc_depth2d, 8192))
      return false;
    
    if (!oline3d.init(loc_mode, loc_position, loc_Mmvp, loc_gcolor, 256))
      return false;

    if (!oline3d_map.init(loc_mode, loc_position, loc_Mmvp, loc_gcolor, 0x0001FFFF))
      return false;

    
    if(!init_map_mask())
      return false;
  }
    
  glm::vec4 clr(0, 1, 0, 1);
  glm::vec4 clrb(0, 0, 0, 0);
  glm::vec4 clrw(1,1,0,1);
  glm::vec4 clr0(0.f,0.f,0.f,0.5f);
  glm::vec2 sz_fnt(20, 20), sz_fnt_small(10, 10);
  glm::vec2 sz_scrn(m_sz_win.width, m_sz_win.height);
  glm::vec2 sz_mark_xy((float)(2.0 * sz_mark), (float)(2.0 * sz_mark));
  glm::vec2 sz_ship2d(sz_fnt.x, (float)(sz_fnt.y * 0.5));

  // initializing ui components
  uim.init(&orect, &otri, &otxt, &oline, 
	   clr, clrb, sz_fnt, fov_cam_x, sz_scrn);

  if (!ocsr.init(&oline, &otxt, clr, sz_fnt, sz_fnt))
    return false;
  
  if (!owp.init(&ocirc, &otxt, &oline, &oline3d, clr,
		sz_fnt_small, sz_mark, num_max_wps))
    return false;

  if (!oais.init(&orect, &otri, &otxt, &oline, clr,
		 sz_fnt_small, sz_mark_xy, num_max_ais))
    return false;
  
  if (!ind.init(&oline, &otxt, &orect, &otri, sz_fnt, clr,
		fov_cam_x, sz_scrn))
    return false; 

  if(!oradar.init(GARMIN_XHD_SPOKES, GARMIN_XHD_MAX_SPOKE_LEN, loc_mode,
		  loc_t2d, loc_scale2d, loc_pos2d, loc_texcoord, loc_sampler,
		  loc_gcolor, loc_gcolorb, loc_depth2d, clrw, clrb))
    return false;
  
  if (!own_ship.init(&otri, &oline, &ocirc, &otxt, &oradar, clr, sz_ship2d))
    return false;

  if (!coast_line.init(&oline3d_map, clr, 4096))
    return false;

  
  glm::vec2 sz((float)(sz_fnt.x * 7), (float)(sz_fnt.y * 2)),
    pos((float)(-sz.x * 0.5), (float)((m_sz_win.height >> 1) - sz.y));

  c_aws_ui_button::set_gl_element(&orect, &otxt);
  if (!btn_lock_cam_dir_hdg.init(pos, sz, 5, clr, clrb)){
    return false;
  }
  btn_lock_cam_dir_hdg.set_invisible();
  btn_lock_cam_dir_hdg.set_text("LOCK");
  
  if (!btn_lock_map_own_ship.init(pos, sz, 5, clr, clrb))
    {
      return false;
    }
  btn_lock_map_own_ship.set_invisible();
  btn_lock_map_own_ship.set_text("LOCK");
   
  pos.x -= sz.x;
  if (!btn_js_ctrl.init(pos, sz, 5, clr, clrb))
    {
      return false;
    }
  btn_js_ctrl.set_visible();
  btn_js_ctrl.set_text("JS");
  
  // Visible object
  visible_obj.resize(ot_nul, true);
  
  // Initializing OpenGL flags
  
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  mouse_button = -1;
  mouse_action = -1;
  mouse_mods = -1;

  cout << ">>>>>>>>>>>> GL Resource Usage <<<<<<<<<<<<<<" << endl;
  cout << "rect:" << orect.get_used_resource_size()
       << "/" << orect.get_reserved_resource_size() << endl;
  cout << "circ:" << ocirc.get_used_resource_size()
       << "/" << ocirc.get_reserved_resource_size() << endl;
  cout << "tri:" << otri.get_used_resource_size()
       << "/" << otri.get_reserved_resource_size() << endl;
  cout << "txt:" << otxt.get_used_resource_size()
       << "/" << otxt.get_reserved_resource_size() << endl;
  cout << "line:" << oline.get_used_resource_size()
       << "/" << oline.get_reserved_resource_size() << endl;
  cout << "line3d:" << oline3d.get_used_resource_size()
       << "/" << oline3d.get_reserved_resource_size() << endl;
  cout << "line3d_map:" << oline3d_map.get_used_resource_size()
       << "/" << oline3d_map.get_reserved_resource_size() << endl;


  msg_builder.Clear();
  
  UIManagerMsg::FinishUIManagerMsgBuffer(msg_builder,
					 UIManagerMsg::CreateUIManagerMsg(msg_builder));
  
  return true;
}

bool f_ui_manager::setup_shader()
{
  load_glsl_program(ffs, fvs, p);
  
  loc_inv_sz_half_scrn = glGetUniformLocation(p, "inv_sz_half_scrn");
  loc_t2d = glGetUniformLocation(p, "t2d");
  loc_Mmvp = glGetUniformLocation(p, "Mmvp");
  loc_Mm = glGetUniformLocation(p, "Mm");
  loc_Lpar = glGetUniformLocation(p, "Lpar");
  loc_sampler = glGetUniformLocation(p, "sampler");
  loc_scale2d = glGetUniformLocation(p, "scale2d");
  loc_depth2d = glGetUniformLocation(p, "depth2d");
  loc_position = glGetAttribLocation(p, "position");
  loc_normal = glGetAttribLocation(p, "normal");
  loc_texcoord = glGetAttribLocation(p, "texcoord");  
  loc_mode = glGetUniformLocation(p, "mode");
  loc_gcolor = glGetUniformLocation(p, "gcolor");
  loc_gcolorb = glGetUniformLocation(p, "gcolorb");
  loc_pos2d = glGetAttribLocation(p, "pos2d");
  
  return true;
}


void f_ui_manager::destroy_run()
{
  orect.destroy();
  otri.destroy();
  ocirc.destroy();
  omap_mask.destroy();
  otxt.destroy();
  oline.destroy();
  oline3d.destroy();
  oline3d_map.destroy();
  oradar.destroy();
  
  f_glfw_window::destroy_run();
}

void f_ui_manager::ui_force_ctrl_stop()
{
  ctrl_src = ControlSource_UI;
  m_eng_f = m_rud_f = 127.f;
}

void f_ui_manager::js_force_ctrl_stop()
{
  if (m_js.id != -1) {
    if (m_js.elb & s_jc_u3613m::EB_STDOWN &&
	m_js.elt & s_jc_u3613m::EB_STDOWN &&
	m_js.erb & s_jc_u3613m::EB_STDOWN &&
	m_js.ert & s_jc_u3613m::EB_STDOWN) {
      ui_force_ctrl_stop();
    }
  }
}


void f_ui_manager::cnv_img_to_view(cv::Mat & img, float av, cv::Size & sz, bool flipx, bool flipy)
{
  if(!img.empty()){
    if(sz.width != img.cols || sz.height != img.rows){
      float ai = (float)((float) img.cols / (float) img.rows);
      if(av > ai){
	sz.width = (int)((float)img.cols * ((float)sz.height / (float)img.rows));
      }else{
	sz.height = (int)((float)img.rows * ((float)sz.width / (float)img.cols));				
      }
      sz.width &= 0xFFFFFFFE;
      sz.height &= 0xFFFFFFFE;
      
      cv::Mat tmp;
      resize(img, tmp, sz);
      img = tmp;
    }else{
      cv::Mat tmp;
      tmp = img.clone();
      img = tmp;
    }
    
    awsFlip(img, flipx, flipy, false);
    GLenum fmt = GL_LUMINANCE;
    GLenum clr = GL_UNSIGNED_BYTE;
    switch(img.type()){
    case CV_8U:
      break;
    case CV_16U:
      clr = GL_UNSIGNED_SHORT;
      break;
    case CV_8UC3:
      fmt = GL_BGR;
      clr = GL_UNSIGNED_BYTE;
      break;
    case CV_16UC3:
      fmt = GL_BGR;
      clr = GL_UNSIGNED_SHORT;
      break;
    }
    glDrawPixels(img.cols, img.rows, fmt, clr, img.data);
  }
}

void f_ui_manager::print_screen()
{
  if(m_bsvw || m_bss){
    if(m_simg.cols != m_sz_win.width || m_simg.rows != m_sz_win.height)
      m_simg.create(m_sz_win.height, m_sz_win.width, CV_8UC3);
    
    glReadPixels(0, 0, m_sz_win.width, m_sz_win.height, GL_BGR, GL_UNSIGNED_BYTE, m_simg.data);
    awsFlip(m_simg, false, true, false);
    
    if(m_bsvw){
      if(!m_vw.isOpened()){
	char fname[2048];
	double fps = (double) SEC / (double) get_period();
	int fourcc = cv::VideoWriter::fourcc('D', 'I', 'V', 'X');
	snprintf(fname, 2048, "%s/%s_%lld.avi", m_path_storage, m_name, get_time());
	
	m_vw.open(fname, fourcc, fps, m_sz_win, true);
	if(!m_vw.isOpened()){
	  cerr << "Failed to create " << fname << endl;
	  m_bsvw = false;
	}
      }else{
	m_vw.write(m_simg);
      }
    }else{
      if(m_vw.isOpened()){
	m_vw.release();
      }
    }
    
    if(m_bss){
      char fname[2048];
      snprintf(fname, 2048, "%s/%s_%lld.png", m_path_storage, m_name, get_time());
      imwrite(fname, m_simg);
    }
  }
}

void f_ui_manager::update_route()
{
  if (!m_ch_wp)
    return;
  
  owp.disable();
  
  if (!visible_obj[ui_obj_wp])
    return;
  
  m_ch_wp->lock();
  int iwp = 0;
  
  waypoints.resize(m_ch_wp->get_num_wps());
  
  for (m_ch_wp->begin(); !m_ch_wp->is_end(); m_ch_wp->next())
    {
      s_wp wp = m_ch_wp->cur();
      
      eceftowrld(Rmap,
		 pt_map_center_ecef.x, pt_map_center_ecef.y,
		 pt_map_center_ecef.z,
		 wp.x, wp.y, wp.z, wp.rx, wp.ry, wp.rz);

      owp.update_wps(iwp, wp);
      owp.enable(iwp);
      
      waypoints[iwp] = UIManagerMsg::Waypoint(wp.lat, wp.lon);
      iwp++;
    }
  
  float ddiff, course, xdiff;
  m_ch_wp->get_target_course(ddiff, course, xdiff);
  owp.set_next(m_ch_wp->get_next(), ddiff, course, xdiff);
  
  if (obj_mouse_on.type == ot_wp){
    owp.set_focus(obj_mouse_on.handle);
    if (obj_mouse_on.handle != m_ch_wp->get_focus()){
      m_ch_wp->set_focus(obj_mouse_on.handle);
    }
    obj_mouse_on.type = ot_nul;
    obj_mouse_on.handle = -1;
  }
  owp.set_focus(m_ch_wp->get_focus());
  
  m_ch_wp->unlock();
  owp.set_fpv_param(pvm, glm::vec2(m_sz_win.width, m_sz_win.height));
  owp.set_map_param(pix_per_meter, Rmap,
		    pt_map_center_ecef.x, pt_map_center_ecef.y,
		    pt_map_center_ecef.z);
  
  owp.update_drawings();
}

void f_ui_manager::update_ais_objs()
{
  if (!m_ch_ais_obj)
    return;
  
  oais.disable();
  
  if (!visible_obj[ui_obj_vsl])
    return;
  
  m_ch_ais_obj->lock();
  aisobjects.resize(m_ch_ais_obj->get_num_objs());
  int iobj = 0;
  int iobj_visible = 0;
  if (obj_mouse_on.type == ot_ais){
    m_ch_ais_obj->set_track(obj_mouse_on.handle);
    obj_mouse_on.type = ot_nul;
    obj_mouse_on.handle = -1;
  }
  oais.set_focus(-1);
  for (m_ch_ais_obj->begin(); !m_ch_ais_obj->is_end(); m_ch_ais_obj->next()){
    c_ais_obj & obj = m_ch_ais_obj->cur();
    if (m_ch_ais_obj->get_tracking_id() >= 0){
      oais.set_focus(iobj);
    }
    {
      float bear = 0.f, dist = 0.f;
      if (!m_ch_ais_obj->get_pos_bd(bear, dist) || dist > map_range){
	oais.disable(iobj);
      }else{ 
	oais.enable(iobj);
	oais.update_ais_obj(iobj, obj);
	double lat, lon, alt;
	float cog, sog;
	
	obj.get_pos_blh(lat, lon, alt);
	obj.get_vel_blh(cog, sog);
	aisobjects[iobj_visible] = UIManagerMsg::AISObject(obj.get_mmsi(),
							   lat, lon, cog, sog);
	iobj_visible++;
      }
    }
    iobj++;
  }
  aisobjects.resize(iobj_visible);
  m_ch_ais_obj->unlock();  
  
  oais.set_fpv_param(pvm, glm::vec2(m_sz_win.width, m_sz_win.height));
  oais.set_map_param(pix_per_meter, Rmap,
		     pt_map_center_ecef.x, pt_map_center_ecef.y,
		     pt_map_center_ecef.z);
  
  oais.update_drawings();
}

void f_ui_manager::update_map()
{  
  if (!m_ch_map)
    return;
  
  coast_line.set_fpv_param(pvm, glm::vec2(m_sz_win.width, m_sz_win.height));
  coast_line.set_map_param(pix_per_meter, Rmap,
			   pt_map_center_ecef.x, pt_map_center_ecef.y,
			   pt_map_center_ecef.z);
  // bupdate_map is asserted when map_scale is changed or drawing object types are re-selected
  
  if (glm::distance(pt_prev_map_update, pt_map_center_ecef)
      > map_range || bupdate_map){
    // update map
    cout << "Updating map" << endl;
    m_ch_map->lock();
    m_ch_map->set_center(pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z);
    m_ch_map->set_range((float)(2 * map_range));
    m_ch_map->set_resolution((float)(meter_per_pix/4.0));
    //cout << "meter_per_pix:" << meter_per_pix << " range:" << map_range << endl;
    m_ch_map->set_update();
    m_ch_map->unlock();
    pt_prev_map_update = pt_map_center_ecef;
    bupdate_map = false;
  }

  if (m_ch_map->is_ready()) {
    if (visible_obj[ui_obj_cl])
      {
	cout << "Map ready, Updating points." << endl;
	list<AWSMap2::LayerDataPtr> layerData;
	m_ch_map->lock();
	layerData = m_ch_map->get_layer_data(AWSMap2::lt_coast_line);
	coast_line.update_points(layerData);
	m_ch_map->unlock();
	m_ch_map->reset_ready();
      }
  }
  coast_line.update_drawings();
}

f_ui_manager::e_button f_ui_manager::get_col_button()
{
  if (btn_lock_map_own_ship.collision(pt_mouse)) {
    return ebtn_lock_map_own_ship;
  }
  else if (btn_lock_cam_dir_hdg.collision(pt_mouse)) {
    return ebtn_lock_cam_dir_hdg;
  }
  else if (btn_js_ctrl.collision(pt_mouse))
    {
      return ebtn_js_ctrl;
    }

  return ebtn_nul;
}

bool f_ui_manager::handle_btn_pushed()
{
  btn_pushed = get_col_button();
  return btn_pushed != ebtn_nul;
}

bool f_ui_manager::handle_btn_released()
{
  btn_released = get_col_button();
  return btn_released != ebtn_nul;
}

void f_ui_manager::update_button(c_view_mode_box * pvm_box)
{
  switch (pvm_box->get_mode()){
  case ui_mode_fpv:
    if (dir_cam_hdg != 0.0f){
      btn_lock_cam_dir_hdg.set_visible();
      btn_lock_cam_dir_hdg.set_normal();
      btn_lock_map_own_ship.set_invisible();
    }
    else{
      btn_lock_cam_dir_hdg.set_invisible();
      btn_lock_map_own_ship.set_invisible();
    }
    break;
  case ui_mode_map:
    if (bmap_center_free){
      btn_lock_cam_dir_hdg.set_invisible();
      btn_lock_map_own_ship.set_normal();
      btn_lock_map_own_ship.set_visible();      
    }
    else{
      btn_lock_cam_dir_hdg.set_invisible();
      btn_lock_map_own_ship.set_invisible();
    }
    break;
  case ui_mode_sys:
    break;
  }

  if (btn_pushed != ebtn_nul){
    if (btn_released == ebtn_nul){
      switch (pvm_box->get_mode()){
      case ui_mode_fpv:
        if (dir_cam_hdg != 0.0f){
          btn_lock_cam_dir_hdg.set_select();
        }
        break;
      case ui_mode_map:
        if (bmap_center_free){
          btn_lock_map_own_ship.set_select();
        }
        break;
      case ui_mode_sys:
        break;
      }
    }
    else if (btn_released == btn_pushed){
      switch (btn_released){
      case ebtn_lock_cam_dir_hdg:
        dir_cam_hdg = 0.0f;
        break;
      case ebtn_lock_map_own_ship:
        bmap_center_free = false;
        break;
      }
    }
    else{
      switch (pvm_box->get_mode()){
      case ui_mode_fpv:
        if (dir_cam_hdg != 0.0f){
          btn_lock_cam_dir_hdg.set_normal();
        }
        break;
      case ui_mode_map:
        if (bmap_center_free){
          btn_lock_map_own_ship.set_normal();
        }
        break;
      case ui_mode_sys:
        break;
      }
    }
    btn_pushed = btn_released = ebtn_nul;
  }else if (btn_released != ebtn_nul){
    // in touch panel, btn_push is not issued. the handler only need to handle release event.
    switch (btn_released){
    case ebtn_lock_cam_dir_hdg:
      dir_cam_hdg = 0.0f;
      break;
    case ebtn_lock_map_own_ship:
      bmap_center_free = false;
      break;
    case ebtn_js_ctrl:
      bjs = !bjs;
      break;
    }
    btn_pushed = btn_released = ebtn_nul;
  }

  if (bjs) 
    btn_js_ctrl.set_check();
  else
    btn_js_ctrl.set_normal();
}

void f_ui_manager::render_gl_objs(c_view_mode_box * pvm_box)
{	
  // the image is completely fitted to the screen
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  
  if(!m_cam.empty() && pvm_box->get_mode() == ui_mode_fpv){
    double s = (double) m_sz_win.width / (double) m_cam.cols;
    int w = m_sz_win.width;
    int h = (int)(s * (double)m_cam.rows);
    int hcut = h - m_sz_win.height;
    cv::Mat roi;
    if(hcut > 0){
      int hcut_half =  (int)(0.5 * (double) hcut / s);
      roi = m_cam(cv::Rect(0,  hcut_half, w, m_cam.rows - hcut_half));
      h = m_sz_win.height;
      hcut = 0;
    }else{
      roi = m_cam;
    }
    
    cv::Mat temp;
    resize(m_cam, temp, cv::Size(w, h));
    cnvCVBGR8toGLRGB8(temp);
    float ypos = -(float)(1.0 + (double)hcut / (double)m_sz_win.height);
    glRasterPos2f(-1.f, ypos);
    glDrawPixels(temp.cols, temp.rows, GL_RGB, GL_UNSIGNED_BYTE, temp.data);   
  }

  glRasterPos2i(-1, -1);
  
  glUseProgram(p);
  glUniform2fv(loc_inv_sz_half_scrn, 1, inv_sz_half_scrn);
  {
    float t2d[2] = {0.f, 0.f};
    glUniform2fv(loc_t2d, 1, t2d);
  }
  // 3d rendering
  glUniform1i(loc_mode, 0);
  
  glUniformMatrix4fv(loc_Mmvp, 1, GL_FALSE, glm::value_ptr(pvm));
  glUniformMatrix4fv(loc_Mm, 1, GL_FALSE, glm::value_ptr(mm));
  glUniform3fv(loc_Lpar, 1, glm::value_ptr(light));

  oline3d.render(pvm);
  oline3d_map.render(pvm);


  if (pvm_box->get_mode() == ui_mode_map) {
    omap_mask.render();
  }
 
  // 2d rendering
  oline.render();
  orect.render();
  otri.render();
  ocirc.render();
  otxt.render(0);
  oradar.render();
  
  glUseProgram(0);
  // show rendering surface.
  glfwSwapBuffers(pwin());	
}

void f_ui_manager::handle_quit()
{
  char cmd[2048];
  chdir(path_quit_script);
  snprintf(cmd, 2048, "bash ./%s &", quit_script);
  system(cmd);
}

void f_ui_manager::handle_set_ctrl_mode()
{
  if(cmd.ival0 >= ControlSource_NONE ||  cmd.ival0 < 0){
    return;    
  }

  if(cmd.ival0 == ControlSource_AP){
    if(cmd.ival1 >= AutopilotMode_NONE || cmd.ival1 < 0)
      return;
    else {
      ap_mode = (AutopilotMode)cmd.ival1;
      switch(ap_mode){
      case AutopilotMode_STAY:
	{
	  long long t;
	  m_state->get_position_ecef(t, xstay, ystay, zstay);
	}	
	break;
      default:
	break;
      }       
    }
  }

  ctrl_src = (ControlSource) cmd.ival0;
}

void f_ui_manager::handle_set_ctrl_value()
{
  // ControlSource_UI (UI control mode)
  // cmd.fval0 : eng
  // cmd.fval1 : rud
  //
  // ControlSource_AP (Autopilot control mode)
  // cmd.fval0 : rev_tgt or sog_tgt
  // cmd.fval1 : cog_tgt

  switch(ctrl_src){
  case ControlSource_UI: // fval0,fval1-> m_eng_f,m_rud_f
    if(cmd.fval0 >= 0 && cmd.fval0 <= 255)
      m_eng_f = cmd.fval0;
    if(cmd.fval1 >= 0 && cmd.fval1 <= 255)
      m_rud_f = cmd.fval1;
    break;
  case ControlSource_AP: // fval0,fval1-> rev_tgt or sog_tgt,cog_tgt
    if(ap_mode == AutopilotMode_STB_MAN){
      if(cmd.fval0 >= -rev_max && cmd.fval0 <= rev_max)
	rev_tgt = cmd.fval0;
    }else{
      if(cmd.fval0 >= 0 && cmd.fval0 <= sog_max)
	sog_tgt = cmd.fval0;
    }
    if(cmd.fval1 >= 0 && cmd.fval1 <= 360)
      cog_tgt = cmd.fval1;
  }
}

void f_ui_manager::handle_set_ctrl_command()
{
  // ControlSource_UI (UI control mode)
  // cmd.ival0 : eng in {eng_fl_as .. eng_nf}
  // cmd.ival1 : rud in {rud_hap .. rud_has}
  //
  // ControlSource_AP (Autopilot control mode)
  // cmd.ival0 : rev in {rev_fl_as .. rev_nf} or sog in {sog_fl_as .. sog_nf}
  // cmd.ival1 : cog in {cog_p20 .. cog_s20}
  
  switch(ctrl_src){
  case ControlSource_UI: // fval0,fval1-> m_eng_f,m_rud_f
    if(cmd.ival0 >= 0 && cmd.ival0 < eng_undef)
      m_eng_f = eng_cmd_val[cmd.ival0];
    if(cmd.ival1 >= 0 && cmd.ival1 < rud_undef)
      m_rud_f = rud_cmd_val[cmd.ival1];
    break;
  case ControlSource_AP: // fval0,fval1-> rev_tgt or sog_tgt,cog_tgt
    if(ap_mode == AutopilotMode_STB_MAN){
      if(cmd.ival0 >= 0 && cmd.ival0 < rev_undef)
	rev_tgt = rev_cmd_val[cmd.ival0];
    }else{
      if(cmd.ival0 >= 0 && cmd.ival0 < sog_undef)
	sog_tgt = sog_cmd_val[cmd.ival0];
    }
    if(cmd.fval1 >= 0 && cmd.fval1 < cog_undef){
      cog_tgt += cog_cmd_val[cmd.ival1];
      if(cog_tgt > 360)
	cog_tgt -= 360.;
      if(cog_tgt < 0)
	cog_tgt += 360.;      
    }
  }
}

void f_ui_manager::handle_radar_on()
{
  if(m_ch_radar_ctrl){
    m_ch_radar_ctrl->push(RC_TXON); 
  }
}

void f_ui_manager::handle_radar_off()
{
  if(m_ch_radar_ctrl){
    m_ch_radar_ctrl->push(RC_TXOFF);
  }
}

void f_ui_manager::handle_set_radar_range()
{ // cmd.ival0 : range in meter
  if(m_ch_radar_ctrl){
    m_ch_radar_ctrl->push(RC_RANGE, (int)cmd.ival0);
  }
}

void f_ui_manager::handle_set_radar_gain()
{ // cmd.ival0 : RadarControlState 0: Manual, <=1: Auto 
  // cmd.ival1 : Gain value
  if(m_ch_radar_ctrl){
    m_ch_radar_ctrl->push(RC_GAIN,
			  (int)cmd.ival1, (RadarControlState)cmd.ival0);
  }
}

void f_ui_manager::handle_set_radar_sea()
{ // cmd.ival0 : RadarControlState 0: Maunal, <=1: Auto, -1: Off
  // cmd.ival1 : Sea value
  if(m_ch_radar_ctrl){
    m_ch_radar_ctrl->push(RC_SEA,
			  (int)cmd.ival1, (RadarControlState)cmd.ival0);
  }
}

void f_ui_manager::handle_set_radar_rain()
{ // cmd.ival0 : RadarControlState 0: Manual, -1: Off
  // cmd.ival1 : Rain value
  if(m_ch_radar_ctrl){
    m_ch_radar_ctrl->push(RC_RAIN,
			  (int)cmd.ival1, (RadarControlState)cmd.ival0);
  }
}

void f_ui_manager::handle_set_radar_interference()
{ // cmd.ival0 : Interference rejection value
  if(m_ch_radar_ctrl){
    m_ch_radar_ctrl->push(RC_INTERFERENCE_REJECTION, (int)cmd.ival0);
  }
}

void f_ui_manager::handle_set_radar_speed()
{ // cmd.ival0 : Scan speed 
  if(m_ch_radar_ctrl){
    m_ch_radar_ctrl->push(RC_SCAN_SPEED, (int)cmd.ival0);
  }
}

void f_ui_manager::handle_add_waypoint()
{ // cmd.ival0 : handle
  // cmd.ival1 : 0: relative 1: absolute
  // relative mode (relative to the map center)
  // cmd.fval2 : x (east positive)
  // cmd.fval3 : y (north positive)
  //
  // absolute mode
  // cmd.fval2 : lat
  // cmd.fval3 : lon

  if(m_ch_wp){
    double lat, lon;
    if(cmd.ival1 == 0){ // relative mode
      double x, y, z, alt;
      wrldtoecef(Rmap, 
		 (double)pt_map_center_ecef.x, (double)pt_map_center_ecef.y,
		 (double)pt_map_center_ecef.z,
		 (double)cmd.fval2, (double)cmd.fval3, 0.,
		 x, y, z);
      eceftoblh(x, y, z, lat, lon, alt);      
    }else if(cmd.ival1 == 1){ // absolute mode
      lat = cmd.fval2 * (PI / 180.f);
      lon = cmd.fval3 * (PI / 180.f);
    }
    
    m_ch_wp->lock();
    int cur_focus = m_ch_wp->get_focus();    
    m_ch_wp->set_focus((int)cmd.ival0);
    m_ch_wp->ins(lat, lon, 10.0, 0.f);
    m_ch_wp->set_focus(cur_focus);
    m_ch_wp->unlock();
  }
}

void f_ui_manager::handle_del_waypoint()
{ // cmd.ival0 : handle
  if(m_ch_wp){
    m_ch_wp->lock();
    int cur_focus = m_ch_wp->get_focus();    
    m_ch_wp->set_focus((int)cmd.ival0);
    m_ch_wp->ers();
    m_ch_wp->set_focus(cur_focus);    
    m_ch_wp->unlock();
  }
} 

void f_ui_manager::handle_reverse_waypoints()
{
  if(m_ch_wp){
    m_ch_wp->lock();
    m_ch_wp->rev();
    m_ch_wp->unlock();
  }
}

void f_ui_manager::handle_refresh_waypoints()
{
  if(m_ch_wp){
    m_ch_wp->lock();
    m_ch_wp->ref();
    m_ch_wp->unlock();
  }
}

void f_ui_manager::handle_set_map_range()
{ // cmd.ival0 : range in meter
  map_range = (int)cmd.ival0;
  recalc_range();
  bupdate_map = true;
}

void f_ui_manager::handle_set_map_center()
{
  // cmd.ival0 : 0: map center as own ship position,
  //             1: map center as fval2, fval3
  //             2: map move fval2(dx), fval3(dy)
  // cmd.fval2,fval3: lat, lon

  if(cmd.ival0 == 0){
    bmap_center_free = false;
  }else if(cmd.ival0 == 1){
    bmap_center_free = true;

    pt_map_center_blh.x = (float)(cmd.fval2 * (PI/180.f));
    pt_map_center_blh.y = (float)(cmd.fval3 * (PI/180.f));
    double x, y, z;
    blhtoecef((double)pt_map_center_blh.x, (double)pt_map_center_blh.y, 0,
	      x, y, z);
    pt_map_center_ecef.x = x;
    pt_map_center_ecef.y = y;
    pt_map_center_ecef.z = z;
    getwrldrot((double)pt_map_center_blh.x, (double)pt_map_center_blh.y, Rmap);
  }else if(cmd.ival0 == 2){
    bmap_center_free = true;
    double x, y, z, lat, lon, alt;
    wrldtoecef(Rmap, 
	       (double)pt_map_center_ecef.x, (double)pt_map_center_ecef.y,
	       (double)pt_map_center_ecef.z,
	       (double)cmd.fval2, (double)cmd.fval3, 0.,
	       x, y, z);
    pt_map_center_ecef.x = (float) x;
    pt_map_center_ecef.y = (float) y;
    pt_map_center_ecef.z = (float) z;	     
    eceftoblh(x, y, z,
	      lat, lon, alt);

    pt_map_center_blh.x = (float) lat;
    pt_map_center_blh.y = (float) lon;
    getwrldrot(lat, lon, Rmap);
  }
}

void f_ui_manager::handle_set_map_obj()
{ // cmd.ival0 : obj id to be switched
  // cmd.ival1 : 0: disable 1: enable
  if(cmd.ival0 >= 0 && cmd.ival0 < ot_nul)
    visible_obj[cmd.ival0] = cmd.ival1 == 1;
}

void f_ui_manager::command_handler()
{
  switch(cmd.id){
  case UC_NULL:
    return;
  case UC_QUIT:
    handle_quit();
    break;
  case UC_SETCTRLMODE:
    handle_set_ctrl_mode();
    break;
  case UC_SETCTRLV:
    handle_set_ctrl_value();
    break;
  case UC_SETCTRLC:
    handle_set_ctrl_command();
    break;
  case UC_RDON:
    handle_radar_on();
    break;
  case UC_RDOFF:
    handle_radar_off();
    break;
  case UC_SETRDRANGE:
    handle_set_radar_range();
    break;
  case UC_SETRDGAIN:
    handle_set_radar_gain();
    break;
  case UC_SETRDSEA:
    handle_set_radar_sea();
    break;
  case UC_SETRDRAIN:
    handle_set_radar_rain();
    break;
  case UC_SETRDIFR:
    handle_set_radar_interference();
    break;
  case UC_SETRDSPD:
    handle_set_radar_speed();
    break;
  case UC_ADDWP:
    handle_add_waypoint();
    break;
  case UC_DELWP:
    handle_del_waypoint();
    break;
  case UC_REVWPS:
    handle_reverse_waypoints();
    break;
  case UC_REFWPS:
    handle_refresh_waypoints();
    break;
  case UC_SETMAPRANGE:
    handle_set_map_range();
    break;
  case UC_SETMAPCENTER:
    handle_set_map_center();
    break;
  case UC_SETMAPOBJ:
    handle_set_map_obj();
    break;
  }
  cmd.id = UC_NULL;
}

bool f_ui_manager::proc()
{
  // loading states
  long long t = 0;
  float roll, pitch, yaw, cog, sog, vx, vy;
  double xown, yown, zown;
  m_state->get_attitude(t, roll, pitch, yaw);

  roll = normalize_angle_deg(roll);
  pitch = normalize_angle_deg(pitch);
  yaw = normalize_angle_deg(yaw);
  
  m_state->get_corrected_velocity(t, cog, sog);
  m_state->get_corrected_velocity_vector(t, vx, vy);
  m_state->get_position_ecef(t, xown, yown, zown);
  
  double lat, lon;
  float alt;
  double Rown[9];
  m_state->get_enu_rotation(t, Rown);
  m_state->get_position(t, lat, lon);
  m_state->get_alt(t, alt);
  
  float rpm = 0.0f;
  unsigned char trim = 0;
  int poil = 0;
  float toil = 0.0f;
  float temp = 0.0f;
  float valt = 0.0f;
  float frate = 0.0f;
  unsigned int teng = 0;
  int pclnt = 0;
  int pfl = 0;
  unsigned char ld = 0;
  unsigned char tq = 0;
  NMEA2000::EngineStatus1 steng1 =
    (NMEA2000::EngineStatus1)(NMEA2000::EngineStatus1_MAX + 1);
  NMEA2000::EngineStatus2 steng2 =
    (NMEA2000::EngineStatus2)(NMEA2000::EngineStatus2_MAX + 1);
  if(m_engstate){ // currentlly only for main engine
    m_engstate->get_rapid(t, rpm, trim);
    m_engstate->get_dynamic(t, poil, toil, temp, valt,
			    frate, teng, pclnt, pfl, steng1, steng2, ld, tq);
  }
  
  float depth;
  m_state->get_depth(t, depth);

  float bar, temp_air, hmdr, dew, dir_wnd_t, wspd_mps;
  bar = temp_air = hmdr = dew = dir_wnd_t = wspd_mps = 0.0f;
  m_state->get_weather(t, bar, temp_air, hmdr, dew, dir_wnd_t, wspd_mps);
  
  c_view_mode_box * pvm_box =
    dynamic_cast<c_view_mode_box*>(uim.get_ui_box(c_aws_ui_box_manager::view_mode));
    
  snd_ctrl_inst(Rown, xown, yown, zown, cog, rpm);
  
  rcv_ctrl_stat();

  // Window forcus is now at this window
  glfwMakeContextCurrent(pwin());
  
  // UI polling.
  glfwPollEvents();
   
  // handling ui events
  bool event_handled = uim.set_mouse_event(pt_mouse,
					   mouse_button, mouse_action,
					   mouse_mods);
  
  // calculating mouse point coordinate
  calc_mouse_enu_and_ecef_pos(pvm_box->get_mode(), Rown,
			      lat, lon, xown, yown, zown, yaw);
  if (event_handled){
    handle_updated_ui_box(pvm_box);
  }
  else{
    handle_base_mouse_event(pvm_box);
  }

  command_handler();
    
  // update ui params 
  update_ui_params(pvm_box, xown, yown, zown, vx, vy, yaw);
  
  // updating indicator
  update_indicator(cog, sog, roll, pitch, yaw,
		   rpm, trim, poil, toil, temp, valt,
		   frate, teng, pclnt, pfl, ld,
		   tq, steng1, steng2, depth);
 
  // update route object
  update_route();
  
  // update ais object
  update_ais_objs();
  
  // update coast line (map)
  update_map();
  
  // update button
  update_button(pvm_box);

  // update radar data
  if(m_ch_radar_image){
    const int range_meters = m_ch_radar_image->get_range_meters();
    /*
      Mat ppi=Mat::zeros(GARMIN_XHD_MAX_SPOKE_LEN*2,
      GARMIN_XHD_MAX_SPOKE_LEN*2,CV_8UC1);
    
      Mat img(GARMIN_XHD_SPOKES, GARMIN_XHD_MAX_SPOKE_LEN, CV_8UC1);  
      m_ch_radar_image->get_spoke_data(img.data);
     
      double spoke_angle = (double)(2 * PI) / (double)GARMIN_XHD_SPOKES;
      for (int r = 0; r < ppi.rows; r++){
      unsigned char * ptr = ppi.ptr<unsigned char>(r);
      for (int c = 0; c < ppi.cols; c++){
      int x = c - GARMIN_XHD_MAX_SPOKE_LEN;
      int y = r - GARMIN_XHD_MAX_SPOKE_LEN;
      int ispoke = (int)((double)GARMIN_XHD_SPOKES * atan2(x, y) * ( 0.5 / PI));
      ispoke = (ispoke + GARMIN_XHD_SPOKES * 2) % GARMIN_XHD_SPOKES;
	
      int d = (int)(0.5 + sqrt((double)(x * x + y * y)));
      unsigned char val;
      if(d >= GARMIN_XHD_MAX_SPOKE_LEN || d < 0)
      val = 255;
      else
      val = *(img.data + ispoke * GARMIN_XHD_MAX_SPOKE_LEN + d);
      *(ptr + c) = val;
      //	cout << "r,c=" << r << "," << c << endl;
      }
      }
      oradar.update_image(ppi);
    */
    int bearing_last = oradar.get_bearing_last_update();
    int bearing_min = bearing_last + 1;
    int bearing_max = bearing_last;
    long long t_last = oradar.get_time_last_update();
    long long t_new = t_last;

    /*
      for(auto itr = updates.begin(); itr != updates.end(); itr++){
      if((*itr)->time < t_last){ // past spoke is ignored
      continue;
      }
      t_new = max((*itr)->time, t_new);
      int bearing = (*itr)->bearing;                  
      if(bearing  < bearing_min){
      bearing += GARMIN_XHD_SPOKES;
      }
      bearing_max = max(bearing, bearing_max);      
      }

      if(bearing_max - bearing_last > 0){
      Mat spokes_update = Mat::zeros(bearing_max - bearing_last,
      GARMIN_XHD_MAX_SPOKE_LEN, CV_8UC1);
      for(auto itr = updates.begin(); itr != updates.end(); itr++){
      if((*itr)->time < t_last){ // past spoke is ignored
      continue;
      }
      int bearing = (*itr)->bearing;
      if(bearing  < bearing_min){
      bearing += GARMIN_XHD_SPOKES;
      }
      unsigned char * spoke = spokes_update.ptr<unsigned char>(bearing - bearing_min);
      //	cout << "update:" << (*itr)->bearing << "," << bearing << "," <<bearing_last << "," << bearing_max << "," << bearing_min << endl;
      memcpy((void*)spoke, (void*)(*itr)->line, sizeof(unsigned char) * (*itr)->len);
	
      }
      if(bearing_min >= GARMIN_XHD_SPOKES){
      bearing_min -= GARMIN_XHD_SPOKES;
      bearing_max -= GARMIN_XHD_SPOKES;
      }
      oradar.update_spokes(t_new, range_meters, bearing_min, bearing_max,
      spokes_update.data);
      }
    */

    const vector<s_radar_line*> & updates = m_ch_radar_image->get_updates();
    
    for(auto itr = updates.begin(); itr != updates.end(); itr++){
      oradar.update_spoke((*itr)->time, (*itr)->pos.lat,
			  (*itr)->pos.lon, range_meters, (*itr)->bearing,
			  (*itr)->len, (*itr)->line);
    }
    
    oradar.update_done();

    m_ch_radar_image->free_updates(get_time());
  }

  set_ui_message(lat, lon, alt, cog, sog, roll, pitch, yaw, rpm, trim, teng,
		 valt, frate, temp, bar, temp_air, hmdr, dew, dir_wnd_t,
		 wspd_mps, depth);
  
  // rendering graphics
  render_gl_objs(pvm_box);
  
  // screen shot or video capturue
  print_screen();
  
  return true;
}

void f_ui_manager::snd_ctrl_inst(const double * Rown, const double xown,
				 const double yown, const double zown,
				 const float cog, const float rpm)
{
  if (replay){
    long long t = get_time();
    if(!log_ctrl.read(t, buf_ctrl_in, sz_buf_ctrl_in)){
      spdlog::error("[{}] Failed to get control log data.", get_name());
    }

    if(m_ch_ctrl_out && sz_buf_ctrl_in > 0)
      m_ch_ctrl_out->push(buf_ctrl_in, sz_buf_ctrl_in);
    
    return;
  }
  
  // process joypad inputs
  if(m_js.id != -1){
    m_js.set_btn();
    m_js.set_stk();
  }

  if (m_js.estart & s_jc_u3613m::EB_EVUP)
    bjs = !bjs;

  js_force_ctrl_stop();

  switch(ctrl_src){
  case ControlSource_UI:
    handle_ctrl_crz();
    cog_tgt = cog;
    if(eng_cmd_val[eng_ds_ah] <= eng_ctrl) // ahead
      rev_tgt = rpm;
    else if (eng_cmd_val[eng_ds_as] >=  eng_ctrl) //astern
      rev_tgt = -rpm;
    break;
  case ControlSource_AP:
    switch(ap_mode){
    case AutopilotMode_WP:
      if(!m_ch_wp->is_finished()){
	float d, xdiff;
	m_ch_wp->lock();
	m_ch_wp->get_target_course(d, cog_tgt, xdiff);
	m_ch_wp->unlock();
	ctrl_sog_tgt();      
	set_ctrl_course();	
      }else{
	sog_tgt = 0.0;
	set_ctrl_speed();
      }      
      if(eng_cmd_val[eng_ds_ah] <= eng_ctrl) // ahead
	rev_tgt = rpm;
      else if (eng_cmd_val[eng_ds_as] >=  eng_ctrl) //astern
	rev_tgt = -rpm;
      break;
    case AutopilotMode_STAY:{
      eceftowrld(Rown, xown, yown, zown, xstay, ystay, zstay,
		 xrstay, yrstay, zrstay);
      float d_stay = (float)(sqrt(xrstay * xrstay + yrstay * yrstay));      
      cog_tgt = (float)(atan2(xrstay, yrstay) * 180. / PI);
      if(cog_tgt != cog_tgt){
	cog_tgt = 0.0f;
      }
      
      if(d_stay < 15){
	rev_tgt = 0;
      }else if(d_stay < 30){
	rev_tgt = 800;
      }else if(d_stay < 100){
	rev_tgt = (1500 - 800) * (d_stay - 30.) / (100.- 30.) + 800;
      }else if(d_stay < 1000){
	rev_tgt = (5000 - 1500) * (d_stay - 100) / (1000-100) + 1500;
      }
      set_ctrl_course();
      set_ctrl_revolution();
    }
      break;
    case AutopilotMode_STB_MAN:
      handle_ctrl_stb();      
      break;
    }
    // UI control values follow the ap's ones.
    m_eng_f = eng_ctrl; 
    m_rud_f = rud_ctrl;
    break;
  default:
    break;
  }  
}

void f_ui_manager::rcv_ctrl_stat()
{
  if(m_ch_ctrl_in){
    while(1){
      m_ch_ctrl_in->pop(buf_ctrl_in, sz_buf_ctrl_in);
      if(sz_buf_ctrl_in == 0)
	break;
      auto data = Control::GetData(buf_ctrl_in);
      switch(data->payload_type()){
      case Control::Payload_Engine:
	eng_ctrl = data->payload_as_Engine()->value();
	break;
      case Control::Payload_Revolution:
	rev_ap = data->payload_as_Revolution()->value();
	break;
      case Control::Payload_Speed:
	spd_ap = data->payload_as_Speed()->value();
	break;
      case Control::Payload_Rudder:
	rud_ctrl = data->payload_as_Rudder()->value();
	break;
      case Control::Payload_Course:
	cog_ap = data->payload_as_Course()->value();
	break;
      case Control::Payload_Config:{
	auto config = data->payload_as_Config();
	engine_max = config->engine_max();
	engine_min = config->engine_min();
	engine_forward = config->engine_forward();
	engine_backward = config->engine_backward();
	engine_nutral = config->engine_nutral();
	rudder_max = config->rudder_max();
	rudder_min = config->rudder_min();
	rudder_mid = config->rudder_mid();	
	if(m_ch_ctrl_out) m_ch_ctrl_out->push(buf_ctrl_in, sz_buf_ctrl_in);
      }
	break;
      default:
	break;
      }
    }    
  }  
}


void f_ui_manager::set_ui_message(const double lat, const double lon,
				  const float alt, const float cog,
				  const float sog, const float roll,
				  const float pitch, const float yaw,
				  const float rpm, const unsigned char trim,
				  const unsigned int teng, const float valt,
				  const float frate, const float temp,
				  const float bar, const float temp_air,
				  const float hmdr, const float dew,
				  const float dir_wnd_t, const float wspd_mps,
				  const float depth)
{
  // update ui state message
  msg_builder.Clear();
  
  UIManagerMsg::Control control(ctrl_src,
				ap_mode,
				m_eng_f, m_rud_f,
				cog_tgt, sog_tgt, rev_tgt);
  UIManagerMsg::Position position(lat, lon, alt);
  UIManagerMsg::Velocity velocity(cog, sog);
  UIManagerMsg::Attitude attitude(roll, pitch, yaw);
  UIManagerMsg::Engine engine(rpm, trim, teng, valt, frate, temp);
  UIManagerMsg::Weather weather(bar, temp_air, hmdr, dew, wspd_mps, dir_wnd_t);
  UIManagerMsg::Map map(map_range, bmap_center_free,
			UIManagerMsg::Position((double)pt_map_center_blh.x,
					       (double)pt_map_center_blh.y,
					       0.0)
			);
  Radar radar = (m_ch_radar_state ?
		 Radar((RadarState)m_ch_radar_state->get_state(),
		       m_ch_radar_state->get_range(),
		       m_ch_radar_state->get_gain(),
		       (RadarControlState)m_ch_radar_state->get_gain_state(),
		       m_ch_radar_state->get_rain(),
		       (RadarControlState)m_ch_radar_state->get_rain_mode(),
		       m_ch_radar_state->get_sea(),
		       (RadarControlState)m_ch_radar_state->get_sea_mode(),
		       m_ch_radar_state->get_interference_rejection(),
		       m_ch_radar_state->get_scan_speed()) : Radar());
  
  auto msg_loc = UIManagerMsg::CreateUIManagerMsg(msg_builder,
						  get_time(),
						  &control,
						  &position,
						  &velocity,
						  &attitude,
						  &engine, 
						  depth,
						  &weather,
						  &map,
						  &radar,
						  msg_builder.CreateVectorOfStructs(waypoints),
						  msg_builder.CreateVectorOfStructs(aisobjects)
						  );
  msg_builder.Finish(msg_loc);  
}

void f_ui_manager::ctrl_cog_tgt()
{
  if (m_js.id != -1 && bjs){
    cog_tgt += (float)(m_js.lr2 * (255. / 90.));
    cog_tgt = (float)(cog_tgt < 0 ? cog_tgt + 360.0 : (cog_tgt >= 360.0 ? cog_tgt - 360.0 : cog_tgt));
  }

  set_ctrl_course();
}

void f_ui_manager::ctrl_sog_tgt()
{
  if (m_js.id != -1 && bjs){
    sog_tgt -= (float)(m_js.ud1 * (sog_max / 90.));
    sog_tgt = max(min(sog_tgt, sog_max), 0.f);
  }

  set_ctrl_speed();
}

void f_ui_manager::ctrl_rev_tgt()
{
  if (m_js.id != -1 && bjs){
    rev_tgt -= (float)( m_js.ud1 * (rev_max / 90.));
    rev_tgt = (float) max(min(rev_tgt, rev_max), -rev_max);
  }
  
  set_ctrl_revolution();
}



void f_ui_manager::handle_ctrl_crz()
{
  // Joystic assignment
  // left stick up/down -> main engine throttle (disabled when neutral state)
  // right stic up/down -> sub engine throttle (disabled when neutral state)
  // x up/down -> engine command (up->ahead, down->astern)
  // left or right left/right -> rudder control

  if (m_js.id != -1 && bjs){
    m_rud_f += (float)(m_js.lr1 * (255. / 90.));
    m_rud_f += (float)(m_js.lr2 * (255. / 90.));
    m_rud_f = min((float)255.0, m_rud_f);
    m_rud_f = max((float)0.0, m_rud_f);
    if(m_eng_f >= eng_cmd_val[eng_ds_ah]){
      m_eng_f -= (float)(m_js.ud1 * (255. / 90));
    }
    if(m_eng_f <= eng_cmd_val[eng_ds_as]){
      m_eng_f += (float)(m_js.ud1 * (255. / 90));
    }
    m_eng_f = min((float)255.0, m_eng_f);
    m_eng_f = max((float)0.0, m_eng_f);

    e_eng_cmd eng_cm = eng_undef;
    e_rud_cmd rud_cm = rud_undef;
    if (m_js.eux & s_jc_u3613m::EB_EVUP) {
      for (eng_cm = eng_fl_as;
	   eng_cm != eng_nf && m_eng_f >= eng_cmd_val[eng_cm];
	   eng_cm = (e_eng_cmd)(eng_cm + 1));
    }
      
    if (m_js.edx & s_jc_u3613m::EB_EVUP) {
      for(eng_cm = eng_nf;
	  eng_cm != eng_fl_as && m_eng_f <= eng_cmd_val[eng_cm];
	  eng_cm = (e_eng_cmd)(eng_cm - 1));
    }

    if (m_js.elx & s_jc_u3613m::EB_EVUP) {
      for (rud_cm = rud_has;
	   rud_cm != rud_hap && m_rud_f <=  rud_cmd_val[rud_cm];
	   rud_cm = (e_rud_cmd)(rud_cm - 1));
    }
    
    if (m_js.erx & s_jc_u3613m::EB_EVUP) {
      for (rud_cm = rud_hap;
	   rud_cm != rud_has && m_rud_f >= rud_cmd_val[rud_cm];
	   rud_cm = (e_rud_cmd)(rud_cm + 1));
    }
    
    if(eng_cm != eng_undef)
      m_eng_f = eng_cmd_val[eng_cm];

    if(rud_cm != rud_undef)
      m_rud_f = rud_cmd_val[rud_cm];
  }
  
  set_ctrl_engine();
  set_ctrl_rudder();  
}

void f_ui_manager::handle_ctrl_ctl()
{
  if (m_js.id != -1){
    m_rud_f = 127.5;
    m_rud_f += (float)(m_js.lr1 * (127.5));
    m_rud_f += (float)(m_js.lr2 * (127.5));
    m_rud_f = min((float)255.0, m_rud_f);
    m_rud_f = max((float)0.0, m_rud_f);
    
    m_eng_f = 127.5;
    m_eng_f -= (float)(m_js.ud1 * (127.5));
    m_eng_f = min((float)255.0, m_eng_f);
    m_eng_f = max((float)0.0, m_eng_f);    
  }
  set_ctrl_engine();
  set_ctrl_rudder();    
}

void f_ui_manager::handle_ctrl_stb()
{
  ctrl_cog_tgt();
  ctrl_rev_tgt();
}


void f_ui_manager::calc_mouse_enu_and_ecef_pos(
					       e_ui_mode vm, double * Rown,
					       const float lat, const float lon,
					       const float xown, const float yown,
					       const float zown, const float yaw)
{
  if (vm == ui_mode_map)
    {
      if (!bmap_center_free){
	pt_map_center_ecef.x = xown;
	pt_map_center_ecef.y = yown;
	pt_map_center_ecef.z = zown;
	pt_map_center_blh.x = (float)(lat * PI / 180.0f);
	pt_map_center_blh.y = (float)(lon * PI / 180.0f);
	memcpy(Rmap, Rown, sizeof(Rmap));
      }
      pt_mouse_enu.x = pt_mouse.x * meter_per_pix;
      pt_mouse_enu.y = pt_mouse.y * meter_per_pix;
      pt_mouse_enu.z = 0.f;
      double alt = 0;
      wrldtoecef(Rmap, pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z,
		 pt_mouse_enu.x, pt_mouse_enu.y, pt_mouse_enu.z,
		 pt_mouse_ecef.x, pt_mouse_ecef.y, pt_mouse_ecef.z);
      eceftoblh(pt_mouse_ecef.x, pt_mouse_ecef.y, pt_mouse_ecef.z,
		pt_mouse_blh.x, pt_mouse_blh.y, alt);
    }
  else if (vm == ui_mode_fpv)
    {
      pt_map_center_ecef.x = xown;
      pt_map_center_ecef.y = yown;
      pt_map_center_ecef.z = zown;
      pt_map_center_blh.x = lat;
      pt_map_center_blh.y = lon;
      memcpy(Rmap, Rown, sizeof(Rmap));
      
      glm::vec2 phi(atan2(pt_mouse.x, fcam), atan2(-pt_mouse.y, fcam));
      float th, RE_sin_th;
      if (phi.y > th_horizon) {
	phi.y = th_horizon;
	th = th_horizon;
      }
      else {
	th = (float)(phi.y - asin((height_cam_ec * iRE) * cos(phi.y)) - 0.5 * PI);
      }
      RE_sin_th = (float)(RE * sin(th));
      glm::vec3 pcam(
		     (float)(RE_sin_th * sin(phi.x)),
		     (float)(RE_sin_th * (-pt_mouse.y * ifcam)),
		     (float)(RE_sin_th * cos(phi.x))
		     );
      
      double c, s, thcam = ((PI / 180.0) * (yaw + dir_cam_hdg));
      c = cos(thcam);
      s = sin(thcam);
      
      pt_mouse_enu.x = (c * pcam.x + s * pcam.y);
      pt_mouse_enu.y = (-s * pcam.x + c * pcam.y);
      pt_mouse_enu.z = (pcam.z - height_cam);
      
      double alt = 0;
      wrldtoecef(Rmap,
		 pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z,
		 pt_mouse_enu.x, pt_mouse_enu.y, pt_mouse_enu.z,
		 pt_mouse_ecef.x, pt_mouse_ecef.y, pt_mouse_ecef.z);
      eceftoblh(pt_mouse_ecef.x, pt_mouse_ecef.y, pt_mouse_ecef.z,
		pt_mouse_blh.x, pt_mouse_blh.y, alt);
    }
}


void f_ui_manager::drag_waypoint()
{
  m_ch_wp->lock();
  s_wp & wp = m_ch_wp->get_focused_wp();
  wp.lat = pt_mouse_blh.x;
  wp.lon = pt_mouse_blh.y;
  blhtoecef(wp.lat, wp.lon, 0., wp.x, wp.y, wp.z);
  
  m_ch_wp->unlock();
}

void f_ui_manager::drag_map()
{
  if (!bmap_center_free){
    bmap_center_free = true;
  }
  // change map center
  glm::vec2 d_pt_mouse = pt_mouse_drag_begin - pt_mouse;
  if(d_pt_mouse.x == 0.f && d_pt_mouse.y == 0.f)
    return;
  
  d_pt_mouse.x *= meter_per_pix;
  d_pt_mouse.y *= meter_per_pix;
  double xnew, ynew, znew, latnew, lonnew, altnew;
  xnew = ynew = znew = latnew = lonnew = altnew = 0;
  
  wrldtoecef(Rmap, 
	     (double)pt_map_center_ecef.x, (double)pt_map_center_ecef.y, (double)pt_map_center_ecef.z,
	     (double)d_pt_mouse.x, (double)d_pt_mouse.y, 0.,
	     xnew, ynew, znew);
  pt_map_center_ecef.x = (float) xnew;
  pt_map_center_ecef.y = (float) ynew;
  pt_map_center_ecef.z = (float) znew;

  eceftoblh(xnew, ynew, znew,
	    latnew, lonnew, altnew);
  getwrldrot(latnew, lonnew, Rmap);
  pt_map_center_blh.x = (float)latnew;
  pt_map_center_blh.y = (float)lonnew;
  pt_mouse_drag_begin = pt_mouse;
}

void f_ui_manager::drag_cam_dir()
{
  float th0 = (float)atan2(pt_mouse.x, fcam);
  float th1 = (float)atan2(pt_mouse_drag_begin.x, fcam);
  dir_cam_hdg_drag = (float)(th1 - th0);
}

void f_ui_manager::det_obj_collision()
{
  int handle;
  obj_mouse_on.handle = -1;
  obj_mouse_on.type = ot_nul;
  
  handle = owp.collision(pt_mouse);
  if (handle >= 0){
    obj_mouse_on.handle = handle;
    obj_mouse_on.type = ot_wp;
  }
  
  handle = oais.collision(pt_mouse);
  if (handle >= 0){
    obj_mouse_on.handle = handle;
    obj_mouse_on.type = ot_ais;
  }
}

void f_ui_manager::handle_base_mouse_event(c_view_mode_box * pvm_box)
{
  if (mouse_button == GLFW_MOUSE_BUTTON_LEFT){
    if (mouse_action == GLFW_PRESS){
      handle_mouse_lbtn_push(pvm_box);
    }
    else if (mouse_action == GLFW_RELEASE){
      handle_mouse_lbtn_release(pvm_box);
    }
    clear_mouse_event();
  }
  else{
    handle_mouse_mv(pvm_box);
  }
  
  if (obj_mouse_on.type == ot_nul){
    ocsr.enable_pos();
    ocsr.set_cursor_position(pt_mouse, pt_mouse_blh);
  }
  else {
    ocsr.disable();
  }
}

void f_ui_manager::handle_mouse_lbtn_push(c_view_mode_box * pvm_box)
{
  if (handle_btn_pushed())
    return;

  det_obj_collision();
  switch (mouse_state){
  case ms_normal:
    pt_mouse_drag_begin = pt_mouse;
    mouse_state = ms_drag;
    break;
  }
}

void f_ui_manager::handle_mouse_lbtn_release(c_view_mode_box * pvm_box)
{
  if (handle_btn_released())
    return;
  
  s_obj obj_tmp = obj_mouse_on;
  det_obj_collision();
  switch (mouse_state){
  case ms_drag:   
    handle_mouse_drag(pvm_box, obj_tmp);
    dir_cam_hdg += dir_cam_hdg_drag;
    dir_cam_hdg_drag = 0.f;
    mouse_state = ms_normal;
    break;
  case ms_normal:
    break;
  }
}

void f_ui_manager::handle_mouse_mv(c_view_mode_box * pvm_box)
{
  switch (mouse_state){
  case ms_drag:
    handle_mouse_drag(pvm_box, obj_mouse_on);
  }
}

void f_ui_manager::handle_mouse_drag(c_view_mode_box * pvm_box, s_obj & obj_tmp)
{
  if (obj_tmp.type == ot_wp){
    drag_waypoint();    
  } else if (obj_tmp.type == ot_nul){
    if (pvm_box->get_mode() == ui_mode_fpv){
      // change dir_cam_hdg
      drag_cam_dir();
    }
    else if (pvm_box->get_mode() == ui_mode_map){
      drag_map();
    }
  }
}

void f_ui_manager::handle_updated_ui_box(c_view_mode_box * pvm_box) 
{
  e_mouse_state mouse_state_new = ms_normal;
  switch (uim.get_box_updated()){
  case c_aws_ui_box_manager::view_mode:
    update_view_mode_box(pvm_box);
    break;
  default:
    break;
  }
  
  uim.reset_box_updated();
  
  ocsr.enable_arrow();
  ocsr.set_cursor_position(pt_mouse, pt_mouse_blh);
  clear_mouse_event();
}


void f_ui_manager::update_view_mode_box(c_view_mode_box * pvm_box)
{
  ind.set_mode(pvm_box->get_mode());
  owp.set_ui_mode(pvm_box->get_mode());
  oais.set_ui_mode(pvm_box->get_mode());
  coast_line.set_ui_mode(pvm_box->get_mode());
  if(pvm_box->get_mode() == ui_mode_map){
    for(int i = 0; i < 2; i++)
      omap_mask.enable(hmap_mask[i]);
  }else{
    for(int i = 0; i < 2; i++)
      omap_mask.disable(hmap_mask[i]);
  }  
}



void f_ui_manager::update_ui_params(c_view_mode_box * pvm_box,
				    const double xown, const double yown,
				    const double zown,
				    const float vx, const float vy,
				    const float yaw)
{
  {
    glm::mat4 tm(1.0);
    tm = glm::translate(tm, -pt_map_center_ecef);
    glm::mat4 rm(1.0);
    rm[0][0] = (float)Rmap[0]; rm[1][0] = (float)Rmap[1]; rm[2][0] = (float)Rmap[2];
    rm[0][1] = (float)Rmap[3]; rm[1][1] = (float)Rmap[4]; rm[2][1] = (float)Rmap[5];
    rm[0][2] = (float)Rmap[6]; rm[1][2] = (float)Rmap[7]; rm[2][2] = (float)Rmap[8];
    mm = rm * tm;
  }
  
  if (pvm_box->get_mode() == ui_mode_fpv){ // calculating projection matrix
    float c, s, th = (float)(dir_cam_hdg + yaw * PI / 180.);
    c = (float)cos(th);
    s = (float)sin(th);
    
    float ratio = (float)((float)m_sz_win.width / (float)m_sz_win.height);
    pm = glm::perspective((float)(fov_cam_y * PI / 180.0f), ratio, 1.f, 10000.f/*30e6f*/);
    vm = glm::lookAt(glm::vec3(0, 0, height_cam), glm::vec3(s, c, height_cam), glm::vec3(0, 0, 1));
    pvm = pm * vm;
    own_ship.set_ui_mode(ui_mode_fpv);
    own_ship.disable();
  }
  else if (pvm_box->get_mode() == ui_mode_map){
    own_ship.set_map_param(pix_per_meter, Rmap,
			   pt_map_center_ecef.x, pt_map_center_ecef.y,
			   pt_map_center_ecef.z);
    own_ship.set_ui_mode(ui_mode_map);
    own_ship.enable();
    
    double rx, ry, rz;
    float rxs, rys, rzs, d, dir;
    eceftowrld(Rmap, pt_map_center_ecef.x,
	       pt_map_center_ecef.y, pt_map_center_ecef.z,
	       xown, yown, zown, rx, ry, rz);

    if(ctrl_src == ControlSource_AP && ap_mode == AutopilotMode_STAY){
      own_ship.set_param(rx, ry, rz, xrstay, yrstay, zrstay,
			 yaw, vx, vy, cog_tgt);
    }else{
      own_ship.set_param(rx, ry, rz, 0, 0, 0,
			 yaw, vx, vy, cog_tgt);      
    }
    
    float wx = (float)(meter_per_pix * (m_sz_win.width >> 1)),
      wy = (float)(meter_per_pix * (m_sz_win.height >> 1));
    
    pm = glm::ortho(-wx, wx, -wy, wy, 1.f, 10000.f/*30e6f*/);
    vm = glm::lookAt(glm::vec3(0, 0, 100*height_cam), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
    pvm = pm * vm;
  }
  pvm *= mm;
}

void f_ui_manager::update_indicator(const float cog, const float sog, 
			const float roll, const float pitch, const float yaw,
			const float rpm, const float trim, const int poil,
			const float toil, const float temp, const float valt,
			const float frate, const unsigned int teng,
			const int pclnt, const int pfl, const unsigned char ld,
			const unsigned char tq, const NMEA2000::EngineStatus1 steng1,
			const NMEA2000::EngineStatus2 steng2, const float depth)
{
  float cogf = (float)(cog * (PI / 180.f));
  float yawf = (float)(yaw * (PI / 180.f));
  const char *str_steng1 = ((steng1 >= 0 && steng1 <= NMEA2000::EngineStatus1_MAX) ? strStatEng1[steng1] : NULL);
  const char *str_steng2 = ((steng2 >= 0 && steng2 <= NMEA2000::EngineStatus2_MAX) ? strStatEng2[steng2] : NULL);
  ind.set_param( m_time_str, eng_ctrl, rpm, rev_tgt, trim, poil,
		 toil, temp, valt, frate, teng, pclnt, pfl, ld, tq,
		 str_steng1, str_steng2, rud_ctrl,
		 cogf, (float)(cog_tgt * (PI/180.f)), sog, sog_tgt, yawf,
		 (float)(pitch* (PI / 180.f)), (float)(-roll* (PI / 180.f)),
		 depth);
    
  ind.set_dir_cam(dir_cam_hdg + dir_cam_hdg_drag);
}

void f_ui_manager::_cursor_position_callback(double xpos, double ypos){
  pt_mouse.x = (float)(xpos - (double)(m_sz_win.width >> 1));
  pt_mouse.y = (float)((double)(m_sz_win.height >> 1) - ypos);
}

void f_ui_manager::_mouse_button_callback(int button, int action, int mods)
{
  mouse_button = button;
  mouse_action = action;
  mouse_mods = mods;
}

void f_ui_manager::_key_callback(int key, int scancode, int action, int mods)
{
}
