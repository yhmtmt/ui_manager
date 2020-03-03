// Copyright(c) 2020 Yohei Matsumoto, All right reserved. 

// ui_command.hpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ui_command.hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ui_command.hpp.  If not, see <http://www.gnu.org/licenses/>. 


#ifndef UI_COMMAND_HPP
#define UI_COMMAND_HPP

enum ui_command_id{
  UC_QUIT,         // Issue quit command

  // Control related command
  UC_SETCTRLMODE, // Set control mode (Manual, semi-auto, auto, anchor)
  UC_SETCTRLV,    // Set engine/rudder control value in manual, semi-auto mode
  UC_SETCTRLC,    // Set engine/rudder control command in manual, semi-auto mode

  // Radar related command
  UC_RDON,        // Radar on
  UC_RDOFF,       // Radar off
  UC_SETRDRANGE,    // Set radar range
  UC_SETRDGAIN,   // Set radar gain
  UC_SETRDSEA,    // Set sea clutter reduction
  UC_SETRDRAIN,   // Set rain clutter reduction
  UC_SETRDIFR,    // Set interference rejection
  UC_SETSPD,      // Set scan speed
  
  // waypoint and route related command
  UC_ADDWP,        // Add waypoint
  UC_DELWP,        // Delete waypoint
  UC_REVWPS,       // Reverse waypoint order
  UC_REFWPS,       // Refresh waypoitns

  // map related command
  UC_SETMAPRANGE,       // Set visible range in main view
  UC_SETMAPCENTER, // Set map center in (lat, lon)
  UC_SETMAPOBJ,       // Set visible object in main view
  UC_NULL          // NULL command
};

extern const char * str_ui_command[UC_NULL];

struct s_ui_command{
  ui_command_id id;
  
  union {
    long long ival0;
    double fval0;
  };
  
  union {
    long long ival1;
    double fval1;
  };
  
  union {
    long long ival2;
    double fval2;
  };
  
  union {
    long long ival3;
    double fval3;
  };

  s_ui_command():id(UC_NULL), ival0(0), ival1(0), ival2(0), ival3(0){}  
};

  
#endif
