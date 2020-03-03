// Copyright(c) 2020 Yohei Matsumoto, All right reserved. 

// ui_command.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ui_command.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ui_command.cpp.  If not, see <http://www.gnu.org/licenses/>. 


#include "ui_command.hpp"

const char * str_ui_command[UC_NULL] = {
  "quit",
  "setctrlmode", "setctrlv", "setctrlc",
  "rdon", "rdoff", "setrdrange", "setrdgain", "setrdsea", "setrdrain", "setrdifr", "setspd",
  "addwp", "delwp", "revwps", "refwps", 
  "setmaprange", "setmapcenter", "setmapobj"
};
