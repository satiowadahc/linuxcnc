/*
 *  Mininmal Linuxcnc Interface
 *
 *     Manual Control for jogging, touchoffs and homing.
 *     Intended as a standalone interface to be ran in Tmux
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE":
 * <satiowadahc> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   Chad A. Woitas
 * ----------------------------------------------------------------------------
 *
 */


#include <curses.h>
#include <unistd.h>

#include "emcIniFile.hh" // emcIniFile
#include "canon.hh"		// CANON_UNITS, CANON_UNITS_INCHES,MM,CM
#include "config.h"
#include "emc.hh"		// EMC NML
#include "emc_nml.hh"
#include "emcglb.h"		// EMC_NMLFILE, TRAJ_MAX_VELOCITY, etc.
#include "emccfg.h"		// DEFAULT_TRAJ_MAX_VELOCITY
#include "inifile.hh"
#include "kinematics.h"
#include "posemath.h"		// PM_POSE, TO_RAD
#include "rcs_print.hh"
#include "rcs.hh"
#include "rs274ngc.hh"
#include "rs274ngc_interp.hh"
#include "emc/usr_intf/shcom.hh"
#include <rtapi_string.h>

WINDOW *w_jogger;
WINDOW *w_jog_settings;
WINDOW *w_controls;

int init_screen(){
  refresh();
  mid_top_col = max_w/2;
  mid_bot_col = max_w/2;
  mid_left_lines = max_h/2;
  mid_right_lines = max_h/2;

  w_jogger = newwin(mid_left_lines, mid_top_col, 0, 0);
  w_jog_settings = newwin(mid_left_lines, mid_bot_col, 0, mid_top_col);
  w_controls = newwin(mid_right_lines, max_w, mid_left_lines, 0);

  box(w_jogger, 0, 0);
  box(w_jog_settings, 0, 0);
  box(w_controls, 0, 0);

  wrefresh(w_jogger);
  wrefresh(w_jog_settings);
  wrefresh(w_controls);

  return 0;
}

int init_jogger(){
  wclear(w_jogger);
  box(w_jogger, 0, 0);

  // Thought process
  // 1 Select Axis
  // 2 Hot key to enable jogging
  // 3 Jogging
  // 4 Hot key to disable jogging

  wrefresh(w_jogger);
  return 0;
}

int init_jog_settings(){
  wclear(w_jog_settings);
  box(w_jog_settings, 0, 0);

  // Thought process
  // Hotkey for incr up
  // Hotkey for incr down
  // Hotkey for jog speed up
  // Hotkey for jog speed down
  // Need to highlight selections
  wrefresh(w_jog_settings);
  return 0;
}

int init_controls(){
  wclear(w_controls);
  box(w_controls, 0, 0);

  // Thought Process
  // Hotkey for homing
  // Hotkey for touch off
  // Hotkey for mist/flood - Can we see if hal signal is connected?
  // Hotkey for spindle on/off
  // Hotkeys for hal pins if enabled?

  wrefresh(w_controls);
  return 0;
}




// <><><><><><><><><><><><><>
// Connect to linuxcnc and load relevant information
// <><><><><><><><><><><><><>
int init_machine(char *inifilename) {

  iniLoad(inifilename);

  if (tryNml() != 0) {
    rcs_print_error("can't connect to emc\n");
    exit(1);
  }

  EmcIniFile ini_file;
  int num_axes = 0;

  ini_file.Open(inifilename);
  const char *coord = ini_file.Find("COORDINATES", "TRAJ");

  if (coord) {
    if(strchr(coord, 'x') || strchr(coord, 'X')) { display_axis[num_axes] = 1; num_axes++; }
    if(strchr(coord, 'y') || strchr(coord, 'Y')) { display_axis[num_axes] = 2; num_axes++; }
    if(strchr(coord, 'z') || strchr(coord, 'Z')) { display_axis[num_axes] = 3; num_axes++; }
    if(strchr(coord, 'a') || strchr(coord, 'A')) { display_axis[num_axes] = 4; num_axes++; }
    if(strchr(coord, 'b') || strchr(coord, 'B')) { display_axis[num_axes] = 5; num_axes++; }
    if(strchr(coord, 'c') || strchr(coord, 'C')) { display_axis[num_axes] = 6; num_axes++; }
    if(strchr(coord, 'u') || strchr(coord, 'U')) { display_axis[num_axes] = 7; num_axes++; }
    if(strchr(coord, 'v') || strchr(coord, 'V')) { display_axis[num_axes] = 8; num_axes++; }
    if(strchr(coord, 'w') || strchr(coord, 'W')) { display_axis[num_axes] = 9; num_axes++; }
  }

  return 0;
}
// <><><><><><><><><><><><><>
//      Main Function
//  TODO Abstract all functions in here
// <><><><><><><><><><><><><>
int main(int argc, char *argv[]) {

  init_machine(argv[1]);

  // Screen Initialization
  // These Can not be abstracted from main
  initscr();
  cbreak();
  noecho();
  getmaxyx(stdscr, max_h, max_w);
  if(max_w < 80 || max_h < 10){
    endwin();
    printf("Terminal must be larger than 80 characters and 10 lines\n");
    return -1;
  }

  init_screen();


  nodelay(stdscr, true);
  intrflush(stdscr, false);
  keypad(stdscr, true);
  curs_set(0);

  char ch;
  bool run = true;
  while(run) {
    update_screen();
    ch = getch();
    if (ch == 'q') {
      run = false;
    }
  }
  // Screen Initialization
  // These Can not be abstracted from main
  endwin();
  return 0;
}
