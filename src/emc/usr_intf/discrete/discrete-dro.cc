/*
 *  Mininmal Linuxcnc Interface
 *
 *     DRO Table Ran from terminal. Intended as a standalone interface to be
 *       ran in Tmux
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

WINDOW *w_dro;

uint16_t mid_top_col, mid_bot_col;
uint16_t mid_left_lines, mid_right_lines;
uint16_t max_h, max_w;


int display_axis[] = {-1, -1, -1, -1, -1,
                      -1, -1, -1, -1, -1};

void init_screen(){
  refresh();
  mid_top_col = max_w/2;
  mid_bot_col = max_w/2;
  mid_right_lines = (max_h/2)-1;
  mid_left_lines  = (max_h/2)-1;

  w_dro = newwin(max_h, max_w, 0, 0);

  box(w_dro, 0, 0);
  nodelay(w_dro, TRUE);

  mvwprintw(w_dro, 0,5  ,"Machine   Program   Velocity");
  wrefresh(w_dro);
}

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

void update_screen(){

  updateStatus();
  for(int i=0; i<10;i++){
    switch (display_axis[i]) {
      case 1: {
        mvwprintw(w_dro, i+1, 1,
                  "X:  %f  %f  %f",
                  emcStatus->motion.traj.actualPosition.tran.x,
                  emcStatus->motion.traj.position.tran.x - emcStatus->task.g5x_offset.tran.x,
                  00);
        break;
      }
      case 2: {
        mvwprintw(w_dro, i+1, 1,
                  "Y:  %f  %f  %f",
                  emcStatus->motion.traj.actualPosition.tran.y,
                  emcStatus->motion.traj.position.tran.y - emcStatus->task.g5x_offset.tran.y,
                  0.123);
        break;
      }
      case 3: {
        mvwprintw(w_dro, i+1, 1,
                  "Z:  %f  %f  %f",
                  emcStatus->motion.traj.actualPosition.tran.z,
                  emcStatus->motion.traj.position.tran.z - emcStatus->task.g5x_offset.tran.z,
                  0.123);
        break;
      }
      case 4: {
        mvwprintw(w_dro, i+1, 1,
                  "A:  %f  %f  %f",
                  emcStatus->motion.traj.actualPosition.a,
                  emcStatus->motion.traj.actualPosition.a - emcStatus->task.g5x_offset.a,
                  0.123);
        break;
      }
      case 5: {
        mvwprintw(w_dro, i+1, 1,
                  "B:  %f  %f  %f",
                  emcStatus->motion.traj.actualPosition.b,
                  emcStatus->motion.traj.actualPosition.b - emcStatus->task.g5x_offset.b,
                  0.123);
        break;
      }
      case 6: {
        mvwprintw(w_dro, i+1, 1,
                  "C:  %f  %f  %f",
                  emcStatus->motion.traj.actualPosition.c,
                  emcStatus->motion.traj.actualPosition.c - emcStatus->task.g5x_offset.c,
                  0.123);
        break;
      }
      case 7: {
        mvwprintw(w_dro, i+1, 1,
                  "U:  %f  %f  %f",
                  emcStatus->motion.traj.actualPosition.u,
                  emcStatus->motion.traj.actualPosition.u - emcStatus->task.g5x_offset.u,
                  0.123);
        break;
      }
      case 8: {
        mvwprintw(w_dro, i+1, 1,
                  "V:  %f  %f  %f",
                  emcStatus->motion.traj.actualPosition.v,
                  emcStatus->motion.traj.actualPosition.v - emcStatus->task.g5x_offset.v,
                  0.123);
        break;
      }
      case 9: {
        mvwprintw(w_dro, i+1, 1,
                  "W:  %f  %f  %f",
                  emcStatus->motion.traj.actualPosition.w,
                  emcStatus->motion.traj.actualPosition.w - emcStatus->task.g5x_offset.w,
                  0.123);
        break;
      }
    }

  }

  wrefresh(w_dro);
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
