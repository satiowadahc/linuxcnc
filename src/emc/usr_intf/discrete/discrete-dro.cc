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

// #include "config.h"
// #include "emc.hh"
// #include "emc_nml.hh"
#include "emcIniFile.hh" // emcIniFile
// #include "emcglb.h"		// EMC_NMLFILE, TRAJ_MAX_VELOCITY, etc.
// #include "emccfg.h"		// DEFAULT_TRAJ_MAX_VELOCITY
// #include "canon.hh"		// CANON_UNITS, CANON_UNITS_INCHES,MM,CM
// #include "posemath.h"		// PM_POSE, TO_RAD
// #include "rcs_print.hh"
// #include "rcs.hh"
// #include <rtapi_string.h>
// #include "usrmotintf.h" // usrmotIniLoad, usrmotInit

WINDOW *w_dro;
WINDOW *w_test;
uint16_t mid_top_col, mid_bot_col;
uint16_t mid_left_lines, mid_right_lines;
uint16_t max_h, max_w;


int display_axis[] = {-1, -1, -1, -1, -1,
                      -1, -1, -1, -1, -1};

int init_screen(){
  //refresh();
  printf("Max Height: %d\n", max_h);
  printf("Max Width: %d\n", max_w);
  mid_top_col = max_w/2;
  mid_bot_col = max_w/2;
  mid_right_lines = (max_h/2)-1;
  mid_left_lines  = (max_h/2)-1;

  w_dro = newwin(mid_right_lines, mid_top_col, 0, 0);
  printf("tesT");
  box(w_dro, 0, 0);
  //nodelay(w_dro, TRUE);

  // mvprintw(0,5  ,"Machine   Program   Offset");
  // wrefresh(w_dro);
}

int init_machine(){
  EmcIniFile ini_file;

  ini_file.Open("/home/chad/linuxcnc/configs/sim-rmd-rtr/sim-rmd-rtr.ini");
  const char *coord = ini_file.Find("COORDINATES", "TRAJ");
  printf("COORDINATES: %s\n", coord);

  // Resume Thought here and parse the coordinates


  return 0;
}

// <><><><><><><><><><><><><>
//      Main Function
//  TODO Abstract all functions in here
// <><><><><><><><><><><><><>
int main(int argc, char *argv[]) {

  init_machine();

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

//  char ch;
//  bool run = true;
//  while(run) {
//    waddstr(w_dro, "DRO");
//    wrefresh(w_dro);
//    ch = getch();
//    if (ch == 'q') {
//      run = false;
//    }
//  }
  // Screen Initialization
  // These Can not be abstracted from main
  endwin();
  return 0;
}
