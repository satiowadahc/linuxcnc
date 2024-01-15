/*
 *  Mininmal Linuxcnc Interface
 *
 *  Attempt at making a terminal based environment for
 *  Linuxcnc. The hopes of this is to allow for low spec
 *  Computers to be able to run Linuxcnc at a reasonable
 *  clock cycle.
 *
*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE":
 * <satiowadahc> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   Chad A. Woitas
 * ----------------------------------------------------------------------------
 *
 */
#define PY_SSIZE_T_CLEAN

#include <curses.h>
#include <unistd.h>



#include "rs274ngc.hh"
#include "rs274ngc_interp.hh"

#include "kinematics.h"
#include "config.h"
#include "inifile.hh"
#include "rcs_print.hh"
#include "rcs.hh"
#include "posemath.h"		// PM_POSE, TO_RAD
#include "emc.hh"		// EMC NML
#include "emc_nml.hh"
#include "canon.hh"		// CANON_UNITS, CANON_UNITS_INCHES,MM,CM
#include "emcglb.h"		// EMC_NMLFILE, TRAJ_MAX_VELOCITY, etc.
#include "emccfg.h"		// DEFAULT_TRAJ_MAX_VELOCITY
#include "inifile.hh"		// INIFILE
#include "config.h"		// Standard path definitions
#include "rcs_print.hh"
#include "emc/usr_intf/shcom.hh"
#include <rtapi_string.h>


// <><><><><><><><><><><><><>
//      Window Variables
// <><><><><><><><><><><><><>
WINDOW *w_dro;
WINDOW *w_editor;
WINDOW *w_control;
WINDOW *w_status;
WINDOW *w_menu;

uint16_t mid_top_col, mid_bot_col;
uint16_t mid_left_lines, mid_right_lines;

// Global windows
uint16_t max_h, max_w;

bool run = false;

char status_messages[10][40];

// <><><><><><><><><><><><><>
//      Linuxcnc Update Functions
//  TODO Move to own file
// <><><><><><><><><><><><><>

// <><><><><><><><><><><><><>
//   Machine Initializations
// <><><><><><><><><><><><><>
int init_machine(){
  iniLoad(emc_inifile);

  if (tryNml() != 0) {
    rcs_print_error("can't connect to emc\n");
    exit(1);
  }

  return 0;
}

// <><><><><><><><><><><><><>
//    Machine Closures
// <><><><><><><><><><><><><>
void close_machine(){

}

// <><><><><><><><><><><><><>
//    Window Initializations
// <><><><><><><><><><><><><>
void create_dro(){

}

void init_screen(){
  create_dro();

  mid_top_col = max_w/2;
  mid_bot_col = max_w/2;
  mid_right_lines = (max_h/2)-1;
  mid_left_lines  = (max_h/2)-1;
  refresh();
  w_dro = newwin(mid_right_lines, mid_top_col, 0, 0);
  w_editor = newwin(max_h - mid_right_lines - 1, mid_bot_col, mid_right_lines, 0);
  w_status = newwin(mid_left_lines, max_w - mid_top_col, 0, mid_top_col);
  w_control = newwin(max_h - mid_left_lines - 1, max_w - mid_bot_col, mid_left_lines, mid_bot_col);
  w_menu = newwin(1, max_w, max_h-1, 0);
  box(w_dro, 0 , 0);
  box(w_editor, 0 , 0);
  box(w_status, 0 , 0);
  box(w_control, 0 , 0);

  nodelay(w_dro, true);
  nodelay(w_editor, true);
  nodelay(w_status, true);
  nodelay(w_control, true);
  nodelay(w_menu, true);

  // keypad(w_dro, true);
}



// <><><><><><><><><><><><><>
//    Window Closures
// <><><><><><><><><><><><><>
void close_screen(){

}


void updateDRO(WINDOW *dro){
  mvprintw(0,5  ,"Machine   Program   Offset");

  mvwprintw(dro, 1,1,"X:  %f  %f  %f", emcStatus->motion.traj.actualPosition.tran.x,0.123,0.123);
  mvwprintw(dro, 2,1,"Y:  %f  %f  %f", emcStatus->motion.traj.actualPosition.tran.y,0.123,0.123);
  mvwprintw(dro, 3,1,"Z:  %f  %f  %f", emcStatus->motion.traj.actualPosition.tran.z,0.123,0.123);

  wrefresh(dro);
}

void addMessage(char *message){
  for(int i = 9; i > 0; i--){
    strcpy(status_messages[i], status_messages[i-1]);
    mvwprintw(w_status, 3+i, 1, status_messages[i]);
  }
  strcpy(status_messages[0], message);
  mvwprintw(w_status, 3, 1, status_messages[0]);
}

void updateStatusWindow(WINDOW *status){
  updateStatus();
  struct timespec current_timespec;
  clock_gettime(CLOCK_REALTIME_COARSE, &current_timespec);
  char time_str[21];
  strftime(time_str, 20, "%F %T",
           localtime(&current_timespec.tv_sec));
  mvwprintw(status,1, 1, "Current time: %s", time_str);
  switch (emcStatus->task.state) {
    case EMC_TASK_STATE::ESTOP:
      mvwprintw(status,2, 1, "ESTOP   ");
      break;
    case EMC_TASK_STATE::OFF:
    case EMC_TASK_STATE::ESTOP_RESET:
      mvwprintw(status,2, 1, "DISABLED");
      break;
    case EMC_TASK_STATE::ON:
      mvwprintw(status,2, 1, "ENABLED ");
      break;
    default:
      mvwprintw(status,2, 1, "Unknown ");
      break;
  }

  switch(emcStatus->task.execState){
    case EMC_TASK_EXEC::ERROR:
      mvwprintw(status,2, 10, "Error                       ");
      break;
    case EMC_TASK_EXEC::DONE:
      mvwprintw(status,2, 10, "Done                        ");
      break;
    case EMC_TASK_EXEC::WAITING_FOR_MOTION:
      mvwprintw(status,2, 10, "Waiting for Motion          ");
      break;
    case EMC_TASK_EXEC::WAITING_FOR_MOTION_QUEUE:
      mvwprintw(status,2, 10, "Waiting for Motion Queue    ");
      break;
    case EMC_TASK_EXEC::WAITING_FOR_IO:
      mvwprintw(status,2, 10, "Waiting for IO              ");
      break;
    case EMC_TASK_EXEC::WAITING_FOR_MOTION_AND_IO:
      mvwprintw(status,2, 10, "Waiting for Motion and IO   ");
      break;
    case EMC_TASK_EXEC::WAITING_FOR_DELAY:
      mvwprintw(status,2, 10, "Waiting for Delay           ");
      break;
    case EMC_TASK_EXEC::WAITING_FOR_SYSTEM_CMD:
      mvwprintw(status,2, 10, "Waiting for System Command  ");
      break;
    case EMC_TASK_EXEC::WAITING_FOR_SPINDLE_ORIENTED:
      mvwprintw(status,2, 10, "Waiting for Spindle Oriented");
      break;
  }
  switch(emcStatus->task.interpState) {
    case EMC_TASK_INTERP::IDLE:
      mvwprintw(status, 2, 40, "Idle");
      break;
    case EMC_TASK_INTERP::READING:
      mvwprintw(status, 2, 40, "Reading");
      break;
    case EMC_TASK_INTERP::PAUSED:
      mvwprintw(status, 2, 40, "Paused");
      break;
    case EMC_TASK_INTERP::WAITING:
      mvwprintw(status, 2, 40, "Waiting");
      break;
  }

  wrefresh(status);
}


void updateEditor(WINDOW *editor, int line){

}

void updateControl(WINDOW *control){
  char ch = getch();
  char buf[40];

  switch (ch) {
    case 'q':
      run = false;
      break;
    case 'h': {
      EMC_JOINT_HOME emc_joint_home_msg;
      emc_joint_home_msg.joint = -1;
      emcCommandSend(emc_joint_home_msg);
      break;
    }
    case 't': {
      sprintf(buf, "Testing");
      addMessage(buf);
      break;
    }
  }


}

void updateMenu(WINDOW *menu){


  mvwprintw(w_menu, 0, 0, "F1 Reset estop");
  mvwprintw(w_menu, 0, 15, "| F2 Load File");
  mvwprintw(w_menu, 0, 30, "| F3 Offsets");
  mvwprintw(w_menu, 0, 45, "| F4 Editor");
  mvwprintw(w_menu, 0, 60, "| F5 Quit");
  wrefresh(w_menu);

  char ch = getch();
  char buf[40];
  /*
   * TODO: Deside on hotkeys.
   *  I kinda like function keys for this
   *  CTRL + key is also an option - This could be CTRL makes focus to this window.
   *  Could make hal pins to connect to dedicated HMI buttons - That would be cool but then sim?
   *  I would prefer to hide hotkeys but HCI then is not existent until muscle memory is learned.
   */

  switch (ch) {
    case 'q':
      run = false;
      break;
    case 'r': {
      if(emcStatus->task.state == EMC_TASK_STATE::ESTOP) {
        sendEstopReset();
        addMessage("Estop Reset");
      }
      else if(emcStatus->task.state == EMC_TASK_STATE::ESTOP_RESET ||
              emcStatus->task.state == EMC_TASK_STATE::OFF) {
        sendMachineOn();
        addMessage("Machine On");
      }
      break;
    }
    case 'f': {
      sprintf(buf, "TODO: Load File Interface");
      addMessage(buf);
      break;
    }
    case 'o': {
      sprintf(buf, "TODO: Offsets Interface");
      addMessage(buf);
      break;
    }
    case 'e': {
      sprintf(buf, "TODO: Editor Interface, Maybe scrap?");
      addMessage(buf);
      break;
    }

  }

}

// <><><><><><><><><><><><><>
//      Main Function
//  TODO Abstract all functions in here
// <><><><><><><><><><><><><>
int main(int argc, char *argv[]) {

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

  waddstr(w_dro, "DRO");
  wrefresh(w_dro);
  waddstr(w_editor, "Editor");
  wrefresh(w_editor);
  waddstr(w_status, "Status");
  wrefresh(w_status);
  waddstr(w_control, "Control");
  wrefresh(w_control);

  nodelay(stdscr, true);
  intrflush(stdscr, false);
  keypad(stdscr, true);
  curs_set(0);

// <><><><><><><><><><><><><>
//      LCNC Setup
// <><><><><><><><><><><><><>
  init_machine();


// <><><><><><><><><><><><><>
//      Main Loop
// <><><><><><><><><><><><><>
  run = true;
  while(run){

    updateDRO(w_dro);
    wrefresh(w_dro);
    updateStatusWindow(w_status);
    updateMenu(w_menu);
    updateControl(w_control);


  }
  // Clean up

  endwin();
  return 0;
}
