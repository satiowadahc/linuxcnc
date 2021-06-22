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


#include <curses.h>
#include <unistd.h>
#include <linuxcnc/emcpos.h>
#include <linuxcnc/emc_nml.hh>
#include <linuxcnc/rtapi.h>
#include <linuxcnc/emc.hh>



// <><><><><><><><><><><><><>
//      Window Variables
// <><><><><><><><><><><><><>
WINDOW *w_dro;
WINDOW *w_editor;
WINDOW *w_messages;
WINDOW *w_status;

uint16_t mid_top_col, mid_bot_col;
uint16_t mid_left_lines, mid_right_lines;

// Global windows
uint16_t max_h, max_w;

// <><><><><><><><><><><><><>
//      Linuxcnc Update Functions
//  TODO Move to own file
// <><><><><><><><><><><><><>
void updateDRO(WINDOW *dro){
  mvprintw(0,5  ,"Machine   Program   Offset");

  mvprintw(1,1,"X:  %f  %f  %f",0.123,0.123,0.123);
  mvprintw(2,1,"Y:  %f  %f  %f",0.123,0.123,0.123);
  mvprintw(3,1,"Z:  %f  %f  %f",0.123,0.123,0.123);

}



void updateEditor(WINDOW *editor, int line){

}

void updateEditor(WINDOW *editor){
  updateEditor(editor, 0);
}

// <><><><><><><><><><><><><>
//      Main Function
//  TODO Abstract all functions in here
// <><><><><><><><><><><><><>
int main() {
  // Initialization
  initscr();
  cbreak();
  noecho();

  getmaxyx(stdscr, max_h, max_w);
  if(max_w < 80 || max_h < 10){
    endwin();
    printf("Terminal must be larger than 80 characters and 10 lines");
    return -1;
  }

  mid_top_col = max_w/2;
  mid_bot_col = max_w/2;
  mid_right_lines = max_h/2;
  mid_left_lines = max_h/2;
  refresh();
  w_dro = newwin(mid_right_lines, mid_top_col, 0, 0);
  w_editor = newwin(max_h-mid_right_lines, mid_bot_col, mid_right_lines, 0);
  w_status = newwin(mid_left_lines, max_w - mid_top_col, 0, mid_top_col);
  w_messages = newwin(max_h - mid_left_lines, max_w - mid_bot_col, mid_left_lines, mid_bot_col);
  box(w_dro, 0 , 0);
  box(w_editor, 0 , 0);
  box(w_status, 0 , 0);
  box(w_messages, 0 , 0);
  keypad(w_dro, true);

  nodelay(w_dro, true);
  nodelay(w_editor, true);
  nodelay(w_status, true);
  nodelay(w_messages, true);

  waddstr(w_dro, "DRO");
  wrefresh(w_dro);
  waddstr(w_editor, "Editor");
  wrefresh(w_editor);
  waddstr(w_status, "Status");
  wrefresh(w_status);
  waddstr(w_messages, "Messages");
  wrefresh(w_messages);

  int ch;

// <><><><><><><><><><><><><>
//      Main Loop
// <><><><><><><><><><><><><>
 bool run = true;
  while(run){
    ch = getch();
    if(ch == 'q'){
      run = false;
    }
    else{
      updateDRO(w_dro);
    }
    wrefresh(w_dro);

  }
  // Clean up

  endwin();
  return 0;
}
