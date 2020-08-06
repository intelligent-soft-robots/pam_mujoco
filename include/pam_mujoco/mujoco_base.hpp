/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/


// This file is a copy of the original mujoco code
// of the file /sample/simulate.cpp, for easier
// include in projects

#pragma once

#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"

#include <thread>
#include <mutex>
#include <chrono>


namespace mujoco
{

  //-------------------------------- global -----------------------------------------------

  // constants
  const int maxgeom = 5000;           // preallocated geom array in mjvScene
  const double syncmisalign = 0.1;    // maximum time mis-alignment before re-sync
  const double refreshfactor = 0.7;   // fraction of refresh available for simulation


  // model and data
  extern mjModel* m;
  extern mjData* d;
  extern char filename[1000];


  // abstract visualization
  extern mjvScene scn;
  extern mjvCamera cam;
  extern mjvOption vopt;
  extern mjvPerturb pert;
  extern mjvFigure figconstraint;
  extern mjvFigure figcost;
  extern mjvFigure figtimer;
  extern mjvFigure figsize;
  extern mjvFigure figsensor;


  // OpenGL rendering and UI
  extern GLFWvidmode vmode;
  extern int windowpos[2];
  extern int windowsize[2];
  extern mjrContext con;
  extern GLFWwindow* window;
  extern mjuiState uistate;
  extern mjUI ui0, ui1;


  // UI settings not contained in MuJoCo structures
  struct
  {
    // file
    int exitrequest = 0;

    // option
    int spacing = 0;
    int color = 0;
    int font = 0;
    int ui0 = 1;
    int ui1 = 1;
    int help = 0;               
    int info = 0;               
    int profiler = 0;           
    int sensor = 0;
    int fullscreen = 0;
    int vsync = 1;
    int busywait = 0;

    // simulation
    int run = 1;
    int key = 0;
    int loadrequest = 0;

    // watch
    char field[mjMAXUITEXT] = "qpos";
    int index = 0;

    // physics: need sync
    int disable[mjNDISABLE];
    int enable[mjNENABLE];

    // rendering: need sync
    int camera = 0;
  } settings;


  // section ids
  enum
    {
      // left ui
      SECT_FILE   = 0,
      SECT_OPTION,
      SECT_SIMULATION,
      SECT_WATCH,
      SECT_PHYSICS,
      SECT_RENDERING,
      SECT_GROUP,
      NSECT0,

      // right ui
      SECT_JOINT = 0,
      SECT_CONTROL,
      NSECT1
    };


  // file section of UI
  const mjuiDef defFile[] = 
    {
      {mjITEM_SECTION,   "File",          1, NULL,                    "AF"},
      {mjITEM_BUTTON,    "Save xml",      2, NULL,                    ""},
      {mjITEM_BUTTON,    "Save mjb",      2, NULL,                    ""},
      {mjITEM_BUTTON,    "Print model",   2, NULL,                    "CM"},
      {mjITEM_BUTTON,    "Print data",    2, NULL,                    "CD"},
      {mjITEM_BUTTON,    "Quit",          1, NULL,                    "CQ"},
      {mjITEM_END}
    };


  // option section of UI
  const mjuiDef defOption[] = 
    {
      {mjITEM_SECTION,   "Option",        1, NULL,                    "AO"},
      {mjITEM_SELECT,    "Spacing",       1, &settings.spacing,       "Tight\nWide"},
      {mjITEM_SELECT,    "Color",         1, &settings.color,         "Default\nOrange\nWhite\nBlack"},
      {mjITEM_SELECT,    "Font",          1, &settings.font,          "50 %\n100 %\n150 %\n200 %\n250 %\n300 %"},
      {mjITEM_CHECKINT,  "Left UI (Tab)", 1, &settings.ui0,           " #258"},
      {mjITEM_CHECKINT,  "Right UI",      1, &settings.ui1,           "S#258"},
      {mjITEM_CHECKINT,  "Help",          2, &settings.help,          " #290"},
      {mjITEM_CHECKINT,  "Info",          2, &settings.info,          " #291"},
      {mjITEM_CHECKINT,  "Profiler",      2, &settings.profiler,      " #292"},
      {mjITEM_CHECKINT,  "Sensor",        2, &settings.sensor,        " #293"},
#ifdef __APPLE__
      {mjITEM_CHECKINT,  "Fullscreen",    0, &settings.fullscreen,    " #294"},
#else
      {mjITEM_CHECKINT,  "Fullscreen",    1, &settings.fullscreen,    " #294"},
#endif
      {mjITEM_CHECKINT,  "Vertical Sync", 1, &settings.vsync,         " #295"},
      {mjITEM_CHECKINT,  "Busy Wait",     1, &settings.busywait,      " #296"},
      {mjITEM_END}
    };


  // simulation section of UI
  const mjuiDef defSimulation[] = 
    {
      {mjITEM_SECTION,   "Simulation",    1, NULL,                    "AS"},
      {mjITEM_RADIO,     "",              2, &settings.run,           "Pause\nRun"},
      {mjITEM_BUTTON,    "Reset",         2, NULL,                    " #259"},
      {mjITEM_BUTTON,    "Reload",        2, NULL,                    "CL"},
      {mjITEM_BUTTON,    "Align",         2, NULL,                    "CA"},
      {mjITEM_BUTTON,    "Copy pose",     2, NULL,                    "CC"},
      {mjITEM_SLIDERINT, "Key",           3, &settings.key,           "0 0"},
      {mjITEM_BUTTON,    "Reset to key",  3},
      {mjITEM_BUTTON,    "Set key",       3},
      {mjITEM_END}
    };


  // watch section of UI
  const mjuiDef defWatch[] = 
    {
      {mjITEM_SECTION,   "Watch",         0, NULL,                    "AW"},
      {mjITEM_EDITTXT,   "Field",         2, settings.field,          "qpos"},
      {mjITEM_EDITINT,   "Index",         2, &settings.index,         "1"},
      {mjITEM_STATIC,    "Value",         2, NULL,                    " "},
      {mjITEM_END}
    };


  // help strings
  const char help_content[] = 
    "Alt mouse button\n"
    "UI right hold\n"
    "UI title double-click\n"
    "Space\n"
    "Esc\n"
    "Right arrow\n"
    "Left arrow\n"
    "Down arrow\n"
    "Up arrow\n"
    "Page Up\n"
    "Double-click\n"
    "Right double-click\n"
    "Ctrl Right double-click\n"
    "Scroll, middle drag\n"
    "Left drag\n"
    "[Shift] right drag\n"
    "Ctrl [Shift] drag\n"
    "Ctrl [Shift] right drag";

  const char help_title[] = 
    "Swap left-right\n"
    "Show UI shortcuts\n"
    "Expand/collapse all  \n"
    "Pause\n"
    "Free camera\n"
    "Step forward\n"
    "Step back\n"
    "Step forward 100\n"
    "Step back 100\n"
    "Select parent\n"
    "Select\n"
    "Center\n"
    "Track camera\n"
    "Zoom\n"
    "View rotate\n"
    "View translate\n"
    "Object rotate\n"
    "Object translate";


  // info strings
  extern char info_title[1000];
  extern char info_content[1000];



  //----------------------- profiler, sensor, info, watch ---------------------------------

  // init profiler figures
  void profilerinit(void);

  // update profiler figures
  void profilerupdate(void);
  
  // show profiler figures
  void profilershow(mjrRect rect);
  
  // init sensor figure
  void sensorinit(void);

  // update sensor figure
  void sensorupdate(void);

  // show sensor figure
  void sensorshow(mjrRect rect);

  // prepare info text
  void infotext(char* title, char* content, double interval);

  // sprintf forwarding, to avoid compiler warning in x-macro
  void printfield(char* str, void* ptr);

  // update watch
  void watch(void);



  //-------------------------------- UI construction --------------------------------------

  // make physics section of UI
  void makephysics(int oldstate);

  // make rendering section of UI
  void makerendering(int oldstate);

  // make group section of UI
  void makegroup(int oldstate);

  // make joint section of UI
  void makejoint(int oldstate);

  // make control section of UI
  void makecontrol(int oldstate);

  // make model-dependent UI sections
  void makesections(void);


  //-------------------------------- utility functions ------------------------------------

  // align and scale view
  void alignscale(void);

  // copy qpos to clipboard as key
  void copykey(void);

  // millisecond timer, for MuJoCo built-in profiler
  mjtNum timer(void);

  // clear all times
  void cleartimers(void);

  // update UI 0 when MuJoCo structures change (except for joint sliders)
  void updatesettings(void);

  // drop file callback
  void drop(GLFWwindow* window, int count, const char** paths);

  // load mjb or xml model
  void loadmodel(void);

  //--------------------------------- UI hooks (for uitools.c) ----------------------------

  // determine enable/disable item state given category
  int uiPredicate(int category, void* userdata);

  // set window layout
  void uiLayout(mjuiState* state);

  // handle UI event
  void uiEvent(mjuiState* state);

  //--------------------------- rendering and simulation ----------------------------------

  // sim thread synchronization
  extern std::mutex mtx;


  // prepare to render
  void prepare(void);

  // render im main thread (while simulating in background thread)
  void render(GLFWwindow* window);

  // simulate in background thread (while rendering in main thread)
  void simulate(void);

  //-------------------------------- init and main ----------------------------------------

  // initalize everything
  void init(void);

}





  
