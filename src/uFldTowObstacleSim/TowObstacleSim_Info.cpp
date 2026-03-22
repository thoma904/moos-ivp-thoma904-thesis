/*****************************************************************/
/*    NAME: Tom Monaghan                                         */
/*    ORGN: MIT, Cambridge MA                                    */
/*    FILE: TowObstacleSim_Info.cpp                              */
/*    DATE: March 2026                                           */
/*   This file is based on the original uFldObstacleSim.         */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "TowObstacleSim_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The uFldTowObstacleSim app simulates posting of obstacles     ");
  blk("  loaded from a text file, splitting them into two groups:      ");
  blk("  the first half for the tow (TOW_GIVEN_OBSTACLE) and the      ");
  blk("  second half for the vehicle (GIVEN_OBSTACLE). Based on        ");
  blk("  uFldObstacleSim.                                              ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: uFldTowObstacleSim file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch uFldTowObstacleSim with the given process name     ");
  blk("      rather than uFldTowObstacleSim.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of uFldTowObstacleSim.        ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blu("=============================================================== ");
  blu("uFldTowObstacleSim Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = uFldTowObstacleSim                              ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  obstacle_file    = obstacles.txt                              ");
  blk("                                                                ");
  blk("  poly_vert_color  = gray50     // default is gray50            ");
  blk("  poly_edge_color  = gray50     // default is gray50            ");
  blk("  poly_fill_color  = white      // default is white             ");
  blk("  poly_label_color = invisible  // default is invisible         ");
  blk("                                                                ");
  blk("  tow_fill_color   = yellow     // default is yellow            ");
  blk("  tow_edge_color   = orange     // default is orange            ");
  blk("                                                                ");
  blk("  poly_vert_size    = 1         // default is 1                 ");
  blk("  poly_edge_size    = 1         // default is 1                 ");
  blk("  poly_transparency = 0.15      // default is 0.15              ");
  blk("                                                                ");
  blk("  draw_region       = true      // default is true              ");
  blk("  region_edge_color = gray50    // default is gray50            ");
  blk("  region_vert_color = white     // default is white             ");
  blk("                                                                ");
  blk("  post_points      = false      // default is false             ");
  blk("  rate_points      = 5          // default is 5                 ");
  blk("  point_size       = 2          // default is 2                 ");
  blk("                                                                ");
  blk("  min_duration     = -1         // default is -1 (off)          ");
  blk("  max_duration     = -1         // default is -1 (off)          ");
  blk("  refresh_interval = -1         // default is -1 (off)          ");
  blk("                                                                ");
  blk("  reset_interval   = -1         // default is -1 (off)          ");
  blk("  reset_range      = 10         // default is 10                ");
  blk("                                                                ");
  blk("  reuse_ids        = true       // default is true              ");
  blk("  sensor_range     = 50         // default is 50                ");
  blk("                                                                ");
  blk("  post_visuals     = true       // default is true              ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("uFldTowObstacleSim INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  PMV_CONNECT     = true                                        ");
  blk("  OBM_CONNECT     = true                                        ");
  blk("  UFOS_RESET      = true                                        ");
  blk("  UFOS_POINT_SIZE = 5                                           ");
  blk("  NODE_REPORT     = NAME=alpha,X=103,Y=-23.8,SPD=1.5,HDG=180   ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  VIEW_POLYGON       = pts={20.6,-29.6:22.4,-31.4:...},        ");
  blk("                       label=ob_0,edge_color=orange             ");
  blk("  VIEW_POINT         = x=-30.0,y=-49.7,label=alpha:ob_4:3,     ");
  blk("                       vertex_color=yellow,vertex_size=2        ");
  blk("                                                                ");
  blk("  KNOWN_OBSTACLE     = pts={...},label=ob_0                     ");
  blk("  KNOWN_OBSTACLE_CLEAR = all                                    ");
  blk("                                                                ");
  blk("  TOW_GIVEN_OBSTACLE = pts={20.6,-29.6:22.4,-31.4:...},        ");
  blk("                       label=ob_0                               ");
  blk("  GIVEN_OBSTACLE     = pts={32,-11.9:33.8,-13.7:...},          ");
  blk("                       label=ob_3                               ");
  blk("                                                                ");
  blk("  TOW_TRACKED_FEATURE_ALPHA = x=-30.0,y=-49.7,key=ob_0         ");
  blk("  TRACKED_FEATURE_ALPHA     = x=-25.1,y=-40.3,key=ob_3         ");
  blk("                                                                ");
  blk("  PLOGGER_CMD    = COPY_FILE_REQUEST = obstacles.txt            ");
  blk("  UFOS_MIN_RNG   = 10                                          ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("uFldTowObstacleSim", "gpl");
  exit(0);
}
