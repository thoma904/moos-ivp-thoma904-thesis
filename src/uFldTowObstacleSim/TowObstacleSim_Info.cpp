/*****************************************************************/
/*    NAME: Tom Monaghan                                         */
/*    ORGN: MIT, Cambridge MA                                    */
/*    FILE: TowObstacleSim_Info.cpp                              */
/*    DATE: March 2026                                           */
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
  blk("  half for the tow (TOW_GIVEN_OBSTACLE) and half for the       ");
  blk("  vehicle (GIVEN_OBSTACLE).                                     ");
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
  blk("  poly_vert_color  = color    (default is gray50)               ");
  blk("  poly_edge_color  = color    (default is gray50)               ");
  blk("  poly_fill_color  = color    (default is white)                ");
  blk("  poly_label_color = color    (default is invisible)            ");
  blk("                                                                ");
  blk("  tow_fill_color   = yellow   (default is yellow)               ");
  blk("  tow_edge_color   = orange   (default is orange)               ");
  blk("                                                                ");
  blk("  poly_vert_size    = 1       (default is 1)                    ");
  blk("  poly_edge_size    = 1       (default is 1)                    ");
  blk("  poly_transparency = 0.15    (default is 0.15)                 ");
  blk("                                                                ");
  blk("  draw_region       = true    (default is true)                 ");
  blk("  region_edge_color = color   (default is gray50)               ");
  blk("  region_vert_color = color   (default is white)                ");
  blk("                                                                ");
  blk("  post_points      = true     (default is false)                ");
  blk("  rate_points      = 5        (default is 5)                    ");
  blk("  point_size       = 5        (default is 2)                    ");
  blk("                                                                ");
  blk("  min_duration     = 10       (default is -1)                   ");
  blk("  max_duration     = 15       (default is -1)                   ");
  blk("  refresh_interval = 8        (default is -1)                   ");
  blk("                                                                ");
  blk("  reset_interval   = -1       (default is -1)                   ");
  blk("  reset_range      = 10       (default is 10)                   ");
  blk("                                                                ");
  blk("  reuse_ids        = true     (default is true)                 ");
  blk("  sensor_range     = 50       (default is 50)                   ");
  blk("                                                                ");
  blk("  post_visuals = true  // {true or false} By default true       ");
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
  blk("  PMV_CONNECT      = true                                       ");
  blk("  VEHICLE_CONNECT  = true                                       ");
  blk("  NODE_REPORT                                                   ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  VIEW_POLYGON                                                  ");
  blk("  KNOWN_OBSTACLE                                                ");
  blk("  TOW_GIVEN_OBSTACLE   (first half of obstacles - for tow)      ");
  blk("  GIVEN_OBSTACLE       (second half of obstacles - for vehicle) ");
  blk("  TOW_TRACKED_FEATURE  (tow obstacle sensor points)            ");
  blk("  TRACKED_FEATURE      (vehicle obstacle sensor points)        ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("uFldTowObstacleSim", "gpl");
  exit(0);
}
