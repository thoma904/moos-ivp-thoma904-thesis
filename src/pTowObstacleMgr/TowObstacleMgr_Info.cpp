/****************************************************************/
/*   NAME: Tom Monaghan                                         */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: TowObstacleMgr_Info.cpp                              */
/*   DATE: March 2026                                           */
/*   This file is based on the original pObstacleMgr.           */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "TowObstacleMgr_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pTowObstacleMgr manages incoming sensor data about        ");
  blk("  obstacles and posts alerts for spawning obstacle avoidance     ");
  blk("  behaviors. It extends pObstacleMgr with tow awareness:        ");
  blk("  distances are computed from the vessel, towed body, and       ");
  blk("  cable to each obstacle. Designed for use with                 ");
  blk("  BHV_TowObstacleAvoid.                                         ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pTowObstacleMgr file.moos [OPTIONS]                      ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pTowObstacleMgr with the given process name        ");
  blk("      rather than pTowObstacleMgr.                              ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pTowObstacleMgr.           ");
  mag("  --web,-w                                                      ");
  blk("      Open browser to: https://oceanai.mit.edu/apps/pObstacleMgr");
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
  blu("pTowObstacleMgr Example MOOS Configuration                      ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pTowObstacleMgr                                 ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  point_var = TRACKED_FEATURE  // default is TRACKED_FEATURE    ");
  blk("                                                                ");
  blk("  given_obstacle = pts={90.2,-80.4:...:85.4,-80.4},label=ob_23  ");
  blk("  given_obs_var  = TOW_GIVEN_OBSTACLE  // default GIVEN_OBSTACLE");
  blk("                                                                ");
  blk("  post_dist_to_polys = close  // true, false, or (close)        ");
  blk("  post_view_polys = true      // (true) or false                ");
  blk("  obstacles_color = green     // default is blue                ");
  blk("                                                                ");
  blk("  max_pts_per_cluster = 20    // default is 20                  ");
  blk("  max_age_per_point   = 20    // (secs)  default is 20          ");
  blk("                                                                ");
  blk("  alert_range  = 20           // (meters) default is 20         ");
  blk("  ignore_range = -1           // (meters) default is -1, (off)  ");
  blk("                                                                ");
  blk("  lasso = true                // default is false               ");
  blk("  lasso_points = 6            // default is 6                   ");
  blk("  lasso_radius = 5            // (meters) default is 5          ");
  blk("                                                                ");
  blk("  given_max_duration = 30     // default is 60 seconds          ");
  blk("                                                                ");
  blk("  general_alert = update_var=GEN_ALERT, alert_range=800         ");
  blk("                                                                ");
  blk("  poly_label_thresh  = 25     // label color=off if amt>25      ");
  blk("  poly_shade_thresh  = 100    // shade color=off if amt>100     ");
  blk("  poly_vertex_thresh = 150    // vertex size=0 if amt>150       ");
  blk("                                                                ");
  blk("  disable_var = XYZ_DISABLE_TARGET  // default is empty str     ");
  blk("  enable_var  = XYZ_ENABLE_TARGET   // default is empty str     ");
  blk("  expunge_var = XYZ_EXPUNGE_TARGET  // default is empty str     ");
  blk("                                                                ");
  blk("  // Tow-specific parameters                                    ");
  blk("  use_tow          = true     // default is false               ");
  blk("  tow_only         = true     // default is true                ");
  blk("  use_tow_cable    = true     // default is true                ");
  blk("  cable_sample_step = 1.0     // (meters) default is 1.0        ");
  blk("  tow_pad          = 0.0      // (meters) default is 0.0        ");
  blk("  repost_interval  = 1.0      // (secs) default is 1.0          ");
  blk("  abaft_beam_thresh = off     // degrees or off, default off    ");
  blk("  post_view_point  = true     // default is true                ");
  blk("                                                                ");
  blk("  app_logging = true  // {true or file} By default disabled     ");
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
  blu("pTowObstacleMgr INTERFACE                                       ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  TRACKED_FEATURE   = x=5,y=8,label=a,size=4,color=1            ");
  blk("  GIVEN_OBSTACLE    = pts={90.2,-80.4:...:85.4,-80.4},label=ob_23");
  blk("  TOW_GIVEN_OBSTACLE = pts={...},label=ob_0                     ");
  blk("                                                                ");
  blk("  NAV_X = 103.0                                                 ");
  blk("  NAV_Y = -23.8                                                 ");
  blk("  NAV_HEADING = 180.0                                           ");
  blk("                                                                ");
  blk("  TOWED_X = 120.5                                               ");
  blk("  TOWED_Y = -30.2                                               ");
  blk("  TOWED_VX = 0.8                                                ");
  blk("  TOWED_VY = -0.3                                               ");
  blk("  TOW_DEPLOYED = true                                           ");
  blk("                                                                ");
  blk("  CABLE_NODE_REPORT = nodes=3,x0=103,y0=-23.8,x1=112,y1=-27,    ");
  blk("                      x2=120.5,y2=-30.2                         ");
  blk("                                                                ");
  blk("  OBM_ALERT_REQUEST = name=towobsavoid, alert_range=25,         ");
  blk("                      update_var=TOW_OBSTACLE_ALERT             ");
  blk("                                                                ");
  blk("  XYZ_DISABLE_TARGET = obstacle=3498                            ");
  blk("  XYZ_DISABLE_TARGET = 3498                                     ");
  blk("  XYZ_DISABLE_TARGET = vsource=radar                            ");
  blk("                                                                ");
  blk("  XYZ_ENABLE_TARGET  = obstacle=3498                            ");
  blk("  XYZ_ENABLE_TARGET  = 3498                                     ");
  blk("  XYZ_ENABLE_TARGET  = vsource=radar                            ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  VIEW_POLYGON      = pts={32,-100:38,-98:40,-100:32,-104},     ");
  blk("                      label=d,edge_color=white,vertex_color=blue");
  blk("  VIEW_POINT        = x=120.5,y=-30.2,label=towmgr_tow,         ");
  blk("                      vertex_color=yellow,vertex_size=4         ");
  blk("  OBM_CONNECT       = true                                      ");
  blk("  TOWMGR_TOW_ONLY   = true                                      ");
  blk("                                                                ");
  blk("  OBM_DIST_TO_OBJ   = ob_key,17.5                               ");
  blk("  OBM_DIST_NAV      = 17.5                                      ");
  blk("  OBM_DIST_TOW      = 12.3                                      ");
  blk("  OBM_DIST_CABLE    = 14.1                                      ");
  blk("  OBM_DIST_SYS      = 12.3                                      ");
  blk("  OBM_MIN_DIST_EVER = ob_key,12.3                               ");
  blk("                                                                ");
  blk("  TOW_OBSTACLE_ALERT = name=d#                                  ");
  blk("                      poly=pts={32,-100:38,-98:40,-100:32,-104},");
  blk("                      label=d                                   ");
  blk("  OBM_RESOLVED      = ob_23                                     ");
  blk("                                                                ");
  blk("  BHV_ABLE_FILTER   = obstacle=3498, action=disable             ");
  blk("  BHV_ABLE_FILTER   = obstacle=3498, action=enable              ");
  blk("  BHV_ABLE_FILTER   = vsource=radar, action=disable             ");
  blk("  BHV_ABLE_FILTER   = vsource=radar, action=enable              ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pTowObstacleMgr", "gpl");
  exit(0);
}
