/****************************************************************/
/*   NAME: Tom Monaghan                                         */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: Cable_Info.cpp                                       */
/*   DATE: March 2026                                           */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "Cable_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pCable models the cable shape between the towing vessel and   ");
  blk("  the tow body as a chain of spring-damped nodes. Complements   ");
  blk("  pTowing which computes the tow body endpoint. Dynamics match  ");
  blk("  the pTowing/AOF spring-drag-clamp model applied per segment.  ");
  blk("  Publishes VIEW_SEGLIST and CABLE_NODE_REPORT for obstacle     ");
  blk("  avoidance and visualization.                                  ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pCable file.moos [OPTIONS]                               ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pCable with the given process name                 ");
  blk("      rather than pCable.                                       ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pCable.                    ");
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
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pCable Example MOOS Configuration                               ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pCable                                          ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  cable_length   = 30    // meters                              ");
  blk("  attach_offset  = 0     // meters (stern offset from CG)       ");
  blk("  k_spring       = 5.0   // spring constant                     ");
  blk("  cd             = 0.7   // quadratic drag coefficient          ");
  blk("  c_tan          = 2.0   // tangential damping coefficient      ");
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
  blu("pCable INTERFACE                                                ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NAV_X       = 103.0                                           ");
  blk("  NAV_Y       = -23.8                                           ");
  blk("  NAV_HEADING = 180.0                                           ");
  blk("                                                                ");
  blk("  TOWED_X     = 120.5          (tow body position from pTowing) ");
  blk("  TOWED_Y     = -30.2                                           ");
  blk("                                                                ");
  blk("  TOW_CABLE_LENGTH     = 30    (runtime sync from pTowing)      ");
  blk("  TOW_ATTACH_OFFSET    = 5                                      ");
  blk("  TOW_SPRING_STIFFNESS = 5.0                                    ");
  blk("  TOW_DRAG_COEFF       = 0.7                                    ");
  blk("  TOW_TAN_DAMPING      = 2.0                                    ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  VIEW_SEGLIST      = pts={103,-23.8:112,-27:120.5,-30.2},      ");
  blk("                      label=CABLE,edge_color=white,edge_size=1  ");
  blk("  CABLE_NODE_REPORT = nodes=3,x0=103,y0=-23.8,x1=112,y1=-27,    ");
  blk("                      x2=120.5,y2=-30.2                         ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pCable", "gpl");
  exit(0);
}

