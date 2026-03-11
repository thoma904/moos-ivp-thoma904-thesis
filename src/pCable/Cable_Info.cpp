/****************************************************************/
/*   NAME: Tom Monaghan                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: Cable_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
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
  blk("  pTowing1 which computes the tow body endpoint. Dynamics match ");
  blk("  the pTowing1/AOF spring-drag-clamp model applied per segment.");
  blk("  Publishes VIEW_SEGLIST and CABLE_NODE_REPORT for obstacle     ");
  blk("  avoidance and visualization.                                  ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pCable file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pCable with the given process name         ");
  blk("      rather than pCable.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pCable.        ");
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
  blu("pCable Example MOOS Configuration                   ");
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
  blu("pCable INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NAV_X            = double (vessel x position)                 ");
  blk("  NAV_Y            = double (vessel y position)                 ");
  blk("  NAV_HEADING      = double (vessel heading, degrees)           ");
  blk("  TOWED_X          = double (tow body x, from pTowing1)        ");
  blk("  TOWED_Y          = double (tow body y, from pTowing1)        ");
  blk("  TOW_CABLE_LENGTH = double (dynamic cable length sync)         ");
  blk("  TOW_ATTACH_OFFSET    = double (dynamic attach offset sync)    ");
  blk("  TOW_SPRING_STIFFNESS = double (dynamic spring constant sync)  ");
  blk("  TOW_DRAG_COEFF       = double (dynamic drag coeff sync)       ");
  blk("  TOW_TAN_DAMPING      = double (dynamic tangential damp sync)  ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  VIEW_SEGLIST      = cable shape for pMarineViewer             ");
  blk("  CABLE_NODE_REPORT = nodes=N,x0=..,y0=..,x1=..,y1=..,...      ");
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

