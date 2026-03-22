/****************************************************************/
/*   NAME: Tom Monaghan                                         */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: Towing_Info.cpp                                      */
/*   DATE: February 2026                                        */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "Towing_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pTowing application simulates a towed body attached to    ");
  blk("  the vessel by a cable of configurable length. It uses a       ");
  blk("  spring-drag-clamp model: soft spring tension when the cable   ");
  blk("  is overstretched, quadratic drag, tangential damping, and a   ");
  blk("  rigid cable clamp. Publishes towed body position, velocity,   ");
  blk("  heading, and a NODE_REPORT_LOCAL so the towed body appears    ");
  blk("  in pMarineViewer.                                             ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pTowing file.moos [OPTIONS]                              ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pTowing with the given process name                ");
  blk("      rather than pTowing.                                      ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pTowing.                   ");
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
  blu("pTowing Example MOOS Configuration                              ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pTowing                                         ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  cable_length        = 30    // (meters) default is 10         ");
  blk("  attach_offset       = 5     // (meters) tow hook offset from  ");
  blk("                              // NAV reference, default is 0    ");
  blk("  spring_stiffness    = 5.0   // (1/s^2)  default is 5.0        ");
  blk("  drag_coefficient    = 0.7   // (1/m)    default is 0.7        ");
  blk("  tangential_damping  = 2.0   // (1/s)    default is 2.0        ");
  blk("  post_cable          = false // default is true                ");
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
  blu("pTowing INTERFACE                                               ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NAV_X       = 103.0                                           ");
  blk("  NAV_Y       = -23.8                                           ");
  blk("  NAV_HEADING = 180.0                                           ");
  blk("  NAV_SPEED   = 1.5                                             ");
  blk("                                                                ");
  blk("  TOW_CABLE_LENGTH     = 30   (runtime override of cable_length)");
  blk("  TOW_ATTACH_OFFSET    = 0    (runtime override of offset)      ");
  blk("  TOW_SPRING_STIFFNESS = 5.0  (runtime override of stiffness)   ");
  blk("  TOW_DRAG_COEFF       = 0.7  (runtime override of drag)        ");
  blk("  TOW_TAN_DAMPING      = 2.0  (runtime override of damping)     ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  TOWED_X        = 120.5                                        ");
  blk("  TOWED_Y        = -30.2                                        ");
  blk("  TOWED_VX       = 0.8                                          ");
  blk("  TOWED_VY       = -0.3                                         ");
  blk("  TOWED_SPEED    = 0.85                                         ");
  blk("  TOWED_HEADING  = 200.5                                        ");
  blk("  TOW_DEPLOYED   = true                                         ");
  blk("                                                                ");
  blk("  TOWING_POSITION = x=120.5,y=-30.2                             ");
  blk("  TOWING_HEADING  = heading=200.5                                ");
  blk("                                                                ");
  blk("  TOW_CABLE_LENGTH     = 30                                     ");
  blk("  TOW_ATTACH_OFFSET    = 0                                      ");
  blk("  TOW_SPRING_STIFFNESS = 5.0                                    ");
  blk("  TOW_DRAG_COEFF       = 0.7                                    ");
  blk("  TOW_TAN_DAMPING      = 2.0                                    ");
  blk("                                                                ");
  blk("  VIEW_SEGLIST       = pts={103,-23.8:120.5,-30.2},             ");
  blk("                       label=TOW_LINE,edge_color=gray           ");
  blk("  NODE_REPORT_LOCAL  = NAME=alpha_TOW,TYPE=heron,X=120.5,       ");
  blk("                       Y=-30.2,SPD=0.85,HDG=200.5,COLOR=orange ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pTowing", "gpl");
  exit(0);
}
