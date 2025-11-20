/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: TowTurnMgr.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "TowTurnMgr.h"
#include <AngleUtils.h>
#include <cmath>

using namespace std;

//---------------------------------------------------------
// Constructor()

TowTurnMgr::TowTurnMgr()
{
  m_nav_x = 0;
  m_nav_y = 0;
  m_nav_heading = 0;
  m_nav_speed = 0;
  m_towed_x = 0;
  m_towed_y = 0;
  m_prev_x = 0;
  m_prev_y = 0;
  m_next_x = 0;
  m_next_y = 0;
  m_tow_deployed = false;
  m_cable_length = 0;
  m_leg_length = 0;
  m_dist_end = 0;
  m_turn_factor = 1;
  m_wpt_index = 0;
  m_turn_active = false;
  m_turn_dir = "";
  m_posted_direction = false;
  m_next_turn_dir = -1; // Default to PORT
}

//---------------------------------------------------------
// Destructor

TowTurnMgr::~TowTurnMgr()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool TowTurnMgr::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "NAV_X") 
       m_nav_x = msg.GetDouble();
     
     else if(key == "NAV_Y") 
       m_nav_y = msg.GetDouble();
     
     else if(key == "NAV_HEADING") 
       m_nav_heading = msg.GetDouble();

      else if(key == "NAV_SPEED")
       m_nav_speed = msg.GetDouble();

     else if(key == "TOWED_X") 
       m_towed_x = msg.GetDouble();
     
     else if(key == "TOWED_Y") 
       m_towed_y = msg.GetDouble();

      else if(key == "PREV_WPT_X")
       m_prev_x = msg.GetDouble();

      else if(key == "PREV_WPT_Y")
       m_prev_y = msg.GetDouble();

      else if(key == "NEXT_WPT_X")
       m_next_x = msg.GetDouble();

      else if(key == "NEXT_WPT_Y")
       m_next_y = msg.GetDouble();

      else if(key == "TOW_DEPLOYED")
      {string val = msg.GetString();
        if(val == "true" || val == "TRUE")
          m_tow_deployed = true;
        else
          m_tow_deployed = false;
      }

      else if(key == "WPT_INDEX")
        m_wpt_index = msg.GetDouble();

      else if(key == "TURN_ACTIVE")
      {string val = msg.GetString();
        if(val == "false" || val == "FALSE")
          m_posted_direction = false;
      }

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool TowTurnMgr::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool TowTurnMgr::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(m_wpt_index/2 != int(m_wpt_index/2)) 
  {
    double dx = m_next_x - m_prev_x;
    double dy = m_next_y - m_prev_y;
    m_leg_length = hypot(dx, dy);

    double dist_to_end = hypot(m_next_x - m_nav_x, m_next_y - m_nav_y);
    m_dist_end = dist_to_end;

    double dist_tow_next = hypot(m_next_x - m_towed_x, m_next_y - m_towed_y);

    double capture_radius = 10;
    bool leg_complete = (dist_to_end < capture_radius);

    bool turn_active = (m_tow_deployed && leg_complete);
    m_turn_active = turn_active;

    double turn_dir_radius = capture_radius + 5;
    bool near_turn_point = (dist_to_end < turn_dir_radius);

    if(!m_posted_direction && near_turn_point)
    {

      int turn_dir = m_next_turn_dir;

      m_turn_dir   = (turn_dir > 0) ? "STARBOARD" : "PORT";

      Notify("TURN_DIRECTION", m_turn_dir);

      Notify("TOW_TURN_PARAMS", "turn_dir=" + intToString(turn_dir));
      m_posted_direction = true;

      m_next_turn_dir *= -1; // Alternate turn direction
    }

    if(m_turn_active)
      Notify("TURN_ACTIVE", "true");
  }
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool TowTurnMgr::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    
    /*if(param == "cable_length") 
    {
      m_cable_length = stod(value);
      handled = true;
    }*/

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void TowTurnMgr::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("TOWED_X", 0);
  Register("TOWED_Y", 0);
  Register("PREV_WPT_X", 0);
  Register("PREV_WPT_Y", 0);
  Register("NEXT_WPT_X", 0);
  Register("NEXT_WPT_Y", 0);
  Register("TOW_DEPLOYED", 0);
  Register("WPT_INDEX", 0);
  Register("TURN_ACTIVE", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool TowTurnMgr::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Leg Length | Dist to End | Turn Active | Turn Direction";
  actab.addHeaderLines();
  actab << doubleToString(m_leg_length) << doubleToString(m_dist_end) << boolToString(m_turn_active) << m_turn_dir;
  m_msgs << actab.getFormattedString();
  
  return(true);
}




