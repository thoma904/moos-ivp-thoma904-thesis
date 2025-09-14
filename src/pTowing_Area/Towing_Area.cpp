/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Towing_Area.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "XYPolygon.h"
#include "Towing_Area.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

Towing_Area::Towing_Area()
{
  m_nav_x = 0;
  m_nav_y = 0;
  m_nav_heading = 0;
  m_nav_speed = 0;
  m_tow_deployed = false;
  m_towed_x = 0;
  m_towed_y = 0;
  m_towed_heading = 0;
}

//---------------------------------------------------------
// Destructor

Towing_Area::~Towing_Area()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool Towing_Area::OnNewMail(MOOSMSG_LIST &NewMail)
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

     else if(key == "TOW_DEPLOYED")
     { 
       bool   mstr  = msg.IsString();
        if(mstr)
          m_tow_deployed = (msg.GetString() == "true");
        else
          m_tow_deployed = (msg.GetDouble() != 0);
     } 

     else if(key == "TOWED_X") 
       m_towed_x = msg.GetDouble();

     else if(key == "TOWED_Y") 
       m_towed_y = msg.GetDouble();

     else if(key == "TOWED_HEADING") 
       m_towed_heading = msg.GetDouble();

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool Towing_Area::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Towing_Area::Iterate()
{
  AppCastingMOOSApp::Iterate();

  double heading_diff = m_nav_heading - m_towed_heading;
  double distance_diff = hypot((m_nav_x - m_towed_x), (m_nav_y - m_towed_y));

  //from towed x and towed y to future positions should be avoidance zone

  const double lookahead_s = 1;
  const double hdg_rad = (M_PI/180.0) * (90.0 - m_nav_heading); // MOOS→radians
  const double dist_ahead = m_nav_speed * lookahead_s;

  double future_pos_x = m_nav_x + dist_ahead * std::cos(hdg_rad);
  double future_pos_y = m_nav_y + dist_ahead * std::sin(hdg_rad);

  // --- Build a minimal "avoidance zone" triangle ---
  // Line from tow -> future pos, with a small width at the tow end.
  double dx = future_pos_x - m_towed_x;
  double dy = future_pos_y - m_towed_y;
  double L  = hypot(dx, dy);

  // Avoid degenerate triangle if very short
  if(L < 1e-6) {
    dx = std::cos(hdg_rad);
    dy = std::sin(hdg_rad);
    L  = 1.0;
    future_pos_x = m_towed_x + dx;
    future_pos_y = m_towed_y + dy;
  }

  // Perpendicular unit vector to give the triangle some width near the tow
  double nx = -dy / L;
  double ny =  dx / L;

  const double half_width_m = 3.0; // visual half-width at the tow (adjust to taste)

  // Triangle vertices: tow-left, tow-right, future apex
  double p1x = m_towed_x + nx * half_width_m;
  double p1y = m_towed_y + ny * half_width_m;
  double p2x = m_towed_x - nx * half_width_m;
  double p2y = m_towed_y - ny * half_width_m;
  double p3x = future_pos_x;
  double p3y = future_pos_y;


  if(m_tow_deployed)
  {
  // --- Publish as a VIEW_POLYGON spec string (no XYPolygon needed) ---
  std::string spec = "pts={";
  spec += doubleToStringX(p1x,1) + "," + doubleToStringX(p1y,1) + ":";
  spec += doubleToStringX(p2x,1) + "," + doubleToStringX(p2y,1) + ":";
  spec += doubleToStringX(p3x,1) + "," + doubleToStringX(p3y,1) + "}";
  spec += ",label=TOW_AVOID";
  spec += ",edge_color=red,fill_color=red,fill_transparency=0.25";
  spec += ",edge_size=1,vertex_size=0";

  Notify("VIEW_POLYGON", spec);

  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Towing_Area::OnStartUp()
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
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void Towing_Area::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("TOW_DEPLOYED", 0);
  Register("TOWED_X", 0);
  Register("TOWED_Y", 0);
  Register("TOWED_HEADING", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool Towing_Area::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Tow Deployed | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << m_tow_deployed << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




