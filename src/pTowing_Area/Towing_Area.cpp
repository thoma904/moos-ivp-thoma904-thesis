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

// ---- Visualize the wedge ("delta") between ship and tow headings ----
// Project both rays from the TOW position, same length
const double lookahead_s = 10.0;
double extent_m = std::max(m_nav_speed * lookahead_s, 8.0);  // min size so it's visible

// MOOS heading: 0=N, 90=E  → radians (x-east, y-north)
const double ship_rad = (M_PI/180.0) * (90.0 - m_nav_heading);
const double tow_rad  = (M_PI/180.0) * (90.0 - m_towed_heading);

// Points at equal range along each heading, *anchored at the tow*
double ship_pt_x = m_towed_x + extent_m * std::cos(ship_rad);
double ship_pt_y = m_towed_y + extent_m * std::sin(ship_rad);
double tow_pt_x  = m_towed_x + extent_m * std::cos(tow_rad);
double tow_pt_y  = m_towed_y + extent_m * std::sin(tow_rad);

// Signed smallest angle (ship minus tow) in [-180,180]
double delta_deg = m_nav_heading - m_towed_heading;
delta_deg = std::fmod(delta_deg + 540.0, 360.0) - 180.0;


if(m_tow_deployed)
{
// If almost aligned, don't draw (optional threshold)
if(std::fabs(delta_deg) < 1.0) 
{
  // (skip drawing the wedge if the delta is tiny)
} 
else {
  // Color by side: starboard (ship clockwise of tow) = red, port = cyan
  std::string color = (delta_deg >= 0.0) ? "red" : "cyan";

  // Build the triangle: tow → ship-ray point → tow-ray point
  std::string spec = "pts={";
  spec += doubleToStringX(m_towed_x,1) + "," + doubleToStringX(m_towed_y,1) + ":";
  spec += doubleToStringX(ship_pt_x,1) + "," + doubleToStringX(ship_pt_y,1) + ":";
  spec += doubleToStringX(tow_pt_x,1)  + "," + doubleToStringX(tow_pt_y,1)  + "}";
  spec += ",label=TOW_DELTA";
  spec += ",edge_size=1,vertex_size=0";
  spec += ",fill_transparency=0.25";
  spec += ",edge_color=" + color;
  spec += ",fill_color=" + color;

  if(m_tow_deployed)
    Notify("VIEW_POLYGON", spec);

  // (Optional) draw the two rays so the edges of the wedge are obvious
  // Ship heading ray
  std::string ray_ship = "pts={";
  ray_ship += doubleToStringX(m_towed_x,1) + "," + doubleToStringX(m_towed_y,1) + ":";
  ray_ship += doubleToStringX(ship_pt_x,1) + "," + doubleToStringX(ship_pt_y,1) + "}";
  ray_ship += ",label=DELTA_RAY_SHIP,edge_color=white,edge_size=2,vertex_size=0";
  Notify("VIEW_SEGLIST", ray_ship);

  // Tow heading ray
  std::string ray_tow = "pts={";
  ray_tow += doubleToStringX(m_towed_x,1) + "," + doubleToStringX(m_towed_y,1) + ":";
  ray_tow += doubleToStringX(tow_pt_x,1)  + "," + doubleToStringX(tow_pt_y,1)  + "}";
  ray_tow += ",label=DELTA_RAY_TOW,edge_color=white,edge_size=1,vertex_size=0";
  Notify("VIEW_SEGLIST", ray_tow);

  // --- Uncertainty circle around the tow (as a simple N-gon) ---
{
  const double r_base   = 3.0;    // base radius (m)
  const double r_gain   = 1.5;    // radius grows with drift (m per m/s)
  const double r_min    = 2.0;
  const double r_max    = 10.0;
  // Reuse tow_speed_est if you added snippet (A); otherwise just set to 0
  double r = std::clamp(r_base + r_gain * tow_speed_est, r_min, r_max);

  const int N = 20; // resolution of the “circle”
  std::string circ = "pts={";
  for(int i=0;i<N;i++){
    double a = 2.0*M_PI * (double)i / (double)N;
    double x = m_towed_x + r * std::cos(a);
    double y = m_towed_y + r * std::sin(a);
    circ += doubleToStringX(x,1) + "," + doubleToStringX(y,1);
    circ += (i < N-1) ? ":" : "}";
  }
  circ += ",label=TOW_UNCERT,edge_color=yellow,fill_color=yellow,fill_transparency=0.25";
  circ += ",edge_size=1,vertex_size=0";
  Notify("VIEW_POLYGON", circ);
}

}
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




