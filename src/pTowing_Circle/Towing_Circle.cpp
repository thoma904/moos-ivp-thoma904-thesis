/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Towing_Circle.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "Towing_Circle.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

Towing_Circle::Towing_Circle()
{
  m_nav_x = 0;
  m_nav_y = 0;
  m_nav_heading = 0;
  m_nav_speed = 0;
  m_towed_x = 0;
  m_towed_y = 0;
  m_towed_heading = 0;
  m_prev_time = 0;
  m_radius = 0;
  m_radius_initial = 1;
}

//---------------------------------------------------------
// Destructor

Towing_Circle::~Towing_Circle()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool Towing_Circle::OnNewMail(MOOSMSG_LIST &NewMail)
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
     else if(key == "TOWED_HEADING") 
        m_towed_heading = msg.GetDouble();

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool Towing_Circle::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Towing_Circle::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  double now = MOOSTime();
  double dt = 0;
  
  if (m_prev_time == 0)
    m_prev_time = now;

  dt = std::max(1e-6, now - m_prev_time);
  m_prev_time = now;

  if (m_radius == 0)
    m_radius = m_radius_initial;

  if (m_nav_speed > 0.1 && dt > 1e-6)
    m_radius += dt/20;

  string towing_circle = "format=radial, label=foxtrot, x=" + doubleToString(m_towed_x) + ", y=" + doubleToString(m_towed_y) + ", radius=" + doubleToString(m_radius) + ", pts=12, snap=1";

  Notify("VIEW_POLYGON", towing_circle);

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Towing_Circle::OnStartUp()
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

void Towing_Circle::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("TOWED_X", 0);
  Register("TOWED_Y", 0);
  Register("TOWED_HEADING", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool Towing_Circle::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




