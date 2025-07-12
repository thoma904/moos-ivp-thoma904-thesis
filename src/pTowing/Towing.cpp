/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Towing.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "Towing.h"
#include <AngleUtils.h>

using namespace std;

//---------------------------------------------------------
// Constructor()

Towing::Towing()
{
  m_nav_x = 0;
  m_nav_y = 0;
  m_nav_heading = 0;
  m_towed_x = 0;
  m_towed_y = 0;
  m_towing_position.clear();
  m_start_x = 0;
  m_start_y = 0;
  m_cable_length = 10; // default value
}

//---------------------------------------------------------
// Destructor

Towing::~Towing()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool Towing::OnNewMail(MOOSMSG_LIST &NewMail)
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

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool Towing::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Towing::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(m_towing_position.size() == 1) //set starting position to act as deployment point
  {
    m_start_x = m_nav_x;
    m_start_y = m_nav_y;
  } 

  // Store towing vessel position
  m_towing_position.add_vertex(m_nav_x, m_nav_y);

  // Trim list to keep it manageable, arbitrarily set to 500 points
  if(m_towing_position.size() > 500)
    m_towing_position.delete_vertex(0);

  // Compute distance from starting point
  double dx0 = m_nav_x - m_start_x;
  double dy0 = m_nav_y - m_start_y;
  double dist_from_start = hypot(dx0, dy0);

  // define local variables
  double dist = 0;
  double towed_x = m_nav_x;
  double towed_y = m_nav_y;

  // find the point that is m_cable_length away from the start point 
  for(int i = m_towing_position.size()-1; i > 0; i--) 
  {
    double x1 = m_towing_position.get_vx(i);
    double y1 = m_towing_position.get_vy(i);
    double x0 = m_towing_position.get_vx(i-1);
    double y0 = m_towing_position.get_vy(i-1);

    double seg_len = hypot(x1 - x0, y1 - y0);
    dist += seg_len;

    if(dist >= m_cable_length) // alittle clunky but it works
    {
      towed_x = x0;
      towed_y = y0;
      break;
    }
  }

  if (dist_from_start < m_cable_length) // keeps towed body at start point until cable is fully deployed
  {
    towed_x = m_start_x;
    towed_y = m_start_y;
  }

  else // the towed body is moving so update its position
  {
    m_towed_x = towed_x;
    m_towed_y = towed_y;
  }

  // Publish towing position
  string tb_pos_str = "x=" + doubleToStringX(m_towed_x,1) + ",y=" +
                    doubleToStringX(m_towed_y,1);
  Notify("TOWING_POSITION", tb_pos_str);

  string tb_hdg_str = "heading=" + doubleToStringX(m_nav_heading,1);
  Notify("TOWING_HEADING", tb_hdg_str);

  // Publish towed body for visualization
  string body_str = "x=" + doubleToStringX(m_towed_x,1) + ",y=" +
                    doubleToStringX(m_towed_y,1) + ",label=TOW_BODY";
  body_str += ",type=diamond,color=red"; //look into resizing and if there are actually shapes
  body_str += ",heading=" + doubleToStringX(m_nav_heading,1);
  Notify("VIEW_POINT", body_str);


  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Towing::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  m_cable_length = 10; // default value

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
    if(param == "cable_length") 
    {
      m_cable_length = stod(value);
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

void Towing::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool Towing::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "  Towing Simulation Status                  " << endl;
  m_msgs << "============================================" << endl;

  m_msgs << " NAV_X: " << m_nav_x << endl;
  m_msgs << " NAV_Y: " << m_nav_y << endl;
  m_msgs << " HEADING: " << m_nav_heading << endl;
  m_msgs << " TOWED_X: " << m_towed_x << endl;
  m_msgs << " TOWED_Y: " << m_towed_y << endl;
  m_msgs << " CABLE_LENGTH: " << m_cable_length << endl;

  return(true);
}




