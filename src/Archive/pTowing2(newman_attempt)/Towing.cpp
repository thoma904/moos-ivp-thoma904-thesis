/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Towing.cpp                                      */
/*    DATE: December 29th, 1963                             */
/*    Simple Mass Spring Damper Model                       */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "Towing.h"
#include <AngleUtils.h>
//#include <Eigen/Dense>

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
  m_prev_heading = 0;
  m_nav_speed = 0;
  m_prev_time = MOOSTime();
  m_cable_tension = 0;
  m_cable_angle = 0; // Initialize cable angle
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

     else if(key == "NAV_SPEED")
       m_nav_speed = msg.GetDouble();

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
  double now = MOOSTime();
  double dt  = now - m_prev_time;
  m_prev_time = now;
  if(dt <= 0)    // safety
    return true;
  
  //Goal:
  // Predict the XY position of the towed body when influenced by towing vehicle
  //Description:
  // Equations of motion from Newman's Marine Hydrodynamics (equations 60 and 61 from Chapter 7)
  // initial assumptions:
  // Symmetric Cylinder for Towed Body
  // Towed Body is completely submerged
  // Cable angle acts as rudder force
  // U_3 and Omega_2 are initially 0
  //ODE's
  // F_3 = Tension*sin(cable_angle)
  // F_3 = U_1 * m_t * U_3 - U_1 * (x_T * m_T + m) * Omega_2 + (m^s_33 + m)*Udot_3 + (m^s_35 +M_35)*Omegadot_2
  // -F_3*x_T = -U_1*(m^s_33 + x_T*m_T)*U_3 -U_1*(m^s_35 + M_35 - x_T*x_T*m_T)*Omega_2 + (m^s_35 + M_35)*Udot_3 + (m^s_55 + M_55)*Omegadot_2;
  // Right Hand Side 1:
  // R1 = -F_3 + U_1 * m_T * U_3 - U_1 * (x_T * m_T + m) * Omega_2
  // Right Hand Side 2:
  // R2 = F_3 * x_T + U_1 * (m^s_33 + x_T * m_T) * U_3 - U_1 * [m^s_35 + M_35 - (x_T)^2 * m_T] * Omega_2
  // Left Hand Side 1:
  // L1 = -(m^s_33 + m) * Udot_3 - (m^s_35 + M_35) * Omegadot_2
  // Left Hand Side 2:
  // L2 = -(m^s_35 + M_35) * Udot_3 - (m^s_55 + M_55) * Omegadot_2
  // M Matrix:
  // M = [m^s_33 + m, m^s_35 + M_35; m^s_35 + M_35, m^s_55 + M_55]
  // Vdot Matrix:
  // Vdot = [-Udot_3; -Omegadot_2]
  // R Matrix:
  // R = [R1; R2]
  // Simplified M Matrix with Newman U3 and Omega2 initially 0 assumption:
  // M = [m^s_33 + m, 0; 0, m^s_55 + M_55]
// On first position update, store starting point
  if(m_towing_position.size() == 0) {
    m_start_x = m_nav_x;
    m_start_y = m_nav_y;
    m_towed_x = m_nav_x;  // initialize towed body here too
    m_towed_y = m_nav_y;
  }

  // 1. Store vessel position
  m_towing_position.add_vertex(m_nav_x, m_nav_y);

  // 2. Trim history to prevent memory bloat
  if(m_towing_position.size() > 500)
    m_towing_position.delete_vertex(0);

  // 3. Compute distance from start
  double dx0 = m_nav_x - m_start_x;
  double dy0 = m_nav_y - m_start_y;
  double dist_from_start = hypot(dx0, dy0);

  if(dist_from_start < m_cable_length) {
    // Towed body stays at starting location until cable is fully deployed
    m_towed_x = m_start_x;
    m_towed_y = m_start_y;
  }
  else {

  }

  // -----------------------
  // Publish position and visuals
  // -----------------------
  string tb_pos_str = "x=" + doubleToStringX(m_towed_x,1) + ",y=" +
                      doubleToStringX(m_towed_y,1);
  Notify("TOWING_POSITION", tb_pos_str);

  string tb_hdg_str = "heading=" + doubleToStringX(tow_heading,1);
  Notify("TOWING_HEADING", tb_hdg_str);

  // VIEW_POINT for MarineViewer
  string body_str = "x=" + doubleToStringX(m_towed_x,1) + ",y=" +
                    doubleToStringX(m_towed_y,1) + ",label=TOW_BODY";
  body_str += ",type=diamond,color=red";
  body_str += ",heading=" + doubleToStringX(tow_heading,1);
  Notify("VIEW_POINT", body_str);

  // VIEW_SEGLIST for cable
  string cable_str = "pts={" + doubleToStringX(m_nav_x,1) + "," +
                              doubleToStringX(m_nav_y,1) + ":" +
                              doubleToStringX(m_towed_x,1) + "," +
                              doubleToStringX(m_towed_y,1) + "}";
  cable_str += ",label=TOW_LINE";
  cable_str += ",edge_color=gray,edge_size=2,vertex_size=0";
  Notify("VIEW_SEGLIST", cable_str);

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
  Register("NAV_SPEED", 0);
  Register("CABLE_TENSION", 0);
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


std::string Towing::join(const vector<string> &vec, const string &delim) {
  string result;
  for(size_t i = 0; i < vec.size(); ++i) {
    if(i != 0) result += delim;
    result += vec[i];
  }
  return result;
}


