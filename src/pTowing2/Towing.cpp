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
#include <Eigen/Dense>

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

  //ODE's
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


