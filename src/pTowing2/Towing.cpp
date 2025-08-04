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
  m_cable_tension = 100; //Newtons
  m_cable_angle = m_nav_heading; // initial angle same as heading
  m_rho     = 1025.0;

  m_m_t   = 10.0;     // strip added mass   [kg]
  m_x_T   = -2.0;     // lever arm          [m]
  m_m     = 50.0;     // dry mass           [kg]
  m_M_35  = 0.0;
  m_M_55  = 100.0;

  m_ms_33 = 20.0;
  m_ms_35 = 0.0;
  m_ms_55 = 50.0;

  m_u3 = m_omega2 = m_psi = 0.0;
  m_b_pos.setZero();
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

     else if(key == "CABLE_TENSION")
       m_cable_tension = msg.GetDouble();

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

   //--------------------------- 1.  Cable unit vector & angle
  Eigen::Vector2d tow_pos(m_nav_x, m_nav_y);          // tug position
  Eigen::Vector2d r_vec = tow_pos - m_b_pos;          // body → tug
  double r_norm = r_vec.norm();
  Eigen::Vector2d r_hat =
    (r_norm > 1e-6) ? r_vec.normalized()
                    : Eigen::Vector2d(1, 0);

  // body‑fixed components of r_hat
  double hdg_body = MOOSDeg2Rad(m_nav_heading) + m_psi;   // world yaw
  double cosH = cos(hdg_body);
  double sinH = sin(hdg_body);

  double x_b =  cosH * r_hat.x() + sinH * r_hat.y();      // surge
  double y_b = -sinH * r_hat.x() + cosH * r_hat.y();      // sway (starboard+)

  m_cable_angle = atan2(y_b, x_b);                        // diag only

  //--------------------------- 2.  Lateral cable force  F3
  const double F3 = m_cable_tension * y_b;                // [N]

  //--------------------------- 3.  Assemble RHS (R1, R2)
  const double U1 = m_nav_speed;      // surge speed [m/s]

  const double R1 = -F3
                    + U1 * m_m_t * m_u3
                    - U1 * (m_x_T * m_m_t + m_m) * m_omega2;

  const double R2 =  F3 * m_x_T
                    - U1 * (m_ms_33 + m_x_T * m_m_t) * m_u3
                    - U1 * (m_ms_35 + m_M_35 - m_x_T * m_x_T * m_m_t)
                          * m_omega2;

  //--------------------------- 4.  Solve  M · vdot = R
  Eigen::Matrix2d M;
  M << m_ms_33 + m_m,          m_ms_35 + m_M_35,
       m_ms_35 + m_M_35,       m_ms_55 + m_M_55;

  Eigen::Vector2d vdot = M.ldlt().solve(Eigen::Vector2d(R1, R2));
  double Udot3     = -vdot(0);
  double Omegadot2 = -vdot(1);

  //--------------------------- 5.  Integrate body states
  m_u3     += Udot3     * dt;
  m_omega2 += Omegadot2 * dt;
  m_psi    += m_omega2 * dt;

  //--------------------------- 6.  World‑frame velocity & position
  double Vx =  U1 * cosH - m_u3 * sinH;
  double Vy =  U1 * sinH + m_u3 * cosH;

  m_b_pos.x() += Vx * dt;
  m_b_pos.y() += Vy * dt;

  //--------------------------- 7.  MOOS outputs
  Notify("TOWED_X",        m_b_pos.x());
  Notify("TOWED_Y",        m_b_pos.y());
  Notify("TOWED_HEADING",  MOOSRad2Deg(hdg_body));
  Notify("TOWED_U3",       m_u3);
  Notify("TOWED_R",        m_omega2);
  Notify("CABLE_ANGLE",    MOOSRad2Deg(m_cable_angle));
  Notify("F3_LATERAL",     F3);

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


