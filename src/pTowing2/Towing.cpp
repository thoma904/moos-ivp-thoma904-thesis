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
  m_cable_tension = 1000;
  m_cable_angle = 0; // Initialize cable angle

  m      =  85.0;     // dry mass of towed body  [kg]
  mT     =   10.0;     // entrained (cable‑end) mass [kg]
  xT     =   5.0;     

  ms33   = 120.0;     // added‑mass terms from Newman
  ms35   =   0.0;
  ms55   =  15.0;
  M35    =   0.0;
  M55    =   0.0;

  /* states propagated each Iterate() */
  U1        = 0.0;   // surge of towing vehicle (input)  [m/s]
  U3        = 0.0;   // sway/heave component of towed body [m/s]
  Omega2    = 0.0;   // pitch‑rate (body‑axis 2)          [rad/s]

  tow_heading = 0.0; // deg, world frame yaw (uses Ω2)
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

static inline double deg2rad(double d){return d*M_PI/180.0;}
static inline double rad2deg(double r){return r*180.0/M_PI;}

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

  /*if(m_towing_position.size() == 0) {
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
  
  else 
  {
    double dx = m_nav_x - m_towed_x;
    double dy = m_nav_y - m_towed_y;
    double separation = hypot(dx, dy);

    if(separation <= m_cable_length)          // slack → no force
        m_cable_tension = 0.0;
    else {
        const double k_spring = 500.0;        // N / m (tune)
        m_cable_tension = k_spring * (separation - m_cable_length);
    }

    U1 = m_nav_speed;                               // surge of towing vehicle (input)  [m/s]
    const double F3 = m_cable_tension * sin(m_cable_angle);  // rudder‑like force

    //------------------------------------------------------------------
    // 2.  Right‑hand sides (Newman eq. 60‑61, your notation)
    //------------------------------------------------------------------
    const double R1 = -F3
                      + U1 * mT * U3
                      - U1 * (xT * mT + m) * Omega2;

    const double R2 =  F3 * xT
                      + U1 * (ms33 + xT * mT) * U3
                      - U1 * (ms35 + M35 - xT*xT*mT) * Omega2;

    //------------------------------------------------------------------
    // 3.  Solve diagonal mass matrix  M · (‑V̇) = R   →  V̇
    //------------------------------------------------------------------
    const double denom_U3   =  ms33 + m;
    const double denom_O2   =  ms55 + M55;

    const double Udot3      = (-R1) / denom_U3;
    const double Omegadot2  = (-R2) / denom_O2;

    //------------------------------------------------------------------
    // 4.  Integrate states (Euler‑1st‑order; dt is already small)
    //------------------------------------------------------------------
    U3       += Udot3     * dt;
    Omega2   += Omegadot2 * dt;

    //------------------------------------------------------------------
    // 5.  Kinematics to world frame (flat‑plane model)
    //------------------------------------------------------------------
    double hdg_rad = deg2rad(tow_heading);
    double world_u =  U1 * cos(hdg_rad) - U3 * sin(hdg_rad); // ẋ world
    double world_v =  U1 * sin(hdg_rad) + U3 * cos(hdg_rad); // ẏ world

    m_towed_x += world_u * dt;
    m_towed_y += world_v * dt;

    tow_heading += rad2deg(Omega2 * dt);
    if(tow_heading <   0) tow_heading += 360.0;
    if(tow_heading >= 360) tow_heading -= 360.0;
  }

    //------------------------------------------------------------------
    // 6.  Visual / MOOS postings (unchanged from your stub)
    //------------------------------------------------------------------
    string tb_pos_str = "x=" + doubleToStringX(m_towed_x,1) + ",y=" +
                        doubleToStringX(m_towed_y,1);
    Notify("TOWING_POSITION", tb_pos_str);

    string tb_hdg_str = "heading=" + doubleToStringX(tow_heading,1);
    Notify("TOWING_HEADING", tb_hdg_str);

    string body_str = "x=" + doubleToStringX(m_towed_x,1) + ",y=" +
                      doubleToStringX(m_towed_y,1) + ",label=TOW_BODY";
    body_str += ",type=diamond,color=red";
    body_str += ",heading=" + doubleToStringX(tow_heading,1);
    Notify("VIEW_POINT", body_str);

    string cable_str = "pts={" + doubleToStringX(m_nav_x,1) + "," +
                                doubleToStringX(m_nav_y,1) + ":" +
                                doubleToStringX(m_towed_x,1) + "," +
                                doubleToStringX(m_towed_y,1) + "}";
    cable_str += ",label=TOW_LINE";
    cable_str += ",edge_color=gray,edge_size=2,vertex_size=0";
    Notify("VIEW_SEGLIST", cable_str);

    AppCastingMOOSApp::PostReport();
    return true;*/
    //------------------------------------------------------------------
    // 1.  Deploy cable – keep body parked until hawser is taut
    //------------------------------------------------------------------
    double dx0 = m_nav_x - m_start_x;
    double dy0 = m_nav_y - m_start_y;
    double dist_from_start = hypot(dx0, dy0);

    if(m_towing_position.size() == 0) // first iterate
    {     
        m_start_x = m_nav_x;
        m_start_y = m_nav_y;
        m_towed_x = m_nav_x;
        m_towed_y = m_nav_y;
    }

    if(dist_from_start < m_cable_length) {
        m_towed_x = m_start_x;
        m_towed_y = m_start_y;
    }

    //------------------------------------------------------------------
    // 2A.  True cable direction and body‑frame angle β
    //------------------------------------------------------------------
    // World‑frame vector *from body to tug*
    double dx = m_nav_x - m_towed_x;
    double dy = m_nav_y - m_towed_y;
    double separation = hypot(dx, dy);
    if(separation < 1e-6) separation = 1e-6;      // avoid divide‑by‑zero

    // Rotate into body frame:  p_body = Rz(‑ψ_body) · p_world
    double psi = deg2rad(tow_heading);
    double cosP = cos(psi);
    double sinP = sin(psi);

    double cable_x_body =  cosP*dx + sinP*dy;     // forward is +x
    double cable_y_body = -sinP*dx + cosP*dy;     // +y is starboard

    double beta = atan2(cable_y_body, cable_x_body);     // (‑π, π)
    m_cable_angle = beta;                                 // log for plots

    //------------------------------------------------------------------
    // 2B.  Spring–dashpot tension (always ≥ 0)
    //------------------------------------------------------------------
    double rel_ext  = separation - m_cable_length;        // stretch [m]
    const double k_spring = 200.0;                        // N m‑1    (tune)
    const double c_damp   = 50.0;                         // N s m‑1  (tune)

    // Approximate d(separation)/dt
    static double prev_sep = separation;
    double stretch_rate = (separation - prev_sep) / dt;
    prev_sep = separation;

    // Damping only while the line is *extending*
    double tension = 0.0;
    if(rel_ext > 0.0) {
        double visc = (stretch_rate > 0.0) ? c_damp*stretch_rate : 0.0;
        tension = k_spring*rel_ext + visc;
    }
    const double T_MAX = 2.0e4;                            // 20 kN
    if(tension > T_MAX) tension = T_MAX;
    m_cable_tension = tension;

    //------------------------------------------------------------------
    // 3.  Hydrodynamics – add linear sway drag
    //------------------------------------------------------------------
    U1 = m_nav_speed;
    double F3 =  tension * sin(beta);

    // Simple linear water damping in sway:  D₃ = –C_y U₃
    const double Cy = 100.0;                               // kg s‑1  (tune)
    double D3 = -Cy * U3;

    double R1 = -(F3 + D3) + U1 * mT * U3
                - U1 * (xT * mT + m) * Omega2;

    double R2 =  F3 * xT + U1 * (ms33 + xT * mT) * U3
                - U1 * (ms35 + M35 - xT*xT*mT) * Omega2;

    double denom_U3 = ms33 + m;
    double denom_O2 = ms55 + M55;

    double Udot3     = (-R1) / denom_U3;
    double Omegadot2 = (-R2) / denom_O2;

    // Symplectic (semi‑implicit) Euler – velocity first, then position
    U3     += Udot3     * dt;
    Omega2 += Omegadot2 * dt;

    //------------------------------------------------------------------
    // 4.  Advance position and heading with *new* velocities
    //------------------------------------------------------------------
    double tug_hdg_rad = deg2rad(m_nav_heading);           // tug’s ψ
    double world_u =  U1 * cos(tug_hdg_rad)
                    - U3 * sin(psi);                       // body sway into world
    double world_v =  U1 * sin(tug_hdg_rad)
                    + U3 * cos(psi);

    m_towed_x += world_u * dt;
    m_towed_y += world_v * dt;

    tow_heading += rad2deg(Omega2 * dt);
    if(tow_heading < 0)   tow_heading += 360.0;
    if(tow_heading >= 360) tow_heading -= 360.0;

    //------------------------------------------------------------------
    // 5.  History housekeeping (optional)
    //------------------------------------------------------------------
    m_towing_position.add_vertex(m_nav_x, m_nav_y);
    if(m_towing_position.size() > 500)
        m_towing_position.delete_vertex(0);

    //------------------------------------------------------------------
    // 6.  Publish visuals and report
    //------------------------------------------------------------------
    publish:
    string tb_pos_str = "x=" + doubleToStringX(m_towed_x,1) + ",y=" +
                        doubleToStringX(m_towed_y,1);
    Notify("TOWING_POSITION", tb_pos_str);

    string tb_hdg_str = "heading=" + doubleToStringX(tow_heading,1);
    Notify("TOWING_HEADING", tb_hdg_str);

    string body_str = "x=" + doubleToStringX(m_towed_x,1) + ",y=" +
                      doubleToStringX(m_towed_y,1) + ",label=TOW_BODY";
    body_str += ",type=diamond,color=red";
    body_str += ",heading=" + doubleToStringX(tow_heading,1);
    Notify("VIEW_POINT", body_str);

    string cable_str = "pts={" + doubleToStringX(m_nav_x,1) + "," +
                                doubleToStringX(m_nav_y,1) + ":" +
                                doubleToStringX(m_towed_x,1) + "," +
                                doubleToStringX(m_towed_y,1) + "}";
    cable_str += ",label=TOW_LINE";
    cable_str += ",edge_color=gray,edge_size=2,vertex_size=0";
    Notify("VIEW_SEGLIST", cable_str);

    AppCastingMOOSApp::PostReport();
    return true;
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


