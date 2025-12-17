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
  m_prev_heading = 0; 
  m_towed_vx = m_towed_vy = 0; // towed body velocity
  m_prev_time = 0; 
  m_deployed = false;
  m_cable_distance = 0;
  m_nav_speed = 0; 
  m_nav_vx = 0; // vessel velocity in x direction
  m_nav_vy = 0; // vessel velocity in y direction
  m_attach_offset = 0;   // tow hook offset from NAV_ reference [m]
  m_anchor_x = 0;        // world x of tow hook
  m_anchor_y = 0;        // world y of tow hook
  m_spring_stiffness = 5.0; // spring stiffness constant 1/s^2
  m_cd = 0.7;               // lumped drag coefficient 1/m
  m_tan_damping = 2.0; // tangential damping constant 1/s
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
  AppCastingMOOSApp::Iterate();

  //Establish time increment
  double now = MOOSTime();
  if(m_prev_time == 0) 
  {
    m_prev_time = now; // Initialize on first call
  }
  double dt = std::max(1e-3, now - m_prev_time); // Time since last iteration
  m_prev_time = now; // Update previous time

  // Decompose Towing vessel's speed into x and y components
  // MOOS heading: 0° = North, 90° = East; x is East, y is North.
  double hdg_rad = (90.0 - m_nav_heading) * M_PI / 180.0; // Convert to radians for calculations
  m_nav_vx = m_nav_speed * cos(hdg_rad);
  m_nav_vy = m_nav_speed * sin(hdg_rad);

  // Compute the position of the tow hook
  m_anchor_x = m_nav_x - m_attach_offset * cos(hdg_rad);
  m_anchor_y = m_nav_y - m_attach_offset * sin(hdg_rad);

  // On first position update, store starting point
  if(m_towing_position.size() == 0) 
  {
    m_start_x = m_nav_x;
    m_start_y = m_nav_y;
    //towed body starts at vessel position
    m_towed_x = m_nav_x;
    m_towed_y = m_nav_y;
  }

  // Store vessel position
  m_towing_position.add_vertex(m_nav_x, m_nav_y);

  // Trim history to prevent memory bloat
  if(m_towing_position.size() > 500)
    m_towing_position.delete_vertex(0);

  // Approximate paid-out distance: vessel displacement from start minus attach_offset
  // (Best when vessel travels roughly straight; turning makes this approximation less exact.)
  double dx0 = m_nav_x - m_start_x;
  double dy0 = m_nav_y - m_start_y;
  double dist_from_start = hypot(dx0, dy0) - m_attach_offset;

  if(!m_deployed)
  {
    if(dist_from_start < m_cable_length) 
    {
      // Towed body stays at starting location until cable is fully deployed
      m_towed_x = m_start_x;
      m_towed_y = m_start_y;
      Notify("TOW_DEPLOYED", "false");
      m_cable_distance = dist_from_start; // Store for troubleshooting
    }
    else 
    {
      m_deployed = true; // Cable fully deployed, switch to tow model
      // Initialize towed body velocity to vessel velocity
      m_towed_vx = m_nav_vx; 
      m_towed_vy = m_nav_vy; 
      Notify("TOW_DEPLOYED", "true");
    }
  }

  if(m_deployed) 
  {
    // -----------------------
    // Spring-Pull Tow Model
    // Tow dynamics (drag + damping + optional tension) + rigid cable constraint projection
    // -----------------------
    double dx = m_anchor_x - m_towed_x;
    double dy = m_anchor_y - m_towed_y;
    double distance = hypot(dx, dy); // Current distance between anchor point and towed body
    m_cable_distance = distance; // pre-clamp distance (useful for detecting overshoot), stored for troubleshooting

    if(distance > 0.01) //avoid division by zero
    {
      double dx_dir = m_anchor_x - m_towed_x;
      double dy_dir = m_anchor_y - m_towed_y;
      double d_dir  = hypot(dx_dir, dy_dir);
      if (d_dir < 1e-6)
        d_dir = 1.0;

      // unit vector along cable
      double ux = dx_dir / d_dir;
      double uy = dy_dir / d_dir;
      // tangential unit vector
      double nx = -uy;
      double ny = ux;

      // Soft tension term: adds an inward acceleration if cable is overstretched.
      // Note: even with the rigid clamp below, this still affects velocity (dynamics).
      // Disable this if you want a purely rigid/inextensible cable behavior.
      if(distance > m_cable_length) 
      {
        double overshoot = distance - m_cable_length; //amount stretched beyond cable length
        double k = m_spring_stiffness; // s^-2 (tuneable spring constant, higher = stiffer spring)
        m_towed_vx += k * overshoot * ux * dt;
        m_towed_vy += k * overshoot * uy * dt;
      }

      // --- Quadratic drag based on the tow speed ---
      // This simulates the drag force proportional to the square of the speed
      // Uses the towed body's speed to apply quadratic drag.

      double speed = hypot(m_towed_vx, m_towed_vy);
      if(speed > 1e-6) 
      {
        double CdA_over_m = m_cd; // 1/m  (tunable lumped drag coefficient)
        m_towed_vx += -CdA_over_m * m_towed_vx * speed * dt;
        m_towed_vy += -CdA_over_m * m_towed_vy * speed * dt;
      }

      // Extra tangential damping: damps sideways motion perpendicular to the cable direction (reduces swinging).
      double vt = m_towed_vx*nx + m_towed_vy*ny;    // sideways
      double c_tan = m_tan_damping; // 1/s  (tuneable)
      m_towed_vx += (-c_tan * vt) * nx * dt;
      m_towed_vy += (-c_tan * vt) * ny * dt;

      // Integrate position
      m_towed_x += m_towed_vx * dt;
      m_towed_y += m_towed_vy * dt;

      // --- Rigid cable clamp ---
      // Forces the towed body to stay within the cable length

      // vector from towed body to anchor
      double sx = m_anchor_x - m_towed_x;
      double sy = m_anchor_y - m_towed_y;

      //actual distance between towed body and anchor
      double dist_a = hypot(sx, sy);

      if(dist_a > m_cable_length) 
      {

       //ratio to scale back the vector, <1 means towed body is too far
       double sc = m_cable_length / dist_a;

       // slide the towed body back to the cable length
       m_towed_x = m_anchor_x - sx * sc;
       m_towed_y = m_anchor_y - sy * sc;

       // unit vector from towed body to anchor
       double urx = sx / dist_a;
       double ury = sy / dist_a;

       // Radial velocity along unit vector (tow -> anchor). Positive = toward anchor, negative = outward.
       // Note: This uses tow velocity in world frame; a more physical model would use (v_towed - v_anchor).
       double vrad = m_towed_vx * urx + m_towed_vy * ury;
      
       // If vrad < 0, tow is moving outward (away from anchor), which would increase cable length.
       if(vrad < 0) 
       {
        // Remove the outward radial component so velocity is tangential and/or inward only.
        m_towed_vx -= vrad * urx;
        m_towed_vy -= vrad * ury;
       }
      }
    }
  }

  // -----------------------
  // Publish heading
  // -----------------------
  double dx = m_anchor_x - m_towed_x;
  double dy = m_anchor_y - m_towed_y;
  double tow_heading = relAng(0, 0, dx, dy);  // degrees

  // -----------------------
  // Publish position and visuals
  // -----------------------
  string tb_pos_str = "x=" + doubleToStringX(m_towed_x,1) + ",y=" +
                      doubleToStringX(m_towed_y,1);
  Notify("TOWING_POSITION", tb_pos_str);
  Notify("TOWED_X", m_towed_x);
  Notify("TOWED_Y", m_towed_y);

  string tb_hdg_str = "heading=" + doubleToStringX(tow_heading,1);
  Notify("TOWING_HEADING", tb_hdg_str);
  Notify("TOWED_HEADING", tow_heading);

  /* This was replaced by NODE_REPORT_LOCAL, can be re-enabled if desired for troubleshooting.
  // VIEW_POINT for MarineViewer
  string body_str = "x=" + doubleToStringX(m_towed_x,1) + ",y=" +
                    doubleToStringX(m_towed_y,1) + ",label=TOW_BODY";
  body_str += ",type=diamond,color=red";
  body_str += ",heading=" + doubleToStringX(tow_heading,1);
  Notify("VIEW_POINT", body_str);
  */

  // VIEW_SEGLIST for cable
  string cable_str = "pts={" + doubleToStringX(m_anchor_x,1) + "," +
                              doubleToStringX(m_anchor_y,1) + ":" +
                              doubleToStringX(m_towed_x,1) + "," +
                              doubleToStringX(m_towed_y,1) + "}";
  cable_str += ",label=TOW_LINE";
  cable_str += ",edge_color=gray,edge_size=2,vertex_size=0";
  Notify("VIEW_SEGLIST", cable_str);

  // -----------------------
  // Publish NODE_REPORT_LOCAL for the towed body (acts like a vessel)
  // -----------------------

  // Prefer heading from tow velocity; fallback to cable bearing if nearly stopped
  double tow_speed = hypot(m_towed_vx, m_towed_vy);
  Notify("TOWED_SPEED", tow_speed);
  double tow_hdg_vel = (tow_speed > 0.05) ? relAng(0, 0, m_towed_vx, m_towed_vy) : tow_heading;  // fall back to cable direction

  std::ostringstream tow_nr;
  tow_nr << "NAME="   << m_host_community << "_TOW"      // unique name
        << ",TYPE="  << "heron"      // pick any known type
        << ",TIME="  << std::fixed << m_curr_time        // AppCastingMOOSApp time
        << ",X="     << doubleToStringX(m_towed_x, 2)
        << ",Y="     << doubleToStringX(m_towed_y, 2)
        << ",SPD="   << doubleToStringX(tow_speed, 2)
        << ",HDG="   << doubleToStringX(angle360(tow_hdg_vel), 1)
        << ",LENGTH="<< 1.0                               //adjusts size of towed body icon
        << ",MODE="  << "TOWING"
        << ",COLOR=" << "orange";                        // helps distinguish in viewer

  Notify("NODE_REPORT_LOCAL", tow_nr.str());

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

    else if(param == "attach_offset") 
    {
      m_attach_offset = stod(value);
      handled = true;
    }

    else if(param == "spring_stiffness")
    {
      m_spring_stiffness = stod(value);
      handled = true;
    }

    else if(param == "drag_coefficient")
    {
      m_cd = stod(value);
      handled = true;
    }

    else if(param == "tangential_damping")
    {
      m_tan_damping = stod(value);
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
  m_msgs << " CABLE_DISTANCE: " << m_cable_distance << endl;
  m_msgs << " Deployed: " << (m_deployed ? "true" : "false") << endl;
  m_msgs << " TOW_VX: " << m_towed_vx << endl;
  m_msgs << " TOW_VY: " << m_towed_vy << endl;
  m_msgs << " ATTACH_OFFSET: " << m_attach_offset << endl;

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