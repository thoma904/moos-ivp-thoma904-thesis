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
  m_towed_vx = m_towed_vy = 0; // Initialize towed body velocity
  m_prev_time = 0; // Initialize previous time
  m_deployed = false;
  m_cable_distance = 0; // Initialize cable distance for troubleshooting
  m_nav_speed = 0; // Initialize vessel speed
  m_nav_vx = 0; // Initialize vessel velocity in x direction
  m_nav_vy = 0; // Initialize vessel velocity in y direction
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
  if(m_prev_time == 0) {
    m_prev_time = now; // Initialize on first call
  }
  double dt = std::max(1e-3, now - m_prev_time); // Time since last iteration
  m_prev_time = now; // Update previous time

  // Decompose Towing vessel's speed into x and y components
  // Assuming heading is in degrees, convert to radians for calculations
  // Assuming MOOS heading: 0° = North, 90° = East; x is East, y is North.
  double hdg_rad = (90.0 - m_nav_heading) * M_PI / 180.0;
  m_nav_vx = m_nav_speed * cos(hdg_rad);
  m_nav_vy = m_nav_speed * sin(hdg_rad);


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
      m_towed_vx = m_nav_vx; //initialize with towing vessel's velocity
      m_towed_vy = m_nav_vy; //initialize with towing vessel's velocity
      Notify("TOW_DEPLOYED", "true");
    }
  }

  if(m_deployed) 
  {
    // -----------------------
    // Spring-Pull Tow Model
    // -----------------------
    double dx = m_nav_x - m_towed_x;
    double dy = m_nav_y - m_towed_y;
    double distance = hypot(dx, dy); // Current distance between vessel and towed body
    m_cable_distance = distance; // Store for troubleshooting

    if(distance > 0.01) //avoid division by zero
    {

      // Build a point L behind the boat along current heading
      double ax = m_nav_x, ay = m_nav_y;
      if (m_nav_speed > 0.1) // Filter out extreme low speed/stopped cases
      {  
        ax = m_nav_x - m_cable_length * cos(hdg_rad);
        ay = m_nav_y - m_cable_length * sin(hdg_rad);
      }

      double dx_dir = ax - m_towed_x;
      double dy_dir = ay - m_towed_y;
      double d_dir  = hypot(dx_dir, dy_dir);
      if (d_dir < 1e-6) 
        d_dir = 1.0;

      // Normalize the vector from towed body to vessel
      // to get unit vector in the direction of the cable
      double ux = dx_dir / d_dir;          // unit vector x
      double uy = dy_dir / d_dir;          // unit vector y

      // Calculate tangential unit vector (perpendicular to cable)
      double nx = -uy; // unit vector normal x
      double ny = ux; // unit vector normal y

      // --- Spring (only pull) ---
      if(distance > m_cable_length) 
      {
        double overshoot = distance - m_cable_length; //amount stretched beyond cable length
        double k = 5.0; // s^-2 (tuneable spring constant)
        m_towed_vx += k * overshoot * ux * dt;
        m_towed_vy += k * overshoot * uy * dt;
      }

      // --- Quadratic drag based on the tow‑fish speed ---
      // This simulates the drag force proportional to the square of the speed
      // Essentially uses towing body speed to calculate drag

      double speed = hypot(m_towed_vx, m_towed_vy);
      if(speed > 1e-6) 
      {
        double CdA_over_m = 0.7; // 1/m  (tunable lumped drag coefficient)
        m_towed_vx += -CdA_over_m * m_towed_vx * speed * dt;
        m_towed_vy += -CdA_over_m * m_towed_vy * speed * dt;
      }

      // --- Extra tangential damping (Returns towed body to centerline) ---
      double vt = m_towed_vx*nx + m_towed_vy*ny;    // sideways
      double c_tan = 2; // 1/s  (tuneable)
      m_towed_vx += (-c_tan * vt) * nx * dt;
      m_towed_vy += (-c_tan * vt) * ny * dt;

      // Integrate position
      m_towed_x += m_towed_vx * dt;
      m_towed_y += m_towed_vy * dt;

      // --- Rigid cable clamp ---
      // Forces the towed body to stay within the cable length
      double dist2 = hypot(m_nav_x - m_towed_x, m_nav_y - m_towed_y);
      if(dist2 > m_cable_length) 
      {
        double sx = m_nav_x - m_towed_x;
        double sy = m_nav_y - m_towed_y;
        double sc = m_cable_length / dist2;
        m_towed_x = m_nav_x - sx * sc;
        m_towed_y = m_nav_y - sy * sc;
        // Remove outward radial velocity so it doesn't re‑stretch immediately
        double urx = sx / dist2, ury = sy / dist2;
        double vrad = m_towed_vx*urx + m_towed_vy*ury;
        if(vrad < 0) 
        { // only if pointing outward
          m_towed_vx -= vrad * urx;
          m_towed_vy -= vrad * ury;
        }
      }
    }
  }

  // -----------------------
  // Publish heading
  // -----------------------
  double dx = m_nav_x - m_towed_x;
  double dy = m_nav_y - m_towed_y;
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

  // VIEW_POINT for MarineViewer
  string body_str = "x=" + doubleToStringX(m_towed_x,1) + ",y=" +
                    doubleToStringX(m_towed_y,1); //+ ",label=TOW_BODY";
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

  // -----------------------
  // Publish NODE_REPORT_LOCAL for the towed body (acts like a node)
  // -----------------------

  // Prefer heading from tow velocity; fallback to cable bearing if nearly stopped
  double tow_speed = hypot(m_towed_vx, m_towed_vy);
  double tow_hdg_vel = (tow_speed > 0.05) ? relAng(0, 0, m_towed_vx, m_towed_vy) : tow_heading;  // fall back to cable direction

  std::ostringstream tow_nr;
  tow_nr << "NAME="   << m_host_community << "_TOW"      // unique name
        << ",TYPE="  << "heron"      // pick any known type
        << ",TIME="  << std::fixed << m_curr_time        // AppCastingMOOSApp time
        << ",X="     << doubleToStringX(m_towed_x, 2)
        << ",Y="     << doubleToStringX(m_towed_y, 2)
        << ",SPD="   << doubleToStringX(tow_speed, 2)
        << ",HDG="   << doubleToStringX(angle360(tow_hdg_vel), 1)
        << ",LENGTH="<< 1.0                               // optional visual hint
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


//consider using a cable length behind the vessel as an anchor point to guide the towed body instead of the vessel's current position

//register for positions of obstacles and have the towed body avoid them
//make more realistic shape of towed body for obstacle avoidance
//are the obstacles on the fly or pre-known?
//how are they representing obstacles in the MOOS world?
//how to convert bathymetry to obstacles?
//bound the behavior to avoid straight line and breadcrumb paths