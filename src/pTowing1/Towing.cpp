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

  //Establish time increment
  double now = MOOSTime();
  if(m_prev_time == 0) {
    m_prev_time = now; // Initialize on first call
  }
  double dt = std::max(1e-3, now - m_prev_time); // Time since last iteration
  m_prev_time = now; // Update previous time

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

    if(distance > 0.01) //prevent division by zero
    {
      double dir_x = dx / distance; //unit direction vector component
      double dir_y = dy / distance; //unit direction vector component

      // Apply spring pull toward vessel if cable stretched
      if(distance > m_cable_length) //only applies when cable is taut
      {
        double overshoot = distance - m_cable_length;
        double k = 5; // spring constant, tune as needed (1/s^2); 5 for now due to cable stretching past length
        m_towed_vx += k * overshoot * dir_x * dt; // Update x velocity
        m_towed_vy += k * overshoot * dir_y * dt; // Update y velocity
      }

      //simple drag
      double c = .2; //linear drag (1/s)
      m_towed_vx -= c * m_towed_vx * dt; // Update x velocity with drag
      m_towed_vy -= c * m_towed_vy * dt; // Update y velocity with drag

      //integrate velocity to get new position
      m_towed_x += m_towed_vx * dt;
      m_towed_y += m_towed_vy * dt;
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


//body does not align perfectly with vehicle.