/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Cable.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <cmath>
#include <algorithm>
#include "MBUtils.h"
#include "ACTable.h"
#include "Cable.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

Cable::Cable()
{
  // Configuration defaults (match pTowing1)
  m_cable_length      = 30.0;
  m_attach_offset     = 0.0;
  m_k_spring          = 5.0;
  m_cd                = 0.7;
  m_c_tan             = 2.0;

  // State
  m_nav_x             = 0;
  m_nav_y             = 0;
  m_nav_heading       = 0;
  m_towed_x           = 0;
  m_towed_y           = 0;

  m_initialized       = false;
  m_num_nodes         = 3;
  m_rest_length       = 0;
  m_last_iterate_time = -1;
}

//---------------------------------------------------------
// Destructor

Cable::~Cable()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool Cable::OnNewMail(MOOSMSG_LIST &NewMail)
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
    else if(key == "TOWED_X")
      m_towed_x = msg.GetDouble();
    else if(key == "TOWED_Y")
      m_towed_y = msg.GetDouble();
    // Dynamic parameter sync from pTowing1
    else if(key == "TOW_CABLE_LENGTH") {
      m_cable_length = msg.GetDouble();
      if(m_cable_length < 30.0)
        m_num_nodes = 3;
      else
        m_num_nodes = std::max(3, (int)(m_cable_length / 10.0));
      m_rest_length = m_cable_length / (double)(m_num_nodes - 1);
      m_initialized = false;
    }
    else if(key == "TOW_ATTACH_OFFSET")
      m_attach_offset = msg.GetDouble();
    else if(key == "TOW_SPRING_STIFFNESS")
      m_k_spring = msg.GetDouble();
    else if(key == "TOW_DRAG_COEFF")
      m_cd = msg.GetDouble();
    else if(key == "TOW_TAN_DAMPING")
      m_c_tan = msg.GetDouble();
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool Cable::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Cable::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // ============================================================
  // Initialization: place nodes on straight line from anchor
  // to tow body. Re-runs if cable length changes.
  // ============================================================
  if(!m_initialized) {
    m_nodes.clear();
    m_nodes.resize(m_num_nodes);

    double hdg_rad_init = (90.0 - m_nav_heading) * M_PI / 180.0;
    double anchor_x = m_nav_x - m_attach_offset * cos(hdg_rad_init);
    double anchor_y = m_nav_y - m_attach_offset * sin(hdg_rad_init);

    for(int i = 0; i < m_num_nodes; i++) {
      double t = (double)i / (double)(m_num_nodes - 1);
      m_nodes[i].x  = anchor_x + t * (m_towed_x - anchor_x);
      m_nodes[i].y  = anchor_y + t * (m_towed_y - anchor_y);
      m_nodes[i].vx = 0;
      m_nodes[i].vy = 0;
    }
    m_initialized = true;
  }

  // ============================================================
  // Compute dt
  // ============================================================
  double now = MOOSTime();
  double dt  = 0.0;
  if(m_last_iterate_time > 0)
    dt = now - m_last_iterate_time;
  m_last_iterate_time = now;

  if(dt <= 0 || dt > 1.0)
    dt = 1.0 / GetAppFreq();

  // ============================================================
  // Pin endpoints
  // ============================================================
  double hdg_rad = (90.0 - m_nav_heading) * M_PI / 180.0;

  m_nodes[0].x  = m_nav_x - m_attach_offset * cos(hdg_rad);
  m_nodes[0].y  = m_nav_y - m_attach_offset * sin(hdg_rad);
  m_nodes[0].vx = 0;
  m_nodes[0].vy = 0;

  m_nodes[m_num_nodes - 1].x  = m_towed_x;
  m_nodes[m_num_nodes - 1].y  = m_towed_y;
  m_nodes[m_num_nodes - 1].vx = 0;
  m_nodes[m_num_nodes - 1].vy = 0;

  // ============================================================
  // Interior node dynamics (matches pTowing1/AOF physics)
  // For each interior node: spring from both neighbors,
  // quadratic drag, tangential damping, Euler integration.
  // ============================================================
  for(int i = 1; i < m_num_nodes - 1; i++)
  {
    CableNode &node = m_nodes[i];

    // --- Spring force from PREVIOUS neighbor (i-1) ---
    double dx_prev   = m_nodes[i-1].x - node.x;
    double dy_prev   = m_nodes[i-1].y - node.y;
    double dist_prev = hypot(dx_prev, dy_prev);

    if(dist_prev > 0.01 && dist_prev > m_rest_length && m_k_spring > 0) {
      double ux = dx_prev / dist_prev;
      double uy = dy_prev / dist_prev;
      double overshoot = dist_prev - m_rest_length;
      node.vx += m_k_spring * overshoot * ux * dt;
      node.vy += m_k_spring * overshoot * uy * dt;
    }

    // --- Spring force from NEXT neighbor (i+1) ---
    double dx_next   = m_nodes[i+1].x - node.x;
    double dy_next   = m_nodes[i+1].y - node.y;
    double dist_next = hypot(dx_next, dy_next);

    if(dist_next > 0.01 && dist_next > m_rest_length && m_k_spring > 0) {
      double ux = dx_next / dist_next;
      double uy = dy_next / dist_next;
      double overshoot = dist_next - m_rest_length;
      node.vx += m_k_spring * overshoot * ux * dt;
      node.vy += m_k_spring * overshoot * uy * dt;
    }

    // --- Quadratic drag ---
    double speed = hypot(node.vx, node.vy);
    if(speed > 1e-6 && m_cd > 0) {
      node.vx += -m_cd * node.vx * speed * dt;
      node.vy += -m_cd * node.vy * speed * dt;
    }

    // --- Tangential damping ---
    // Cable direction: vector from node[i-1] to node[i+1]
    // Normal is perpendicular; damp velocity component along the normal
    if(m_c_tan > 0) {
      double cx   = m_nodes[i+1].x - m_nodes[i-1].x;
      double cy   = m_nodes[i+1].y - m_nodes[i-1].y;
      double clen = hypot(cx, cy);
      if(clen > 1e-6) {
        double ux = cx / clen;  // unit tangent along cable
        double uy = cy / clen;
        double nx = -uy;        // unit normal (perpendicular)
        double ny =  ux;
        double vn = node.vx * nx + node.vy * ny;
        node.vx += (-m_c_tan * vn) * nx * dt;
        node.vy += (-m_c_tan * vn) * ny * dt;
      }
    }

    // --- Euler position integration ---
    node.x += node.vx * dt;
    node.y += node.vy * dt;
  }

  // ============================================================
  // Rigid distance constraints (bidirectional pass)
  // Forward: clamp interior nodes relative to predecessor
  // Backward: clamp interior nodes relative to successor
  // ============================================================

  // Forward pass
  for(int i = 1; i < m_num_nodes - 1; i++) {
    double dx   = m_nodes[i-1].x - m_nodes[i].x;
    double dy   = m_nodes[i-1].y - m_nodes[i].y;
    double dist = hypot(dx, dy);

    if(dist > m_rest_length && dist > 1e-9) {
      double sc = m_rest_length / dist;
      m_nodes[i].x = m_nodes[i-1].x - dx * sc;
      m_nodes[i].y = m_nodes[i-1].y - dy * sc;

      double urx  = dx / dist;
      double ury  = dy / dist;
      double vrad = m_nodes[i].vx * urx + m_nodes[i].vy * ury;
      if(vrad < 0) {
        m_nodes[i].vx -= vrad * urx;
        m_nodes[i].vy -= vrad * ury;
      }
    }
  }

  // Backward pass
  for(int i = m_num_nodes - 2; i >= 1; i--) {
    double dx   = m_nodes[i+1].x - m_nodes[i].x;
    double dy   = m_nodes[i+1].y - m_nodes[i].y;
    double dist = hypot(dx, dy);

    if(dist > m_rest_length && dist > 1e-9) {
      double sc = m_rest_length / dist;
      m_nodes[i].x = m_nodes[i+1].x - dx * sc;
      m_nodes[i].y = m_nodes[i+1].y - dy * sc;

      double urx  = dx / dist;
      double ury  = dy / dist;
      double vrad = m_nodes[i].vx * urx + m_nodes[i].vy * ury;
      if(vrad < 0) {
        m_nodes[i].vx -= vrad * urx;
        m_nodes[i].vy -= vrad * ury;
      }
    }
  }

  // ============================================================
  // Publish VIEW_SEGLIST and CABLE_NODE_REPORT
  // ============================================================

  // VIEW_SEGLIST: cable shape for pMarineViewer
  string pts_str = "pts={";
  for(int i = 0; i < m_num_nodes; i++) {
    if(i > 0)
      pts_str += ":";
    pts_str += doubleToStringX(m_nodes[i].x, 1) + "," +
               doubleToStringX(m_nodes[i].y, 1);
  }
  pts_str += "}";
  pts_str += ",label=CABLE";
  pts_str += ",edge_color=white,edge_size=1,vertex_size=0";
  Notify("VIEW_SEGLIST", pts_str);

  // CABLE_NODE_REPORT: all node positions for pTowObstacleMgr
  string report = "nodes=" + intToString(m_num_nodes);
  for(int i = 0; i < m_num_nodes; i++) {
    report += ",x" + intToString(i) + "=" + doubleToStringX(m_nodes[i].x, 2);
    report += ",y" + intToString(i) + "=" + doubleToStringX(m_nodes[i].y, 2);
  }
  Notify("CABLE_NODE_REPORT", report);

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Cable::OnStartUp()
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
    if(param == "cable_length") {
      m_cable_length = stod(value);
      handled = true;
    }
    else if(param == "attach_offset") {
      m_attach_offset = stod(value);
      handled = true;
    }
    else if(param == "k_spring") {
      m_k_spring = stod(value);
      handled = true;
    }
    else if(param == "cd") {
      m_cd = stod(value);
      handled = true;
    }
    else if(param == "c_tan") {
      m_c_tan = stod(value);
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Compute node count and rest length
  if(m_cable_length < 30.0)
    m_num_nodes = 3;
  else
    m_num_nodes = std::max(3, (int)(m_cable_length / 10.0));

  m_rest_length = m_cable_length / (double)(m_num_nodes - 1);

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void Cable::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("TOWED_X", 0);
  Register("TOWED_Y", 0);
  Register("TOW_CABLE_LENGTH", 0);
  Register("TOW_ATTACH_OFFSET", 0);
  Register("TOW_SPRING_STIFFNESS", 0);
  Register("TOW_DRAG_COEFF", 0);
  Register("TOW_TAN_DAMPING", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool Cable::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "  pCable Status                             " << endl;
  m_msgs << "============================================" << endl;

  m_msgs << " Cable Length:    " << m_cable_length << " m" << endl;
  m_msgs << " Num Nodes:       " << m_num_nodes << endl;
  m_msgs << " Rest Length:     " << doubleToStringX(m_rest_length, 2) << " m" << endl;
  m_msgs << " k_spring:        " << m_k_spring << endl;
  m_msgs << " cd:              " << m_cd << endl;
  m_msgs << " c_tan:           " << m_c_tan << endl;
  m_msgs << " attach_offset:   " << m_attach_offset << " m" << endl;
  m_msgs << " Initialized:     " << (m_initialized ? "true" : "false") << endl;
  m_msgs << endl;

  m_msgs << " Vessel:   (" << doubleToStringX(m_nav_x, 1) << ", "
         << doubleToStringX(m_nav_y, 1) << ") hdg="
         << doubleToStringX(m_nav_heading, 1) << endl;
  m_msgs << " Tow Body: (" << doubleToStringX(m_towed_x, 1) << ", "
         << doubleToStringX(m_towed_y, 1) << ")" << endl;
  m_msgs << endl;

  if(m_initialized) {
    ACTable actab(4);
    actab << "Node | X | Y | Speed";
    actab.addHeaderLines();
    for(int i = 0; i < m_num_nodes; i++) {
      string label;
      if(i == 0)
        label = "0(attach)";
      else if(i == m_num_nodes - 1)
        label = intToString(i) + "(tow)";
      else
        label = intToString(i);

      actab << label
            << doubleToStringX(m_nodes[i].x, 2)
            << doubleToStringX(m_nodes[i].y, 2)
            << doubleToStringX(hypot(m_nodes[i].vx, m_nodes[i].vy), 3);
    }
    m_msgs << actab.getFormattedString();
  }

  return(true);
}




