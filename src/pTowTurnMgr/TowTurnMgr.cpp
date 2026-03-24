/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: TowTurnMgr.cpp                                  */
/*    DATE:                                                 */
/*                                                          */
/* Manages teardrop turns for a towed vehicle in a          */
/* lawnmower pattern. Detects turn points by comparing      */
/* the vehicle heading with the bearing to the next         */
/* waypoint when a new waypoint is captured. Publishes      */
/* TURN_ACTIVE and TOW_PAST_WPT for BHV_TowedTurn.         */
/************************************************************/

#include <iterator>
#include <cmath>
#include "MBUtils.h"
#include "ACTable.h"
#include "TowTurnMgr.h"
#include "AngleUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor

TowTurnMgr::TowTurnMgr()
{
  m_turn_hdg_thresh = 120;

  m_nav_x = 0;
  m_nav_y = 0;
  m_nav_heading = 0;
  m_towed_x = 0;
  m_towed_y = 0;
  m_prev_x = 0;
  m_prev_y = 0;
  m_next_x = 0;
  m_next_y = 0;
  m_wpt_index = -1;
  m_prev_wpt_index = -1;
  m_tow_deployed = false;
  m_turn_active = false;

  m_last_leg_bearing = 0;
  m_last_leg_valid = false;

  m_turn_pt_x = 0;
  m_turn_pt_y = 0;
  m_approach_hdg = 0;
}

//---------------------------------------------------------
TowTurnMgr::~TowTurnMgr() {}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool TowTurnMgr::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

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
    else if(key == "PREV_WPT_X")
      m_prev_x = msg.GetDouble();
    else if(key == "PREV_WPT_Y")
      m_prev_y = msg.GetDouble();
    else if(key == "NEXT_WPT_X")
      m_next_x = msg.GetDouble();
    else if(key == "NEXT_WPT_Y")
      m_next_y = msg.GetDouble();
    else if(key == "WPT_INDEX")
      m_wpt_index = (int)msg.GetDouble();
    else if(key == "TOW_DEPLOYED") {
      string val = msg.GetString();
      m_tow_deployed = (val == "true" || val == "TRUE");
    }
    else if(key == "TURN_ACTIVE") {
      string val = msg.GetString();
      if(val == "false" || val == "FALSE") {
        m_turn_active = false;
        // Update reference bearing to the current leg so the
        // next long leg isn't compared against the old direction
        double leg_len = hypot(m_next_x - m_prev_x, m_next_y - m_prev_y);
        if(leg_len > 1.0) {
          m_last_leg_bearing = relAng(m_prev_x, m_prev_y, m_next_x, m_next_y);
          m_last_leg_valid = true;
        }
      }
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool TowTurnMgr::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool TowTurnMgr::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Detect new waypoint capture by index change
  if(m_wpt_index != m_prev_wpt_index && m_wpt_index >= 0)
  {
    m_prev_wpt_index = m_wpt_index;

    // Compute bearing and length of the new leg
    double new_leg_bearing = relAng(m_prev_x, m_prev_y, m_next_x, m_next_y);
    double new_leg_length  = hypot(m_next_x - m_prev_x, m_next_y - m_prev_y);

    // Detect turn points by comparing the new leg with the last
    // long leg. Two cases trigger a teardrop:
    //   1) New leg is short (shift segment) and angled >60 deg from
    //      the last long leg — the vehicle is at the end of an
    //      outbound leg and about to reverse.
    //   2) New leg is long and reversed (>120 deg) from the last
    //      long leg — direct reversal without a shift.
    if(m_last_leg_valid && m_tow_deployed && !m_turn_active)
    {
      double leg_diff = fabs(angleDiff(m_last_leg_bearing, new_leg_bearing));
      bool is_short = (new_leg_length <= 30.0);

      if((is_short && leg_diff > 60) || (!is_short && leg_diff > m_turn_hdg_thresh))
      {
        m_turn_active = true;
        m_turn_pt_x = m_prev_x;
        m_turn_pt_y = m_prev_y;
        m_approach_hdg = m_last_leg_bearing;

        Notify("TURN_ACTIVE", "true");
        Notify("LEG_HEADING", m_approach_hdg);
      }
    }

    // Only update the reference leg bearing for long legs (>30m).
    // Short shift segments are ignored so we always compare
    // long leg directions across the shift.
    if(new_leg_length > 30.0) {
      m_last_leg_bearing = new_leg_bearing;
      m_last_leg_valid = true;
    }
  }

  // While turn is active, check if tow body has passed the turn point
  if(m_turn_active)
  {
    // Project tow position onto the approach direction.
    // "Past" means the tow has reached or gone beyond the turn
    // point along the direction the vehicle was traveling.
    double hdg_rad = (90.0 - m_approach_hdg) * M_PI / 180.0;
    double ux = cos(hdg_rad);
    double uy = sin(hdg_rad);

    // Signed distance of tow body past the turn point
    double tow_along = (m_towed_x - m_turn_pt_x) * ux
                     + (m_towed_y - m_turn_pt_y) * uy;

    bool tow_past = (tow_along >= 0);
    Notify("TOW_PAST_WPT", tow_past ? "1" : "0");
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool TowTurnMgr::OnStartUp()
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

    if(param == "turn_hdg_thresh") {
      m_turn_hdg_thresh = atof(value.c_str());
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

void TowTurnMgr::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("TOWED_X", 0);
  Register("TOWED_Y", 0);
  Register("PREV_WPT_X", 0);
  Register("PREV_WPT_Y", 0);
  Register("NEXT_WPT_X", 0);
  Register("NEXT_WPT_Y", 0);
  Register("WPT_INDEX", 0);
  Register("TOW_DEPLOYED", 0);
  Register("TURN_ACTIVE", 0);
}

//---------------------------------------------------------
// Procedure: buildReport()

bool TowTurnMgr::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "pTowTurnMgr Status                         " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "WPT Index | Turn Active | Tow Deployed | Hdg Thresh";
  actab.addHeaderLines();
  actab << intToString(m_wpt_index)
        << boolToString(m_turn_active)
        << boolToString(m_tow_deployed)
        << doubleToString(m_turn_hdg_thresh);
  m_msgs << actab.getFormattedString() << endl;

  if(m_turn_active) {
    m_msgs << "Turn point: (" << doubleToString(m_turn_pt_x,1)
           << ", " << doubleToString(m_turn_pt_y,1) << ")" << endl;
    m_msgs << "Approach hdg: " << doubleToString(m_approach_hdg,1) << endl;
  }

  return(true);
}
