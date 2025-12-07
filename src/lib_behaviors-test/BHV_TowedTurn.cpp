/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_TowedTurn.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_TowedTurn.h"
#include "ZAIC_PEAK.h"
#include "AngleUtils.h"
#include <GeomUtils.h>
#include <cmath>

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_TowedTurn::BHV_TowedTurn(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "tow_turn");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Default Values
  m_turn_dir = 1;
  m_target_hdg = 0;

  m_entry_hdg_set = false;
  m_entry_hdg = 0;

  m_phase1_done = false;
  m_phase1_hdg = 0;

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_HEADING, NAV_SPEED, NAV_X, NAV_Y, TOW_PAST_WPT");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_TowedTurn::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  // Params specific to this behavior
    if(param == "turn_dir") 
    {
      double dval = atof(val.c_str());
      if (dval > 1 || dval < -1) 
        return(false);
      else
      {
      m_turn_dir = dval;
      return(true);
      }
    }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_TowedTurn::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_TowedTurn::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_TowedTurn::onIdleState()
{
 m_entry_hdg_set = false;
 m_phase1_done = false;
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_TowedTurn::onCompleteState()
{
 m_entry_hdg_set = false;
 m_phase1_done = false;
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_TowedTurn::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_TowedTurn::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_TowedTurn::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_TowedTurn::onRunState()
{
  // 1) Get nav info
  bool ok_hdg = false;
  double nav_hdg = getBufferDoubleVal("NAV_HEADING", ok_hdg);

  if(!ok_hdg)
    return(0);  // No influence if we don't know heading yet

  // 2) On first run: latch entry heading, phase-1 heading, and target heading
  if(!m_entry_hdg_set) 
  {
    m_entry_hdg = angle360(nav_hdg);

    // Overall plan:
    // - Phase 1: go 60 deg opposite direction of the port turn (i.e. starboard)
    m_phase1_hdg = angle360(m_entry_hdg - m_turn_dir * 60);   // 60 deg

    // - Final target: reciprocal of entry heading
    m_target_hdg = angle360(m_entry_hdg - (180.0*m_turn_dir));

    m_phase1_done   = false;
    m_entry_hdg_set = true;
  }

  //3) Check if tow is past the waypoint
  bool  ok_tow_flag   = false;
  double tow_flag_val = getBufferDoubleVal("TOW_PAST_WPT", ok_tow_flag);

  // Interpret any non-zero as "true"
  bool tow_past_wpt = ok_tow_flag && (tow_flag_val > 0.5);

  double course_des;

  if(!tow_past_wpt)
    course_des = m_entry_hdg; // Hold entry heading until tow is past waypoint
  

  //4) Conduct Teardrop turn if tow is past waypoint
  else
  {
    double turned = fabs(angleDiff(nav_hdg, m_entry_hdg));

    // 3) Phase 1: drive to H0 + 45 deg (starboard)
    if(!m_phase1_done) 
    {
      course_des = m_phase1_hdg;

      if(turned >= 45)
        m_phase1_done = true;
    }

    else {
      // CCW (port) angular distance from current to target
      // (current - target) wrapped into [0,360) is the CCW amount
      double diff_port = angle360(nav_hdg - m_target_hdg);  // [0,360)
      double diff_star = angle360(m_target_hdg - nav_hdg);  // [0,360)

      double diff_dir = (m_turn_dir > 0) ? diff_star : diff_port;

      const double tol_final = 10.0;     // deg
      if(diff_dir < tol_final) {        // close enough to the target
        setComplete();
        return(0);
      }

      double max_step = 60;            // deg per decision
      double step     = std::min(diff_dir, max_step);

      // PORT turn: decrease heading
      course_des = angle360(nav_hdg + m_turn_dir * step);
    }
  }

  // 5) Build IvP function preferring the desired course
  ZAIC_PEAK zaic(m_domain, "course");
  zaic.setSummit(course_des);
  zaic.setBaseWidth(120.0);
  zaic.setPeakWidth(5.0);
  zaic.setSummitDelta(100.0);

  IvPFunction* ipf = zaic.extractIvPFunction();
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}
