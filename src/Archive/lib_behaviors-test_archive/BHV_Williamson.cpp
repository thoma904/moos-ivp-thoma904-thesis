/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Williamson.cpp                              */
/*    DATE:                                                 */
/************************************************************/

#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_Williamson.h"
#include "IvPFunction.h"
#include "ZAIC_PEAK.h"
#include "AngleUtils.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_Williamson::BHV_Williamson(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "Williamson");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course");

  m_entry_hdg_set = false;
  m_entry_hdg     = 0;
  m_target_hdg    = 0;
  m_hdg_tol       = 1.0;   // deg tolerance for completion
  m_turn_dir     = 1;      // default starboard first
  m_initial_turn = 60.0;
  m_max_step     = 90.0;   // deg per iteration in phase2
  m_capture_range   = 30.0;   // start capturing inside 30 deg of target
  m_settle_count    = 0;
  m_settle_required = 5;      // must be within tolerance for 5 helm iterations

  m_phase1_done  = false;
  m_phase1_hdg   = 0;

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_HEADING");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Williamson::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());

  if((param == "heading_tolerance") && isNumber(val)) 
  {
    m_hdg_tol = double_val;
    return(true);
  }
  
  else if((param == "initial_turn") && isNumber(val)) 
  {
    m_initial_turn = double_val;
    return(true);
  }
  
  else if((param == "max_step") && isNumber(val)) 
  {
    m_max_step = double_val;
    return(true);
  }
  
  else if(param == "turn_direction") 
  {
    val = tolower(val);
    if(val == "port") 
    {
      m_turn_dir = -1;
      return(true);
    }
    else if(val == "starboard") 
    {
      m_turn_dir = 1;
      return(true);
    }
    return(false);
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_Williamson::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_Williamson::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_Williamson::onIdleState()
{
  m_entry_hdg_set = false;
  m_phase1_done   = false;
  m_settle_count  = 0;
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_Williamson::onCompleteState()
{
  m_entry_hdg_set = false;
  m_phase1_done   = false;
  m_settle_count  = 0;
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_Williamson::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_Williamson::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_Williamson::onRunToIdleState()
{
}

//-----------------------------------------------------------
// Procedure: buildFunctionWithZAIC

IvPFunction *BHV_Williamson::buildFunctionWithZAIC(double target_heading) 
{
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(angle360(target_heading));
  crs_zaic.setPeakWidth(0.0);
  crs_zaic.setBaseWidth(60.0);
  crs_zaic.setSummitDelta(0.0);
  crs_zaic.setValueWrap(true);

  if(!crs_zaic.stateOK()) {
    postWMessage("Course ZAIC problems: " + crs_zaic.getWarnings());
    return(nullptr);
  }

  IvPFunction* ipf = crs_zaic.extractIvPFunction(false);
  if(ipf) ipf->setPWT(m_priority_wt);
  return ipf;
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_Williamson::onRunState()
{
  bool ok_hdg = false;
  double nav_hdg = getBufferDoubleVal("NAV_HEADING", ok_hdg);

  if(!ok_hdg) 
  {
    postWMessage("Williamson: NAV_HEADING missing/stale.");
    return(0);
  }

  nav_hdg = angle360(nav_hdg);

  // First iteration while active: latch entry heading and compute phase headings
  if(!m_entry_hdg_set) {
    m_entry_hdg     = nav_hdg;

    // Phase 1 target: 60 deg in initial turn direction
    // turn_dir: +1 = starboard (increase heading), -1 = port (decrease)
    m_phase1_hdg    = angle360(m_entry_hdg + m_turn_dir * m_initial_turn);

    // Final target: reciprocal of entry
    m_target_hdg    = angle360(m_entry_hdg + 180.0);

    m_phase1_done   = false;
    m_entry_hdg_set = true;
  }

  double course_des = m_target_hdg;

  // -----------------------
  // Phase 1: initial 60 deg turn (toward MOB side)
  // -----------------------
  if(!m_phase1_done) {

    // Signed delta from entry to current in [-180,180]
    // Positive = starboard, Negative = port
    double delta = angle180(nav_hdg - m_entry_hdg);

    // When we've turned at least initial_turn in the commanded direction, switch to phase 2
    if((m_turn_dir * delta) >= (m_initial_turn - m_hdg_tol)) 
    {
      m_phase1_done = true;
    } 
    else 
    {
      // Keep voting for the phase1 heading
      course_des = m_phase1_hdg;
      return buildFunctionWithZAIC(course_des);
    }
  }

  // -----------------------
  // Phase 2: reverse direction and continue turning "the long way" to reciprocal
  // This creates the 240 deg portion of the Williamson turn.
  // -----------------------
  int dir2 = -m_turn_dir;  // reverse direction from phase 1

  // Compute remaining angle to target if we turn in dir2
  // Starboard (CW/increasing): diff_star = target - current wrapped [0,360)
  // Port     (CCW/decreasing): diff_port = current - target wrapped [0,360)
  double diff_star = angle360(m_target_hdg - nav_hdg);
  double diff_port = angle360(nav_hdg - m_target_hdg);

  double diff_dir = (dir2 > 0) ? diff_star : diff_port;

  // --- Capture mode ---
  // When close enough, stop leading and command the target heading directly.
  // (By the time diff_dir is small (e.g., 30 deg), the short-way direction is dir2 anyway,
  // so commanding the target won't "shortcut" the 240 deg portion.)
  if(diff_dir <= m_capture_range) {
    course_des = m_target_hdg;        // tighten up and settle
  } else {
    double step = std::min(diff_dir, m_max_step);
    course_des  = angle360(nav_hdg + dir2 * step);  // keep forcing turn direction
  }

  // --- Completion logic ---
  // Use absolute heading error to target (shortest difference).
  double err = angleDiff(nav_hdg, m_target_hdg);

  if(err <= m_hdg_tol) {
    m_settle_count++;
    if(m_settle_count >= m_settle_required) {
      setComplete();
      return(0);
    }
  } else {
    m_settle_count = 0;
  }

  return buildFunctionWithZAIC(course_des);
}

