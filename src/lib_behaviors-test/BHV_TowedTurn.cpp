/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT                                             */
/*    FILE: BHV_TowedTurn.cpp                               */
/*    DATE:                                                 */
/*                                                          */
/* Executes a teardrop turn at the end of a lawnmower leg   */
/* for a vessel towing a body. Computes turn direction      */
/* from NEXT_WPT_X/Y so the vehicle turns toward the next   */
/* leg. Two phases:                                         */
/*   Phase 1 (jog): brief swing opposite the main turn      */
/*     direction to pull the tow clear.                     */
/*   Phase 2 (sweep): turn in the main direction toward     */
/*     the reciprocal heading.                              */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include <cmath>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_TowedTurn.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "AngleUtils.h"
#include "GeomUtils.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_TowedTurn::BHV_TowedTurn(IvPDomain domain) :
  IvPBehavior(domain)
{
  IvPBehavior::setParam("name", "tow_turn");
  m_domain = subDomain(m_domain, "course,speed");

  m_jog_angle = 60;

  m_turn_dir      = 1;
  m_target_hdg    = 0;
  m_jog_hdg       = 0;
  m_entry_hdg_set = false;
  m_entry_hdg     = 0;
  m_jog_done      = false;

  addInfoVars("NAV_HEADING, NAV_X, NAV_Y, TOW_PAST_WPT");
  addInfoVars("NEXT_WPT_X, NEXT_WPT_Y", "no_warning");
  addInfoVars("LEG_HEADING", "no_warning");
  addInfoVars("TOW_OBS_AVOIDING", "no_warning");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_TowedTurn::setParam(string param, string val)
{
  param = tolower(param);
  double dval = atof(val.c_str());

  if(param == "turn_dir") {
    if(dval > 0)
      m_turn_dir = 1;
    else if(dval < 0)
      m_turn_dir = -1;
    else
      return(false);
    return(true);
  }
  else if(param == "jog_angle") {
    if(dval > 0 && dval <= 90) {
      m_jog_angle = dval;
      return(true);
    }
    return(false);
  }

  return(false);
}

//---------------------------------------------------------------
void BHV_TowedTurn::onSetParamComplete() {}
void BHV_TowedTurn::onHelmStart() {}
void BHV_TowedTurn::postConfigStatus() {}
void BHV_TowedTurn::onIdleToRunState() {}
void BHV_TowedTurn::onRunToIdleState() {}

//---------------------------------------------------------------
void BHV_TowedTurn::onIdleState()
{
  m_entry_hdg_set = false;
  m_jog_done      = false;
}

//---------------------------------------------------------------
void BHV_TowedTurn::onCompleteState()
{
  m_entry_hdg_set = false;
  m_jog_done      = false;
}

//---------------------------------------------------------------
// Procedure: onRunState()

IvPFunction* BHV_TowedTurn::onRunState()
{
  // 1) Get nav info
  bool ok_hdg = false, ok_x = false, ok_y = false;
  double nav_hdg = getBufferDoubleVal("NAV_HEADING", ok_hdg);
  double nav_x   = getBufferDoubleVal("NAV_X", ok_x);
  double nav_y   = getBufferDoubleVal("NAV_Y", ok_y);

  if(!ok_hdg || !ok_x || !ok_y)
    return(0);

  // 1b) Yield to obstacle avoidance — if tow obstacle avoidance
  //     posted TOW_OBS_AVOIDING within the last 1 second, don't
  //     produce an IVP function so avoidance has full control.
  //     State is preserved so the turn resumes where it left off.
  double obs_age = getBufferTimeVal("TOW_OBS_AVOIDING");
  if(obs_age >= 0 && obs_age < 1.0)
    return(0);

  // 2) On first run: latch entry heading, compute turn direction
  //    from next waypoint, set up jog and target headings
  if(!m_entry_hdg_set) {
    // Prefer LEG_HEADING (the actual leg bearing from pTowTurnMgr)
    // over NAV_HEADING, which may be off course due to obstacle avoidance
    bool ok_leg = false;
    double leg_hdg = getBufferDoubleVal("LEG_HEADING", ok_leg);
    m_entry_hdg = ok_leg ? angle360(leg_hdg) : angle360(nav_hdg);

    // Compute turn direction from next waypoint bearing
    bool ok_nx = false, ok_ny = false;
    double next_x = getBufferDoubleVal("NEXT_WPT_X", ok_nx);
    double next_y = getBufferDoubleVal("NEXT_WPT_Y", ok_ny);

    if(ok_nx && ok_ny) {
      double bearing = relAng(nav_x, nav_y, next_x, next_y);
      // Signed angle from entry heading to bearing: positive = starboard
      double raw = angle360(bearing) - angle360(m_entry_hdg);
      if(raw > 180)  raw -= 360;
      if(raw < -180) raw += 360;
      // Next WP to starboard (raw > 0): sweep starboard, jog port
      // Next WP to port (raw < 0): sweep port, jog starboard
      m_turn_dir = (raw >= 0) ? 1 : -1;
    }
    // else: keep turn_dir from config/updates as fallback

    // Target: reciprocal of entry heading
    m_target_hdg = angle360(m_entry_hdg + 180.0);

    // Jog: swing opposite the main turn direction
    m_jog_hdg = angle360(m_entry_hdg - m_turn_dir * m_jog_angle);

    m_jog_done      = false;
    m_entry_hdg_set = true;
  }

  // 3) Check if tow body is past the waypoint
  bool ok_tow = false;
  double tow_flag = getBufferDoubleVal("TOW_PAST_WPT", ok_tow);
  bool tow_past = ok_tow && (tow_flag > 0.5);

  double course_des = m_entry_hdg;

  if(!tow_past) {
    // Hold entry heading until tow clears the waypoint
    course_des = m_entry_hdg;
  }
  else if(!m_jog_done) {
    // Phase 1: jog opposite the main turn direction
    course_des = m_jog_hdg;

    // Check if the vehicle heading is close to the jog heading.
    // This works regardless of what heading the vehicle started
    // from (e.g., if off-course due to obstacle avoidance).
    double dist_to_jog = angleDiff(nav_hdg, m_jog_hdg);
    if(dist_to_jog < 15.0)
      m_jog_done = true;
  }
  else {
    // Phase 2: sweep in the main turn direction toward target
    double remaining;
    if(m_turn_dir > 0)
      remaining = angle360(m_target_hdg - nav_hdg);  // starboard distance
    else
      remaining = angle360(nav_hdg - m_target_hdg);   // port distance

    if(remaining < 10.0) {
      // Close enough — produce a final IVP function then complete
      course_des = m_target_hdg;
      setComplete();
    }
    else {
      double step = std::min(remaining, 60.0);
      course_des = angle360(nav_hdg + m_turn_dir * step);
    }
  }

  // 4) Build IvP function for course and speed
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(course_des);
  crs_zaic.setBaseWidth(120.0);
  crs_zaic.setPeakWidth(5.0);
  crs_zaic.setSummitDelta(100.0);

  ZAIC_PEAK spd_zaic(m_domain, "speed");
  spd_zaic.setSummit(1.0);
  spd_zaic.setBaseWidth(0.4);
  spd_zaic.setPeakWidth(0.0);
  spd_zaic.setSummitDelta(25.0);

  IvPFunction* crs_ipf = crs_zaic.extractIvPFunction();
  IvPFunction* spd_ipf = spd_zaic.extractIvPFunction();

  IvPFunction* ipf = 0;
  if(crs_ipf && spd_ipf) {
    OF_Coupler coupler;
    ipf = coupler.couple(crs_ipf, spd_ipf);
  }
  else if(crs_ipf)
    ipf = crs_ipf;
  else if(spd_ipf)
    ipf = spd_ipf;

  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}
