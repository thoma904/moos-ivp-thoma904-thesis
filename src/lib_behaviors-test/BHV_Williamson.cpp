/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Williamson.cpp                              */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_Williamson.h"
#include "IvPFunction.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
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

  m_osx = 0;
  m_osy = 0;
  m_osh = 0;

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING, WPT_INDEX");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Williamson::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "foo") && isNumber(val)) {
    // Set local member variables here
    return(true);
  }
  else if (param == "bar") {
    // return(setBooleanOnString(m_my_bool, val));
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
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_Williamson::onCompleteState()
{
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
  crs_zaic.setPeakWidth(0);
  crs_zaic.setBaseWidth(180.0);
  crs_zaic.setSummitDelta(0);
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
// Part 1: Get heading (treat heading like waypoints)
  bool ok_hdg = false;
  double osh = getBufferDoubleVal("NAV_HEADING", ok_hdg);
  if(!ok_hdg) {
    postWMessage("Williamson: NAV_HEADING missing/stale.");
    return(0);
  }

  bool ok_index = false;
  double wpt_index = getBufferDoubleVal("WPT_INDEX", ok_index);
  if(!ok_index) {
    postWMessage("Williamson: WPT_INDEX missing/stale.");
    return(0);
  }

  if (wpt_index < 1) {
    setComplete();
    return(0);
  }

  else if (wpt_index/2 != (int)(wpt_index/2)) {
    setComplete();
    return(0);
  }

  else
  {
    // ---- Simple knobs (tune here) ----
    const bool   FIRST_PORT = false; // false = starboard-first, true = port-first
    const double HDG_TOL    = 5.0;   // deg: when |error| <= tol, advance to next heading
    const double A1         = 60.0;  // deg offset for the first heading
    const double A2         = 60.0;  // deg between 2nd and 3rd (so total ends at reciprocal)

    // ---- Tiny persistent state: a 3-step "heading waypoint" list ----
    // idx: 0 => H1, 1 => H2, 2 => H3, 3 => done
    static bool   s_started = false;
    static int    s_idx     = 0;
    static double s_targets[3];   // H1, H2, H3
    static double s_stem_hdg = 0;

    // Helper: |heading error| using smallest difference (0..180)
    auto hdg_err = [](double a, double b) { return angleDiff(a, b); };

    // Part 2: Initialize "heading waypoints" the first time we run
    if(!s_started) {
      s_started  = true;
      s_idx      = 0;
      s_stem_hdg = osh;

      // H1/H2 force the *direction* change. H3 is the reciprocal.
      if(FIRST_PORT) {
        s_targets[0] = angle360(s_stem_hdg - A1);      // port 60
        s_targets[1] = angle360(s_stem_hdg + A2);      // reverse (starboard) 120 to -60
      } else {
        s_targets[0] = angle360(s_stem_hdg + A1);      // starboard 60
        s_targets[1] = angle360(s_stem_hdg - A2);      // reverse (port) 120 to -60
      }
      s_targets[2] = angle360(s_stem_hdg + 180.0);     // reciprocal
    }

    // Part 3: If we're within tolerance of the current heading-target, advance
    if(s_idx < 3 && hdg_err(osh, s_targets[s_idx]) <= HDG_TOL)
      ++s_idx;

    // Done? Mimic waypoint behavior: mark complete and return no function
    if(s_idx >= 3) {
      s_started = false;
      setComplete();
      return(0);
    }

    // Part 4: Emit a course-only preference toward the current heading "waypoint"
    IvPFunction* ipf = buildFunctionWithZAIC(s_targets[s_idx]);
    return ipf;
  }
}

