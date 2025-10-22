/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Towing.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <limits>
#include <cstdlib>
#include <cmath>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_Towing.h"
#include "OF_Reflector.h"
#include "XYFormatUtilsPoly.h"
#include "XYPolygon.h"
#include "AngleUtils.h"
#include "ZAIC_PEAK.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_Towing::BHV_Towing(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "towing");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("TOWED_X, TOWED_Y");
  addInfoVars(m_obstacle_var);
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Towing::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());

  if(param == "peak_width" && isNumber(val)) {
      m_peak_width = atof(val.c_str());
      return true;
    }
  else if(param == "base_width" && isNumber(val)) {
      m_base_width = atof(val.c_str());
      return true;
    }
  else if(param == "summit_delta" && isNumber(val)) {
      m_summit_delta = atof(val.c_str());
      return true;
    }
  else if(param == "obstacle_var") {
      m_obstacle_var = val;
      addInfoVars(m_obstacle_var);
      return true;
    }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_Towing::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_Towing::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_Towing::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_Towing::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_Towing::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_Towing::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_Towing::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_Towing::onRunState()
{
  // Part 1: Build the IvP function

    // Tow center
  bool okx=false, oky=false;
  double tx = getBufferDoubleVal("TOWED_X", okx);
  double ty = getBufferDoubleVal("TOWED_Y", oky);
  if(!okx || !oky)
    return nullptr;

  // Obstacles (vector<string> of "poly=..." specs)
  bool ok=true;
  vector<string> specs = getBufferStringVector(m_obstacle_var, ok);
  if(!ok || specs.empty())
    return nullptr;

  // Find the nearest *vertex* on any polygon to the tow center
  double best_d = numeric_limits<double>::infinity();
  double vxn=tx, vyn=ty;

  for(const auto& s : specs) {
    XYPolygon poly = string2Poly(s);
    if(poly.size() < 1) continue;
    for(unsigned i=0; i<poly.size(); ++i) {
      double vx = poly.get_vx(i);
      double vy = poly.get_vy(i);
      double d  = hypot(tx - vx, ty - vy);
      if(d < best_d) { best_d = d; vxn = vx; vyn = vy; }
    }
  }

  if(!isfinite(best_d))  // nothing usable
    return nullptr;

  // Preferred course = away from nearest vertex
  double bng_to_vert = relAng(tx, ty, vxn, vyn);      // 0..359
  double crs_pref    = angle360(bng_to_vert + 180.0); // turn away

  // Build a tiny ZAIC over course (fixed widths for simplicity)
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(crs_pref);
  crs_zaic.setValueWrap(true);
  crs_zaic.setPeakWidth(m_peak_width);
  crs_zaic.setBaseWidth(m_base_width);
  crs_zaic.setSummitDelta(m_summit_delta);

  if(!crs_zaic.stateOK()){
    postWMessage("Course ZAIC problem: " + crs_zaic.getWarnings());
    return nullptr;
  }

  IvPFunction* ipf = crs_zaic.extractIvPFunction();
  if(ipf) ipf->setPWT(m_priority_wt);
  return ipf;
}

