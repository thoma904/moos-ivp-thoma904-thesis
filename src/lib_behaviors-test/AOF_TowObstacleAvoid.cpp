/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng, MIT Cambridge MA             */
/*    FILE: AOF_TowObstacleAvoid.cpp                             */
/*                                                               */
/*  Tow extension: evaluate BOTH ownship and towed-body CPA       */
/*  and return the conservative utility.                          */
/*****************************************************************/

#include <cmath>
#include <string>
#include "AOF_TowObstacleAvoid.h"

using namespace std;

//----------------------------------------------------------
// Constructor()

AOF_TowObstacleAvoid::AOF_TowObstacleAvoid(IvPDomain gdomain) :
  AOF(gdomain)
{
  m_crs_ix = gdomain.getIndex("course");
  m_spd_ix = gdomain.getIndex("speed");

  // Tow
  m_tow_eval      = false;
  m_tow_pose_set  = false;
  m_tow_x         = 0;
  m_tow_y         = 0;
  m_tow_hdg       = 0;

  m_tow_model_ready = false;
  m_tow_breached    = false;
  m_tow_only       = false;
}

//----------------------------------------------------------------
// setTowPose()

void AOF_TowObstacleAvoid::setTowPose(double x, double y, double hdg)
{
  m_tow_x = x;
  m_tow_y = y;
  m_tow_hdg = hdg;
  m_tow_pose_set = true;
}

//----------------------------------------------------------------
// setParam()

bool AOF_TowObstacleAvoid::setParam(const string& param, double val)
{
  return(false);
}

bool AOF_TowObstacleAvoid::setParam(const string& param, const string& value)
{
  return(false);
}

//----------------------------------------------------------------
// initialize()

bool AOF_TowObstacleAvoid::initialize()
{
  // --- Sanity checks (same as stock AOF) ---
  if(m_crs_ix == -1)
    return(postMsgAOF("crs_ix is not set"));
  if(m_spd_ix == -1)
    return(postMsgAOF("spd_ix is not set"));

  if(!m_obship_model.paramIsSet("osx"))
    return(postMsgAOF("osx is not set"));
  if(!m_obship_model.paramIsSet("osy"))
    return(postMsgAOF("osy is not set"));
  if(!m_obship_model.paramIsSet("osh"))
    return(postMsgAOF("osh is not set"));

  if(!m_obship_model.paramIsSet("min_util_cpa"))
    return(postMsgAOF("min_util_cpa is not set"));
  if(!m_obship_model.paramIsSet("max_util_cpa"))
    return(postMsgAOF("max_util_cpa is not set"));
  if(!m_obship_model.paramIsSet("allowable_ttc"))
    return(postMsgAOF("allowable_ttc is not set"));

  if(!m_obship_model.getGutPoly().is_convex())
    return(postMsgAOF("m_obstacle is not convex"));

  // Standard: fail if ownship starts inside gut
  // If we're NOT tow-only, keep the original behavior: ownship cannot be in gut.
  if(!(m_tow_eval && m_tow_only)) {
    if(m_obship_model.ownshipInGutPoly())
      return(postMsgAOF("m_obstacle contains osx,osy"));
  }
  // If tow-only: ownship may be in the gut poly, that's OK.


  // --- Tow model build ---
  m_tow_model_ready = false;
  m_tow_breached    = false;

  if(m_tow_eval) {
    if(!m_tow_pose_set)
      return(postMsgAOF("tow_eval enabled but tow pose not set"));

    // Build a tow-aware model that uses the SAME turn model and ownship pose,
    // but shifts the obstacle so evaluating ownship CPA == tow CPA.
    // If tow is at (osx+dx, osy+dy), shift obstacle by (-dx,-dy).
    double osx = m_obship_model.getOSX();
    double osy = m_obship_model.getOSY();
    double dx  = m_tow_x - osx;
    double dy  = m_tow_y - osy;

    XYPolygon shifted = m_obship_model.getGutPoly();
    shifted.shift_horz(-dx);
    shifted.shift_vert(-dy);

    m_obship_model_tow = m_obship_model;

    string msg = m_obship_model_tow.setGutPoly(shifted);
    if(msg != "")
      return(postMsgAOF("tow shifted gut poly failed: " + msg));

    // Ensure the mid/rim polys + caches align with shifted gut poly
    m_obship_model_tow.setCachedVals(true);

    // If tow is inside gut, then ownship is inside shifted gut.
    // We do NOT fail init; we just force min-utility in eval.
    m_tow_breached = m_obship_model_tow.ownshipInGutPoly();

    m_tow_model_ready = true;
  }

  return(true);
}

//----------------------------------------------------------------
// evalBox()

double AOF_TowObstacleAvoid::evalBox(const IvPBox *b) const
{
  double eval_crs = 0;
  double eval_spd = 0;

  m_domain.getVal(m_crs_ix, b->pt(m_crs_ix,0), eval_crs);
  m_domain.getVal(m_spd_ix, b->pt(m_spd_ix,0), eval_spd);

  // If tow is enabled and we're tow-only, don't even evaluate nav.
  if(m_tow_eval && m_tow_only) {

    // Tow model not ready -> neutral (or min). I'd recommend neutral==knownMax
    // so the behavior doesn't distort decisions when tow isn't actually active.
    if(!m_tow_pose_set || !m_tow_model_ready)
      return(getKnownMax());

    // If tow is already in gut, you can decide policy:
    // - strict: return min (all decisions bad)
    // - recover: still evaluate u_tow and let it try to improve (often still flat)
    if(m_tow_breached)
      return(getKnownMin());

    double u_tow = m_obship_model_tow.evalHdgSpd(eval_crs, eval_spd);
    return(u_tow);
  }

  // Otherwise (not tow-only): original behavior
  double u_nav = m_obship_model.evalHdgSpd(eval_crs, eval_spd);

  if(!m_tow_eval || !m_tow_pose_set || !m_tow_model_ready)
    return(u_nav);

  if(m_tow_breached)
    return(getKnownMin());

  double u_tow = m_obship_model_tow.evalHdgSpd(eval_crs, eval_spd);
  return((u_tow < u_nav) ? u_tow : u_nav);
}
