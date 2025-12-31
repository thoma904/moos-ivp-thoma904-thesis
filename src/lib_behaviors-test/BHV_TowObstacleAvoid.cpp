/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT                                             */
/*    FILE: BHV_TowObstacleAvoid.cpp                        */
/*    DATE:                                                 */
/************************************************************/

#include <iostream>
#include <cmath> 
#include <cstdlib>
#include <algorithm> //added
#include "BHV_TowObstacleAvoid.h"
#include "AOF_TowObstacleAvoid.h"
#include "OF_Reflector.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "BuildUtils.h"
#include "MacroUtils.h"
#include "RefineryObAvoidV24.h"
#include "XYFormatUtilsPoly.h"
#include "VarDataPairUtils.h"
#include "XYPoint.h" //added

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_TowObstacleAvoid::BHV_TowObstacleAvoid(IvPDomain gdomain) :
  IvPBehavior(gdomain)
{
   this->setParam("descriptor", "towobsavoid");
  m_domain = subDomain(m_domain, "course,speed");

  // Initialize config vars
  m_pwt_grade = "linear";

  m_use_refinery     = false;
  m_resolved_pending = false;

  m_resolved_obstacle_var = "OBM_RESOLVED";
  m_draw_buff_min_poly = true;
  m_draw_buff_max_poly = true;
  
  // Initialize state vars
  m_obstacle_relevance = 0;

  m_valid_cn_obs_info = false;

  m_cpa_rng_sofar = -1;
  m_fpa_rng_sofar = -1;
  m_cpa_reported = -1;
  m_cpa_rng_ever = -1;
  m_closing = false;

  m_holonomic_ok = false;

  m_allstop_on_breach = true;

  //Tow Specific Additions
  
  // Tow integration
  m_use_tow      = false;
  m_tow_pad      = 0.0;
  m_tow_deployed = false;
  m_towed_x      = 0;
  m_towed_y      = 0;
  m_use_tow_cable = false;
  m_cable_sample_step = 1.0;
  m_tow_only = true;

  m_use_tow_lead = true;
  m_tow_lead_sec = 10.0;

  m_last_tow_x = 0;
  m_last_tow_y = 0;
  m_last_tow_time = -1;

  m_tow_x_eval = 0;
  m_tow_y_eval = 0;

  // Cached ranges
  m_rng_sys = -1;
  m_rng_nav = -1;
  m_rng_tow = -1;
  m_rng_src = "nav";
  m_rng_cable = -1;
  
  initVisualHints();
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING");
  addInfoVars("TOWED_X, TOWED_Y, TOW_DEPLOYED"); //added
  addInfoVars(m_resolved_obstacle_var);
}

//-----------------------------------------------------------
// Procedure: initVisualHints()

void BHV_TowObstacleAvoid::initVisualHints() 
{
  m_hints.setMeasure("vertex_size", 0);
  m_hints.setMeasure("edge_size", 1);
  m_hints.setColor("vertex_color", "gray50");
  m_hints.setColor("edge_color", "gray50");
  m_hints.setColor("fill_color", "off");
  m_hints.setColor("label_color", "white");

  m_hints.setColor("obst_edge_color", "white");
  m_hints.setColor("obst_vertex_color", "white");
  m_hints.setColor("obst_fill_color", "gray60");
  m_hints.setMeasure("obst_vertex_size", 1);
  m_hints.setMeasure("obst_fill_transparency", 0.7);
  
  m_hints.setColor("buff_min_edge_color", "gray60");
  m_hints.setColor("buff_min_vertex_color", "dodger_blue");
  m_hints.setColor("buff_min_fill_color", "gray70");
  m_hints.setColor("buff_min_label_color", "off");
  m_hints.setMeasure("buff_min_vertex_size", 1);
  m_hints.setMeasure("buff_min_fill_transparency", 0.25);
  
  m_hints.setColor("buff_max_edge_color", "gray60");
  m_hints.setColor("buff_max_vertex_color", "dodger_blue");
  m_hints.setColor("buff_max_fill_color", "gray70");
  m_hints.setColor("buff_max_label_color", "off");
  m_hints.setMeasure("buff_max_vertex_size", 1);
  m_hints.setMeasure("buff_max_fill_transparency", 0.1);
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_TowObstacleAvoid::setParam(string param, string val)
{
  if(IvPBehavior::setParam(param, val))
    return(true);

  double dval = atof(val.c_str());
  bool   non_neg_number = (isNumber(val) && (dval >= 0));

  string config_result;
  if((param=="polygon") || (param=="poly")) {
    config_result = m_obship_model.setGutPoly(val);
  }
  else if(param == "allowable_ttc")
    config_result = m_obship_model.setAllowableTTC(dval);
  else if((param == "min_util_cpa_dist") && non_neg_number)
    config_result = m_obship_model.setMinUtilCPA(dval);
  else if((param == "max_util_cpa_dist") && non_neg_number)
    config_result = m_obship_model.setMaxUtilCPA(dval);
  else if((param == "pwt_inner_dist") && non_neg_number)
    config_result = m_obship_model.setPwtInnerDist(dval);
  else if((param == "pwt_outer_dist") && non_neg_number)
    config_result = m_obship_model.setPwtOuterDist(dval);
  else if((param == "completed_dist") && non_neg_number) 
    config_result = m_obship_model.setCompletedDist(dval);

  else if(param == "holonomic_ok") 
    return(setBooleanOnString(m_holonomic_ok, val));
  else if(param == "draw_buff_min_poly") 
    return(setBooleanOnString(m_draw_buff_min_poly, val));
  else if(param == "draw_buff_max_poly") 
    return(setBooleanOnString(m_draw_buff_max_poly, val));

  //else if((param == "turn_radius") && non_neg_number)
  //  return(m_obship_model.setTurnModelRadius(dval));
  //else if((param == "turn_model_degs") && non_neg_number)
  //  return(m_obship_model.setTurnModelDegs(dval));

  else if(param == "can_disable")
    return(setBooleanOnString(m_can_disable, val));

  else if(param == "rng_flag")
    return(handleParamRangeFlag(val));
  else if(param == "cpa_flag")
    return(addFlagOnString(m_cpa_flags, val));
  else if(param == "visual_hints")
    return(m_hints.setHints(val));
  else if(param == "use_refinery")
    return(setBooleanOnString(m_use_refinery, val));
  else if((param == "id") || (param == "obid")) {
    // Once the obstacle id is set, it cannot be overwritten
    if((m_obstacle_id != "") && (m_obstacle_id != val))
      return(false);
    return(setNonWhiteVarOnString(m_obstacle_id, val));
  }
  //else if(param == "radius")
  //  return(m_tm_generator.setParam("radius", dval));
  //else if(param == "spoke_degs")
  //  return(m_tm_generator.setParam("spoke_degs", dval));

  else if(param == "allstop_on_breach") 
    return(setBooleanOnString(m_allstop_on_breach, val));

  //Tow Specific
  else if(param == "use_tow")
    return(setBooleanOnString(m_use_tow, val));

  else if((param == "tow_pad") && non_neg_number) {
    m_tow_pad = dval;
    return(true);
  }

  else if(param == "use_tow_cable")
    return(setBooleanOnString(m_use_tow_cable, val));

  else if((param == "cable_sample_step") && non_neg_number) {
    if(dval <= 0)
      return(false);
    m_cable_sample_step = dval;
    return(true);
  }

  else if(param == "tow_only")
    return(setBooleanOnString(m_tow_only, val));

  else
    return(false);

  if(config_result != "") {
    postBadConfig(config_result);
    return(false);
  }

  return(true);
}

//-----------------------------------------------------------
// Procedure: handleParamRangeFlag()
//   Example: rng_flag = <100 RNG_INFO = $[RNG]
//            rng_flag = RNG_INFO = range=$[RNG],speed=$[SPD]
//      Note: Whenever a range threshold is satisfied, flag is posted.
//            So it will be posted continuously when in range

bool BHV_TowObstacleAvoid::handleParamRangeFlag(string str)
{
  double thresh = -1;

  if(str.length() == 0)
    return(false);

  if(str[0] == '<') {
    biteString(str, '<');
    
    string rng_str = biteStringX(str, ' ');
    if(!isNumber(rng_str))
      return(false);
    thresh = atof(rng_str.c_str());
  }
  
  // Sanity checks on the value part
  bool ok = addFlagOnString(m_rng_flags, str);
  if(ok)
    m_rng_thresh.push_back(thresh);

  return(true);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_TowObstacleAvoid::onSetParamComplete()
{
  m_obship_model.setPlatModel(m_plat_model);
  m_obship_model.setCachedVals(true);
  postConfigStatus();
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_TowObstacleAvoid::onHelmStart()
{
  if(isDynamicallySpawnable() && (m_update_var != "")) {
    double pwt_outer_dist = m_obship_model.getPwtOuterDist();
    string alert_request = "name=" + m_descriptor;
    alert_request += ",update_var=" + m_update_var;
    alert_request += ",alert_range=" + doubleToStringX(pwt_outer_dist,1);
    postMessage("OBM_ALERT_REQUEST", alert_request);
  }
}

//-----------------------------------------------------------
// Procedure: onEveryState()

void BHV_TowObstacleAvoid::onEveryState(string str)
{
  // =================================================================
  // Part 1: Check for completion based on obstacle manager 
  // =================================================================
  // Get list of all obstacles declared to be resolved by obstacle mgr
  bool ok = true;
  vector<string> obstacles_resolved;
  obstacles_resolved = getBufferStringVector(m_resolved_obstacle_var, ok);
  
  // Check ids of all resolved obstacles against our own id. If match
  // then declare the resolution to be pending.
  for(unsigned int i=0; i<obstacles_resolved.size(); i++) {
    string obstacle_id = obstacles_resolved[i];
    postMessage("NOTED_RESOLVED", obstacle_id);

    if(m_obstacle_id == obstacle_id)
      m_resolved_pending = true;
  }


  // =================================================================
  // Part 2: Update the platform info and obship model
  // =================================================================
  if(!updatePlatformInfo())
    postWMessage("Invalid update of ownship position");

  // =================================================================
  // Part 2A: For now, until TurnModels come from the helm...
  // Use the TurnModelGenerator to generate a turn_model based on 
  // =================================================================

  m_obship_model.setPlatModel(m_plat_model);
  m_obship_model.setCachedVals();

  m_valid_cn_obs_info = true;
  if(!m_obship_model.isValid()) {
    m_valid_cn_obs_info = false;
  }
  if(!m_valid_cn_obs_info)
    postWMessage("Invalid update of ownship/obship model");
  
  if(m_obship_model.getFailedExpandPolyStr(false) != "") {
    string msg = m_obship_model.getFailedExpandPolyStr();
    postWMessage(msg);
  }
  if(!m_valid_cn_obs_info)
    return;

  //double os_range_to_poly = m_obship_model.getRange();
  
  // =================================================================
  // Tow Specific: Compute range based on tow status
  
  // Default: nav-only range
  double os_range_to_poly = m_obship_model.getRange();

  // Cache nav range
  m_rng_nav = os_range_to_poly;
  m_rng_tow = os_range_to_poly;
  m_rng_sys = os_range_to_poly;
  m_rng_src = "nav";
  m_tow_deployed = false;

  // If tow enabled, read tow state and compute system range = min(nav,tow)
  if(m_use_tow) {
    bool okx=true, oky=true;
    m_towed_x = getBufferDoubleVal("TOWED_X", okx);
    m_towed_y = getBufferDoubleVal("TOWED_Y", oky);

    // Robust TOW_DEPLOYED handling (string OR numeric)
    bool okd_str=false;
    string dep = tolower(getBufferStringVal("TOW_DEPLOYED", okd_str));
    if(okd_str)
      m_tow_deployed = (dep=="true" || dep=="yes" || dep=="1");
    else {
      bool okd_dbl=false;
      double dep_d = getBufferDoubleVal("TOW_DEPLOYED", okd_dbl);
      m_tow_deployed = (okd_dbl && (dep_d > 0.5));
    }

    if(okx && oky && m_tow_deployed) {

      m_tow_x_eval = m_towed_x;
      m_tow_y_eval = m_towed_y;

      double now = 0;
      if(m_info_buffer)
        now = m_info_buffer->getCurrTime();


      if(m_use_tow_lead && (m_tow_lead_sec > 0) && (m_last_tow_time > 0)) {
        double dt = now - m_last_tow_time;

        // sanity: avoid crazy dt on first iteration / pauses
        if((dt > 0.05) && (dt < 5.0)) {
          double vx = (m_towed_x - m_last_tow_x) / dt;
          double vy = (m_towed_y - m_last_tow_y) / dt;

          m_tow_x_eval = m_towed_x + vx * m_tow_lead_sec;
          m_tow_y_eval = m_towed_y + vy * m_tow_lead_sec;
        }
      }

      // update history
      m_last_tow_x = m_towed_x;
      m_last_tow_y = m_towed_y;
      m_last_tow_time = now;

      XYPolygon gut_poly = m_obship_model.getGutPoly();

      m_rng_tow = gut_poly.dist_to_poly(m_tow_x_eval, m_tow_y_eval);

      if(m_rng_tow < 0) m_rng_tow = 0;          // if dist_to_poly can go negative inside
      m_rng_tow = std::max(0.0, m_rng_tow - m_tow_pad);

      if(m_tow_only) {
        // Tow-only: ignore nav range completely
        m_rng_sys = m_rng_tow;
        m_rng_src = "tow";
        os_range_to_poly = m_rng_sys;
      } else {
        // Old behavior: min(nav,tow)
        m_rng_sys = std::min(m_rng_nav, m_rng_tow);
        m_rng_sys = std::max(0.0, m_rng_sys - m_tow_pad);
        m_rng_src = (m_rng_tow <= m_rng_nav) ? "tow" : "nav";
        os_range_to_poly = m_rng_sys;
      }
    }
  }
  // =================================================================

  if((m_cpa_rng_ever < 0) || (os_range_to_poly < m_cpa_rng_ever))
    m_cpa_rng_ever = os_range_to_poly;
  m_cpa_reported = m_cpa_rng_ever;

  // =================================================================
  // Part 3: Handle Range Flags if any
  // =================================================================
  if(m_rng_thresh.size() != m_rng_flags.size())
    postWMessage("Range flag mismatch");
  else {
    vector<VarDataPair> rng_flags;
    for(unsigned int i=0; i<m_rng_thresh.size(); i++) {
      double thresh = m_rng_thresh[i];
      if((thresh <= 0) || (os_range_to_poly < thresh))
	rng_flags.push_back(m_rng_flags[i]);
    }
    postFlags(rng_flags);
  }

  // =================================================================
  // Part 4: Handle CPA Flags if any, if a CPA event is observed
  // =================================================================
  // Part 4A: Check for CPA Event
  bool cpa_event = false;
  if((m_cpa_rng_sofar < 0) || (m_fpa_rng_sofar < 0)) {
    m_cpa_rng_sofar = os_range_to_poly;
    m_fpa_rng_sofar = os_range_to_poly;
  }
  
  if(m_closing) {
    if(os_range_to_poly < m_cpa_rng_sofar)
      m_cpa_rng_sofar = os_range_to_poly;
    if(os_range_to_poly > (m_cpa_rng_sofar + 1)) {
      m_closing = false;
      cpa_event = true;
      m_fpa_rng_sofar = os_range_to_poly;
    }
  }
  else {
    if(os_range_to_poly > m_fpa_rng_sofar)
      m_fpa_rng_sofar = os_range_to_poly;
    if(os_range_to_poly < (m_fpa_rng_sofar - 1)) {
      m_closing = true;
      m_cpa_rng_sofar = os_range_to_poly;
    }
  }

  // Part 4B: If CPA event observed, post CPA flags if any
  // NOTE: When posting CPA events, the $[CPA] macro will expand to the CPA
  //       value for this encounter. Otherwise $[CPA] is min CPA ever.
  if((cpa_event) && (os_range_to_poly < m_obship_model.getPwtOuterDist())) {
    m_cpa_reported = m_cpa_rng_sofar;
    postFlags(m_cpa_flags);
    m_cpa_reported = m_cpa_rng_ever;
  }
  
  // =================================================================
  // Part 5: Check for completion based on range
  // =================================================================
  if(os_range_to_poly > m_obship_model.getCompletedDist()) {
    m_resolved_pending = true;
  }

  if(!m_holonomic_ok) {
    if(m_plat_model.getModelType() == "holo")
      postWMessage("holo plat_model not best. Set holonomic_ok=true to silence");
  }

  XYPoint tow_act(m_towed_x, m_towed_y);
  tow_act.set_label("tow_act");
  tow_act.set_color("vertex", "yellow");
  tow_act.set_vertex_size(3);
  postMessage("VIEW_POINT", tow_act.get_spec());

  XYPoint tow_eval(m_tow_x_eval, m_tow_y_eval);
  tow_eval.set_label("tow_eval");
  tow_eval.set_color("vertex", "orange");
  tow_eval.set_vertex_size(3);
  postMessage("VIEW_POINT", tow_eval.get_spec());


}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_TowObstacleAvoid::onIdleState()
{
  postErasablePolygons();

  if(m_resolved_pending)
    setComplete();
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_TowObstacleAvoid::onIdleToRunState()
{
  postConfigStatus();
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_TowObstacleAvoid::onRunState()
{
 // Part 1: Handle if obstacle has been resolved
  if(m_resolved_pending) {
    setComplete();
    return(0);
  }
  if(!m_valid_cn_obs_info)
    return(0);

  m_obship_model.setCachedVals();

  /*// Part 2: No IvP function if obstacle is aft
  if(m_obship_model.isObstacleAft(20)) {
    return(0);
  }*/

  // =================================================================
  //Tow Specific: Keep running if obstacle aft
  if(m_obship_model.isObstacleAft(20)) {
    // If tow is what is close (not nav), we may still need to act
    if(!(m_use_tow && m_tow_deployed && (m_rng_src == "tow")))
      return(0);
  }
  // =================================================================
  
  // Part 3: Determine the relevance
  m_obstacle_relevance = getRelevance();
  if(m_obstacle_relevance <= 0)
    return(0);

  IvPFunction *ipf = buildOF();

  // Special case 1: No IPF built, due to ownship being within
  // the obstacle polygon. Likely want to invoke Allstop unless
  // allstop_on_breach has been explicitly set to false.
  if(!ipf) {
    if(m_allstop_on_breach)
      postEMessage("Allstop: Obstacle Breached");
    else
      postWMessage("Obstacle Breached");
    return(0);
  }
  
  // If IvP function has no decisions with positive utility then
  // this means a collision with an obstacle is unavoidable.
  // Possible when using plat model with non-zero turn radius.
  if(ipf->getValMaxUtil() == 0) {
    postEMessage("Allstop: obstacle unavoidable");
    delete(ipf);
    return(0);
  }

  return(ipf);
}

//-----------------------------------------------------------
// Procedure: buildOF()

IvPFunction *BHV_TowObstacleAvoid::buildOF()
{
  AOF_TowObstacleAvoid aof_avoid(m_domain);
  aof_avoid.setObShipModel(m_obship_model);

  // ---------------------------------------------------------
  // NEW: Pass tow info into the AOF so the IPF considers tow CPA
  // ---------------------------------------------------------
  if(m_use_tow && m_tow_deployed) {
    aof_avoid.setTowEval(true);
    aof_avoid.setTowPose(m_tow_x_eval, m_tow_y_eval, 0.0);
    aof_avoid.setTowOnly(m_tow_only);
  } else {
    aof_avoid.setTowEval(false);
    aof_avoid.setTowOnly(false);
  }
  // ---------------------------------------------------------

  bool ok_init = aof_avoid.initialize();
  if(!ok_init) {
    string aof_msg = aof_avoid.getCatMsgsAOF();
    postWMessage("Unable to init AOF_TowObstacleAvoid:" + aof_msg);
    return(0);
  }

  OF_Reflector reflector(&aof_avoid, 1);

  if(m_use_refinery) {
    RefineryObAvoidV24 refinery(m_domain);
    refinery.setSideLock(m_side_lock);
    refinery.setRefineRegions(m_obship_model);

    vector<IvPBox> plateau_regions = refinery.getPlateaus();
    vector<IvPBox> basin_regions   = refinery.getBasins();

    for(unsigned int i=0; i<plateau_regions.size(); i++)
      reflector.setParam("plateau_region", plateau_regions[i]);
    for(unsigned int i=0; i<basin_regions.size(); i++)
      reflector.setParam("basin_region", basin_regions[i]);
  }

  if(m_build_info != "")
    reflector.create(m_build_info);
  else {
    reflector.setParam("uniform_piece", "discrete@course:3,speed:3");
    reflector.setParam("uniform_grid",  "discrete@course:9,speed:9");
    reflector.create();
  }

  if(!reflector.stateOK()) {
    postWMessage(reflector.getWarnings());
    return(0);
  }

  IvPFunction *ipf = reflector.extractIvPFunction(true);
  if(ipf) {
    ipf->setPWT(m_obstacle_relevance * m_priority_wt);
    postViewablePolygons();
  }

  return(ipf);
}

//-----------------------------------------------------------
// Procedure: getRelevance()
//            Calculate the relevance first. If zero-relevance, 
//            we won't bother to create the objective function.

double BHV_TowObstacleAvoid::getRelevance()
{
  // Let the ObShipModel tell us the raw range relevance
  //double range_relevance = m_obship_model.getRangeRelevance();

  // =================================================================
  // Tow Specific

  // Tow-only mode: if tow isn't deployed/valid, this behavior should be irrelevant.
  if(m_tow_only) {
    if(!(m_use_tow && m_tow_deployed && (m_rng_sys >= 0)))
      return(0);
  }

  double range_relevance = m_obship_model.getRangeRelevance();

  // If tow is deployed, base relevance on the tow-aware system range
  if(m_use_tow && m_tow_deployed && (m_rng_sys >= 0))
    range_relevance = computeRangeRelevanceFromRange(m_rng_sys);
  // =================================================================

  if(range_relevance <= 0)
    return(0);

  if(range_relevance > 0.6) {
    if(m_side_lock == "") {
      if(m_obship_model.getPassingSide() == "star")
	m_side_lock = "port";
      else if(m_obship_model.getPassingSide() == "port")
	m_side_lock = "star";
      else 
	m_side_lock = "";
    }
  }
  else
    m_side_lock = "";
  
  // Part 2: Possibly apply the grade scale to the raw distance
  double relevance = range_relevance;
  if(m_pwt_grade == "quadratic")
    relevance = range_relevance * range_relevance;
  else if(m_pwt_grade == "quasi")
    relevance = pow(range_relevance, 1.5);

  return(relevance);
}

//-----------------------------------------------------------
// Procedure: postViewablePolygons()

void BHV_TowObstacleAvoid::postViewablePolygons()
{
  // =================================================
  // Part 1 - Render the gut (physical) polygon
  // =================================================
  XYPolygon gut_poly = m_obship_model.getGutPoly();
  gut_poly.set_active(true);
  // If the obstacle is relevant, perhaps draw filled in
  if(m_obstacle_relevance > 0) {
    if(m_side_lock != "")
      m_hints.setColor("gut_fill_color", "pink");
    else
      m_hints.setColor("gut_fill_color", "gray60");
  }
  else
    m_hints.setColor("gut_fill_color", "off");

  applyHints(gut_poly, m_hints, "gut");
  postMessage("VIEW_POLYGON", gut_poly.get_spec(5), "gut");
    
  // =================================================
  // Part 2 - Render the mid polygon
  // =================================================
  XYPolygon mid_poly = m_obship_model.getMidPoly();
  mid_poly.set_active(true);
  // If the obstacle is relevant, perhaps draw filled in
  if(m_obstacle_relevance > 0)
    m_hints.setColor("mid_fill_color", "gray70");
  else
    m_hints.setColor("mid_fill_color", "off");

  applyHints(mid_poly, m_hints, "mid");
  postMessage("VIEW_POLYGON", mid_poly.get_spec(5), "mid");

  // =================================================
  // Part 3 - Render the rim (outermost) polygon
  // =================================================
  XYPolygon rim_poly = m_obship_model.getRimPoly();
  rim_poly.set_active(true);
  
  // If the obstacle is relevant, perhaps draw filled in
  if(m_obstacle_relevance > 0)
    m_hints.setColor("rim_fill_color", "gray70");
  else
    m_hints.setColor("rim_fill_color", "off");

  applyHints(rim_poly, m_hints, "rim");
  postMessage("VIEW_POLYGON", rim_poly.get_spec(5), "rim");
}


//-----------------------------------------------------------
// Procedure: postErasablePolygons()

void BHV_TowObstacleAvoid::postErasablePolygons()
{
  XYPolygon gut_poly = m_obship_model.getGutPoly();
  postMessage("VIEW_POLYGON", gut_poly.get_spec_inactive(), "gut");

  XYPolygon mid_poly = m_obship_model.getMidPoly();
  postMessage("VIEW_POLYGON", mid_poly.get_spec_inactive(), "mid");

  XYPolygon rim_poly = m_obship_model.getRimPoly();
  rim_poly.set_color("fill", "invisible");
  postMessage("VIEW_POLYGON", rim_poly.get_spec_inactive(), "rim");

  // =================================================
  // Tow Specific: Erase tow point
  
  // Erase tow point
  XYPoint towpt(0,0);
  towpt.set_label("tow_" + m_descriptor);
  postMessage("VIEW_POINT", towpt.get_spec_inactive(), "tow");
  
  // =================================================
  
}

//-----------------------------------------------------------
// Procedure: updatePlatformInfo()

bool BHV_TowObstacleAvoid::updatePlatformInfo()
{
  return(true);

  // Note: This behavior obtains the osx,osy,osh information
  // through the platform model. The platform model is
  // maintained and updated by the helm with osx,osy,osh,osv
  // nav info. The ObShipModel used in this behavior has direct
  // access to the platform model.

#if 0    
    bool ok_update = m_obship_model.setPose(m_osx, m_osy, m_osh);
  if(!ok_update) 
    warning_msg = "Problem updating obship_model pose";
  if(!m_obship_model.getObstacle().is_convex()) 
    warning_msg = "Non-convex Obstacle";
  if(!m_obship_model.getObstacleBuffMin().is_convex()) 
    warning_msg = "Non-convex ObstacleBuffMin";
  if(!m_obship_model.getObstacleBuffMax().is_convex()) 
    warning_msg = "Non-convex ObstacleBuffMax";
#endif
    
  return(true);

}

//-----------------------------------------------------------
// Procedure: postConfigStatus()

void BHV_TowObstacleAvoid::postConfigStatus()
{
  string str = "type=BHV_AvoidObstacleV24,name=" + m_descriptor;

  double pwt_inner_dist = m_obship_model.getPwtInnerDist();
  double pwt_outer_dist = m_obship_model.getPwtOuterDist();
  double completed_dist = m_obship_model.getCompletedDist();
  double min_util_cpa   = m_obship_model.getMinUtilCPA();
  double max_util_cpa   = m_obship_model.getMaxUtilCPA();
  double allowable_ttc  = m_obship_model.getAllowableTTC();
  
  str += ",allowable_ttc="  + doubleToString(allowable_ttc,2);
  str += ",min_util_cpa="   + doubleToString(min_util_cpa,2);
  str += ",max_util_cpa="   + doubleToString(max_util_cpa,2);
  str += ",pwt_outer_dist=" + doubleToString(pwt_outer_dist,2);
  str += ",pwt_inner_dist=" + doubleToString(pwt_inner_dist,2);
  str += ",completed_dist=" + doubleToString(completed_dist,2);

  postRepeatableMessage("BHV_SETTINGS", str);
}

//-----------------------------------------------------------
// Procedure: getDoubleInfo()

double BHV_TowObstacleAvoid::getDoubleInfo(string str)
{
  if(str == "osx")
    return(m_obship_model.getOSX());
  else if(str == "osy")
    return(m_obship_model.getOSY());
  else if(str == "osh")
    return(m_obship_model.getOSH());
  else if(str == "allowable_ttc")
    return(m_obship_model.getAllowableTTC());
  else if(str == "pwt_outer_dist")
    return(m_obship_model.getPwtOuterDist());
  else if(str == "pwt_inner_dist")
    return(m_obship_model.getPwtInnerDist());
  else if(str == "completed_dist")
    return(m_obship_model.getCompletedDist());
  else if(str == "min_util_cpa")
    return(m_obship_model.getMinUtilCPA());
  else if(str == "max_util_cpa")
    return(m_obship_model.getMaxUtilCPA());

  return(0);
}

//-----------------------------------------------------------
// Procedure: expandMacros()

string BHV_TowObstacleAvoid::expandMacros(string sdata)
{
  // =======================================================
  // First expand the macros defined at the superclass level
  // =======================================================
  sdata = IvPBehavior::expandMacros(sdata);

  // =======================================================
  // Then expand the macros unique to this behavior
  // =======================================================
  /*if(strContains(sdata, "$[RNG]"))
    sdata = macroExpand(sdata, "RNG", m_obship_model.getRange());*/

  // =======================================================
  // Tow Specific

    double rng = m_obship_model.getRange();
  if(m_use_tow && m_tow_deployed && (m_rng_sys >= 0))
    rng = m_rng_sys;

  if(strContains(sdata, "$[RNG]"))
    sdata = macroExpand(sdata, "RNG", rng);

  // =======================================================
    
  if(strContains(sdata, "$[BNG]"))
    sdata = macroExpand(sdata, "BNG", m_obship_model.getObcentBng());

  if(strContains(sdata, "$[RBNG]"))
    sdata = macroExpand(sdata, "BNG", m_obship_model.getObcentRelBng());

  if(strContains(sdata, "$[SIDE]"))
    sdata = macroExpand(sdata, "SIDE", m_obship_model.getPassingSide());
  
  string obs_id = m_obship_model.getObstacleLabel();

  sdata = macroExpand(sdata, "OID", obs_id);
  sdata = macroExpand(sdata, "CPA", m_cpa_reported);
  sdata = macroExpand(sdata, "SLOCK", m_side_lock);

  // =======================================================
  // Added Oct1724
  // =======================================================
  // Extract TARGETIDs of the form TYPE_ARPANUM_TARGETID 
  // If string has no underscore, then obs_idx is empty string.
  string obs_idx = "";
  if(strContains(obs_id, '_')) {
    string tmp_obs_id = obs_id;
    obs_idx = rbiteString(tmp_obs_id, '_');
  }
  sdata = macroExpand(sdata, "OIDX", obs_idx);

  double min_util_cpa = m_obship_model.getMinUtilCPA();
  double max_util_cpa = m_obship_model.getMaxUtilCPA();
  sdata = macroExpand(sdata, "MINU_CPA", min_util_cpa);
  sdata = macroExpand(sdata, "MAXU_CPA", max_util_cpa);
  
  return(sdata);
}


//-----------------------------------------------------------
// Procedure: applyAbleFilter()
//  Examples: action=disable, obstacle_id=345
//            action=enable,  obstacle_id=345
//            action=disable, vsource=radar
//            action=expunge, obstacle_id=345
//    Fields: action=disable/enable/expunge  (mandatory)
//            contact=345          (one of these four)
//            gen_type=safety
//            bhv_type=AvdColregs
//            vsource=ais

bool BHV_TowObstacleAvoid::applyAbleFilter(string str)
{
  // If this behavior is configured to be immune to disable
  // actions, then just ignore and return true, even if the
  // passed argument is not proper.
  if(!m_can_disable)
    return(true);

  // ======================================================
  // Part 1: Parse the filter string. Must be one of the
  //         supported fields, no field more than once.
  // ======================================================
  string action, obid, vsource;
  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = tolower(biteStringX(svector[i], '='));
    string value = tolower(svector[i]);

    if((param == "action") && (action == ""))
      action = value;
    else if((param == "obstacle_id") && (obid == ""))
      obid = value;
    else if((param == "vsource") && (vsource == ""))
      vsource = value;
    else
      return(false);
  }

  // ======================================================
  // Part 2: Check for proper format.
  // ======================================================
  // action must be specified and only disable or enable
  if((action != "disable") && (action != "enable") &&
     (action != "expunge"))
    return(false);

  // At least one of obid or vsource must be given
  if((obid == "") && (vsource == ""))
    return(false);

  // ======================================================
  // Part 3: At this point syntax checking has passed, now
  // check if this filter message applies to this behavior.
  // ======================================================

  // Check 3: If obstacle_id has been set then MUST 
  // match, regardless of other filter factors
  if(obid != "") {
    if(tolower(obid) != tolower(m_obstacle_id))
      return(true);  // Return true since syntax if fine
  }
  // Check 4: If obstacle vsource has been set then MUST 
  // match, regardless of other filter factors
  else if(vsource != "") {
    cout << "vsource:" << vsource << endl;
    string poly_vsource = m_obship_model.getVSource();
    cout << "poly_vsource" << poly_vsource << endl;
    if(tolower(vsource) != tolower(poly_vsource))
      return(true); // Return true since syntax if fine
  }
  else 
    return(false); // No criteria given (obid or vsource)

  if(action == "disable")
    m_disabled = true;
  else
    m_disabled = false;

  if(action == "expunge")
    setComplete();
  
  return(true);
}

double BHV_TowObstacleAvoid::computeRangeRelevanceFromRange(double range) const
{
  double inner = m_obship_model.getPwtInnerDist();
  double outer = m_obship_model.getPwtOuterDist();

  if(outer <= inner)
    outer = inner + 0.001;

  if(range >= outer)
    return(0);

  if(range <= inner)
    return(1);

  return((outer - range) / (outer - inner));  // linear ramp
}