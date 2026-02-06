/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT                                             */
/*    FILE: BHV_TowObstacleAvoid.cpp                        */
/*    DATE:                                                 */
/*                                                          */
/* NOTE: This behavior is derived from MOOS-IvP             */
/* BHV_AvoidObstacleV24. Modifications add tow-aware        */
/* obstacle avoidance instead of the original vessel logic. */
/* Tow “truth” range (m_rng_tow_actual) is used for         */
/* completion, while an optional lead point                 */
/* (m_tow_x_eval/m_tow_y_eval) is used for relevance/range. */
/************************************************************/

#include <iostream>
#include <cmath> 
#include <cstdlib>
#include <algorithm>
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
#include "XYPoint.h"

using namespace std;

//---------------------------------------------------------------
// Constructor()
// Note: Most obstacle geometry/state lives in m_obship_model.
//       Tow-related state is tracked in member variables. Tow pose/velocity
//       are refreshed from InfoBuffer (TOWED_X/Y, TOWED_VX/VY) each iteration,
//       while m_rng_tow_actual is computed internally from the tow position.


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

  m_tow_pad      = 0.0;
  m_towed_x      = 0;
  m_towed_y      = 0;

  m_use_tow_lead = true;
  m_tow_lead_sec = 6.0;

  m_last_tow_x = 0;
  m_last_tow_y = 0;
  m_last_tow_time = -1;

  m_tow_x_eval = 0;
  m_tow_y_eval = 0;

  m_towed_vx = 0;
  m_towed_vy = 0;
  m_towed_vel_valid = false;
  m_tow_pose_valid = false;

  // Cached ranges
  m_rng_sys = -1;
  m_rng_nav = -1;
  m_rng_tow = -1;
  m_rng_src = "nav";

  m_rng_tow_actual = -1;

  //attempt to fix pred jumping
  m_tow_vx_filt = 0;
  m_tow_vy_filt = 0;
  m_tow_vel_valid = false;

  m_tow_pose_stale = 1.0;    // seconds: consider tow stale if older than this
  m_tow_lead_alpha = 0.3;    // 0..1 low-pass filter aggressiveness
  m_tow_lead_max_speed = 3.0; // m/s cap
  m_tow_xy_sync_eps  = 0.10;

  // Tow dynamics defaults (overwritten each iteration by pTowing publications)
  m_cable_length   = 30;
  m_attach_offset  = 0;
  m_k_spring       = 5;
  m_cd             = 0.7;
  m_c_tan          = 2.0;

  m_sim_dt         = 0.1;
  m_sim_horizon    = -1;
  m_turn_rate_max  = 15.0;

  initVisualHints();
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING");
  addInfoVars("TOWED_X, TOWED_Y", "no_warning");
  addInfoVars("TOWED_VX, TOWED_VY", "no_warning");
  addInfoVars("TOW_CABLE_LENGTH, TOW_ATTACH_OFFSET", "no_warning");
  addInfoVars("TOW_SPRING_STIFFNESS, TOW_DRAG_COEFF, TOW_TAN_DAMPING", "no_warning");
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
// Tow-specific configuration:
//  - tow_pad: subtract from computed tow-to-obstacle range
// Note: This behavior is always tow-based.

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

  else if((param == "tow_pad") && non_neg_number) {
    m_tow_pad = dval;
    return(true);
  }

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
  
  // =================================================================
  // Part 2B: Tow integration (tow-based behavior)
  //   - m_tow_pose_valid: true when BOTH TOWED_X and TOWED_Y are available this iteration
  //   - m_rng_tow: range from tow eval/lead point to obstacle polygon (used for relevance/flags)
  //   - m_rng_tow_actual: range from actual tow pose to obstacle polygon (used for completion)
  //   - During tow dropout, no range flags/CPA are evaluated
  // =================================================================
  // Tow Specific: Compute range based on tow status
  
  // Initialize NAV range for diagnostics only; behavioral logic is gated on tow_sys_valid
  double os_range_to_poly = m_obship_model.getRange();

  // Diagnostic only. When tow pose is valid, m_rng_sys/m_rng_src are overwritten with tow-based values.
  m_rng_nav = os_range_to_poly;
  m_rng_tow = os_range_to_poly;
  m_rng_sys = os_range_to_poly;
  m_rng_src = "nav";
  m_rng_tow_actual = -1;

  // m_rng_tow_actual is reset to -1 so completion cannot use stale tow truth when tow pose is missing this iteration.

  m_tow_pose_valid   = false;
  m_towed_vel_valid  = false;
  m_towed_vx         = 0;
  m_towed_vy         = 0;

  // Read tow state and compute tow ranges.
  // When tow pose is valid, system range is tow eval/lead range (m_rng_sys = m_rng_tow).
    
  bool okx=true, oky=true;
  double tx = getBufferDoubleVal("TOWED_X", okx);
  double ty = getBufferDoubleVal("TOWED_Y", oky);
  if(okx) m_towed_x = tx;
  if(oky) m_towed_y = ty;

  m_tow_pose_valid = (okx && oky);

  // Tow pose is “valid” only when BOTH X and Y are present in the InfoBuffer.
  // If only one updates, we treat tow pose as invalid for this iteration to avoid
  // mixing new X with old Y in downstream calculations.

  bool okvx=true, okvy=true;
  m_towed_vx = getBufferDoubleVal("TOWED_VX", okvx);
  m_towed_vy = getBufferDoubleVal("TOWED_VY", okvy);
  m_towed_vel_valid = (okvx && okvy);

  // Read tow dynamics parameters published by pTowing.
  // These update the member variables used by the AOF; defaults are
  // kept if pTowing has not yet published.
  bool ok_tmp = false;
  double tmp_val = 0;

  tmp_val = getBufferDoubleVal("TOW_CABLE_LENGTH", ok_tmp);
  if(ok_tmp) m_cable_length = tmp_val;

  tmp_val = getBufferDoubleVal("TOW_ATTACH_OFFSET", ok_tmp);
  if(ok_tmp) m_attach_offset = tmp_val;

  tmp_val = getBufferDoubleVal("TOW_SPRING_STIFFNESS", ok_tmp);
  if(ok_tmp) m_k_spring = tmp_val;

  tmp_val = getBufferDoubleVal("TOW_DRAG_COEFF", ok_tmp);
  if(ok_tmp) m_cd = tmp_val;

  tmp_val = getBufferDoubleVal("TOW_TAN_DAMPING", ok_tmp);
  if(ok_tmp) m_c_tan = tmp_val;

  // Tow velocity is optional.
  // If valid, it is used for (1) lead-point prediction and (2) tow-heading inference
  // for refinery passing-side calculations.

  if(m_tow_pose_valid) 
  {
    // Part 2B.1: Robust tow lead-point estimation (optional)
    //   - Uses InfoBuffer timestamps when available (tQuery) to avoid “jumping”
    //   - Low-pass filters velocity and caps speed to reject spikes
    //   - Lead point defaults to the actual tow pose when velocity is invalid

    // Always start with "no-lead" eval at the actual tow pose
    m_tow_x_eval = m_towed_x;
    m_tow_y_eval = m_towed_y;

    double now = m_curr_time;
    if(m_info_buffer)
      now = m_info_buffer->getCurrTime();

    // Try to get message timestamps for X/Y so we only compute velocity on fresh samples.
    bool   ok_tx_t = false, ok_ty_t = false;
    double tx_t = 0, ty_t = 0;
    double pose_time = now;
    double newest_t  = now;

    if(m_info_buffer) 
    {
      tx_t = m_info_buffer->tQuery("TOWED_X", ok_tx_t);
      ty_t = m_info_buffer->tQuery("TOWED_Y", ok_ty_t);

      if(ok_tx_t && ok_ty_t) 
      {
        newest_t  = std::max(tx_t, ty_t);
        pose_time = newest_t;  // treat pose time as the newer of (x,y)
      }
      else if(ok_tx_t) 
      {
        newest_t = pose_time = tx_t;
      }
      else if(ok_ty_t) 
      {
        newest_t = pose_time = ty_t;
      }
    }

    // If the tow pose hasn't updated recently, disable velocity projection
    bool pose_stale = ((now - newest_t) > m_tow_pose_stale);
    if(pose_stale) 
    {
      m_tow_vel_valid = false;
    }
    else if(m_use_tow_lead && (m_tow_lead_sec > 0)) 
    {
      // If we have both timestamps, require them to be "paired" (avoid mixing new X with old Y)
      bool pose_synced = true;
      if(ok_tx_t && ok_ty_t) 
      {
        pose_synced = (fabs(tx_t - ty_t) <= m_tow_xy_sync_eps);
      }

      // Determine whether this is a *new* pose sample
      bool new_sample = false;
      if(m_last_tow_time < 0) 
      {
        new_sample = true;
      }
      else if(ok_tx_t && ok_ty_t) 
      {
        // Only treat as new when BOTH components are newer than last accepted pose
        new_sample = pose_synced && (tx_t > (m_last_tow_time + 1e-6)) && (ty_t > (m_last_tow_time + 1e-6));
      }
      else 
      {
        // If no timestamps available, fall back to time-based updates (less ideal)
        new_sample = (pose_time > (m_last_tow_time + 0.05));
      }
      if(new_sample && (m_last_tow_time > 0)) 
      {
        double dt = pose_time - m_last_tow_time;

        // Guard dt
        if((dt > 0.05) && (dt < 2.0)) 
        {
          double dx = (m_towed_x - m_last_tow_x);
          double dy = (m_towed_y - m_last_tow_y);

          double inst_vx  = dx / dt;
          double inst_vy  = dy / dt;
          double inst_spd = hypot(inst_vx, inst_vy);

          // Reject teleport spikes: don't update velocity on implausible speed
          if(inst_spd <= m_tow_lead_max_speed) 
          {
            // Low-pass filter velocity (reduces jitter)
            if(!m_tow_vel_valid) 
            {
              m_tow_vx_filt   = inst_vx;
              m_tow_vy_filt   = inst_vy;
              m_tow_vel_valid = true;
            }
            else 
            {
              m_tow_vx_filt = m_tow_lead_alpha * inst_vx + (1.0 - m_tow_lead_alpha) * m_tow_vx_filt;
              m_tow_vy_filt = m_tow_lead_alpha * inst_vy + (1.0 - m_tow_lead_alpha) * m_tow_vy_filt;
            }

            // Cap filtered speed too
            double filt_spd = hypot(m_tow_vx_filt, m_tow_vy_filt);
            if(filt_spd > m_tow_lead_max_speed) 
            {
              double s = m_tow_lead_max_speed / filt_spd;
              m_tow_vx_filt *= s;
              m_tow_vy_filt *= s;
            }

            // Accept this sample as the new history anchor (ONLY when it passed checks)
            m_last_tow_x    = m_towed_x;
            m_last_tow_y    = m_towed_y;
            m_last_tow_time = pose_time;
          }
          else 
          {
            // Bad sample -> drop velocity, but DO NOT move the history anchor.
            // This makes you recover immediately when good data returns.
            m_tow_vel_valid = false;
          }
        }
        else 
        {
          // dt not usable -> drop velocity
          m_tow_vel_valid = false;
        }
      }
      else if(new_sample && (m_last_tow_time < 0)) 
      {
        // First-ever accepted anchor
        m_last_tow_x    = m_towed_x;
        m_last_tow_y    = m_towed_y;
        m_last_tow_time = pose_time;
        m_tow_vel_valid = false;  // need at least two samples to compute velocity
      }
        
      // Finally: compute eval point using filtered velocity if valid
      if(m_tow_vel_valid) 
      {
        m_tow_x_eval = m_towed_x + m_tow_vx_filt * m_tow_lead_sec;
        m_tow_y_eval = m_towed_y + m_tow_vy_filt * m_tow_lead_sec;
      }
    }

    XYPolygon gut_poly = m_obship_model.getGutPoly();

    //----------------------------------------------------
    // Tow truth range (actual tow pose) used for completion
    //----------------------------------------------------

    double tow_rng_actual = gut_poly.dist_to_poly(m_towed_x, m_towed_y);
    if(tow_rng_actual < 0) tow_rng_actual = 0;
    tow_rng_actual = std::max(0.0, tow_rng_actual - m_tow_pad);

    m_rng_tow_actual = tow_rng_actual;

    m_rng_tow = gut_poly.dist_to_poly(m_tow_x_eval, m_tow_y_eval);
    if(m_rng_tow < 0) m_rng_tow = 0;
    m_rng_tow = std::max(0.0, m_rng_tow - m_tow_pad);

    // Truth range (m_rng_tow_actual): actual tow pose to polygon (used for completion)
    // Eval range  (m_rng_tow): lead/eval tow pose to polygon (used for relevance/system range)

    m_rng_sys = m_rng_tow;
    m_rng_src = "tow";
    os_range_to_poly = m_rng_sys;
  }

  // Strict tow-only gate: only treat range as valid when it is tow-derived this iteration.
  bool tow_sys_valid = (m_tow_pose_valid && (m_rng_src == "tow") && (m_rng_sys >= 0));

  if(tow_sys_valid) 
  {
    if((m_cpa_rng_ever < 0) || (os_range_to_poly < m_cpa_rng_ever))
      m_cpa_rng_ever = os_range_to_poly;
    m_cpa_reported = m_cpa_rng_ever;
  } 
  else 
  {
    // Strict tow-only: prevent NAV fallback from polluting CPA memory/state.
    // Also reset the CPA tracker so we don't trigger a false CPA event when tow returns.
    m_cpa_rng_sofar = -1;
    m_fpa_rng_sofar = -1;
    m_closing       = false;

    // Optional: make CPA unknown during dropout (useful for macros/diagnostics)
    // m_cpa_reported  = -1;
  }


  // =================================================================
  // Part 3: Handle Range Flags if any
  // =================================================================
  if(m_rng_thresh.size() != m_rng_flags.size()) {
  postWMessage("Range flag mismatch");
  }
  else if(tow_sys_valid) {
    vector<VarDataPair> rng_flags;
    for(unsigned int i=0; i<m_rng_thresh.size(); i++) {
      double thresh = m_rng_thresh[i];
      if((thresh <= 0) || (os_range_to_poly < thresh))
        rng_flags.push_back(m_rng_flags[i]);
    }
    postFlags(rng_flags);
  }
  // else: strict tow-only -> do not post range flags on NAV fallback

  // =================================================================
  // Part 4: Handle CPA Flags if any, if a CPA event is observed
  // =================================================================
  // Part 4A: Check for CPA Event
  if(tow_sys_valid) 
  {
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
    if((cpa_event) && (os_range_to_poly < m_obship_model.getPwtOuterDist())) {
      m_cpa_reported = m_cpa_rng_sofar;
      postFlags(m_cpa_flags);
      m_cpa_reported = m_cpa_rng_ever;
    }
  }
  // else: strict tow-only -> no CPA tracking/events when tow pose is missing


  // =================================================================
  // Part 5: Completion based on ACTUAL tow (truth), not eval point
  // =================================================================
  // Completion uses tow TRUTH range when tow pose is available.
  // If tow pose is not valid, completion is not evaluated (tow-only semantics).

  double complete_rng = -1;

  if(m_tow_pose_valid && (m_rng_tow_actual >= 0)) 
    complete_rng = m_rng_tow_actual;

  if(complete_rng > m_obship_model.getCompletedDist()) 
  {
    m_resolved_pending = true;
  }

  if(!m_holonomic_ok) 
  {
    if(m_plat_model.getModelType() == "holo")
      postWMessage("holo plat_model not best. Set holonomic_ok=true to silence");
  }

  // Visualization: show both the tow actual pose and the tow eval/lead pose.

  if(m_tow_pose_valid) 
  {
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
  else {
    // Clear/disable points so stale markers don't linger during tow dropouts
    XYPoint tow_act(0, 0);
    tow_act.set_label("tow_act");
    tow_act.set_active(false);
    postMessage("VIEW_POINT", tow_act.get_spec());

    XYPoint tow_eval(0, 0);
    tow_eval.set_label("tow_eval");
    tow_eval.set_active(false);
    postMessage("VIEW_POINT", tow_eval.get_spec());
  }

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
  if(m_resolved_pending) { setComplete(); return 0; }
  if(!m_valid_cn_obs_info) return 0;

  m_obship_model.setCachedVals();

  // Part 2: If the obstacle is aft of the NAV-based ownship pose, we normally skip avoidance.
  // But in tow-based operation, the tow may be in a different location, so we allow
  // avoidance whenever tow pose is valid and rng_src=="tow".

  if(m_obship_model.isObstacleAft(20)) {
    if(!(m_tow_pose_valid && (m_rng_src=="tow")))
      return 0;
  }

  m_obstacle_relevance = getRelevance();
  if(m_obstacle_relevance <= 0)
    return 0;
  
  IvPFunction* ipf = buildOF();

  if(!ipf) 
  {
    // Tow-only: suppress breach allstop messaging
    return(0);
  }

  if(ipf->getValMaxUtil() == 0) 
  {
    // Tow-only: suppress unavoidable allstop messaging
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

  // Part 1A: Tow-aware AOF configuration
  // If tow pose is valid, evaluate CPA/risk using the tow state as the modeled body.
  // Defensive fallback only: in tow-only behavior, relevance gating should prevent
  // reaching this branch unless tow pose is unavailable.

  if(m_tow_pose_valid) 
  {
    aof_avoid.setTowEval(true);
    aof_avoid.setTowOnly(true);  

    bool okvx=true, okvy=true;
    double towed_vx = getBufferDoubleVal("TOWED_VX", okvx);
    double towed_vy = getBufferDoubleVal("TOWED_VY", okvy);

    // Fallback: if tow velocity is unavailable, assume zero velocity.
    if(!okvx || !okvy) {
      towed_vx = 0;
      towed_vy = 0;
    }

    // Use ACTUAL tow state as initial condition (not the lead point)
    aof_avoid.setTowState(m_towed_x, m_towed_y, towed_vx, towed_vy);

    // Pass tow dynamics parameters to the AOF.
    aof_avoid.setTowDynParams(m_cable_length, m_attach_offset,
                            m_k_spring, m_cd, m_c_tan);

    // Optional: match simulation dt to pTowing AppTick (e.g., 0.1 for 10 Hz)
    aof_avoid.setSimParams(0.1, -1 /*use allowable_ttc*/);

    aof_avoid.setTowSpeedPenalty(true);
    aof_avoid.setTowSpeedMin(0.5);        // start penalizing below 0.5 m/s tow speed
    aof_avoid.setTowSpeedHardMin(0.1);    // optional: forbid <0.1 m/s tow speed
    aof_avoid.setTowSpeedPenaltyPower(2); // quadratic
    aof_avoid.setTowSpeedPenaltyFloor(0); // can set e.g. 0.05 to avoid "all flats" if needed
  } 
  
  else {
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

  // Refinery: optionally refine regions using tow pose/heading as the effective “ownship”
  // so plateau/basin regions reflect tow-based clearance geometry.

  if(m_use_refinery) {
    RefineryObAvoidV24 refinery(m_domain);
    refinery.setSideLock(m_side_lock);

    ObShipModelV24 refine_model = m_obship_model;

    // Use tow pose/heading for refinement
    if(m_tow_pose_valid) 
    {
      // Compute a tow heading from tow velocity if available
      double tow_hdg = m_obship_model.getOSH(); // fallback (nav heading)
      bool okvx=true, okvy=true;
      double towed_vx = getBufferDoubleVal("TOWED_VX", okvx);
      double towed_vy = getBufferDoubleVal("TOWED_VY", okvy);

      if(okvx && okvy) 
      {
        double spd = hypot(towed_vx, towed_vy);
        if(spd > 1e-6)
          tow_hdg = relAng(0, 0, towed_vx, towed_vy);  // MOOS heading convention
      }

      // Override what ObShipModel considers "ownship"
      refine_model.setPose(m_towed_x, m_towed_y, tow_hdg);

      // Refresh cached values after overriding pose.
      refine_model.setCachedVals(true);
    }

    refinery.setRefineRegions(refine_model);


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
  // Relevance is computed from the tow-aware system range (m_rng_sys).
  // If tow pose is missing, this behavior produces zero relevance (tow-only operation).

  if(!(m_tow_pose_valid && (m_rng_sys >= 0)))
    return(0);

  double range_relevance = m_obship_model.getRangeRelevance();

  // If tow is deployed, base relevance on the tow-aware system range
  if(m_tow_pose_valid && (m_rng_sys >= 0))
    range_relevance = computeRangeRelevanceFromRange(m_rng_sys);

  if(range_relevance <= 0)
    return(0);

  if(range_relevance > 0.6) 
  {
    if(m_side_lock == "") 
    {
      // When relevance is high, lock a passing side.
      // Passing side is computed using a tow-posed model when tow pose is valid.

      string pass_side = getPassingSideTowAware(m_tow_pose_valid, m_towed_x, m_towed_y, m_towed_vel_valid, m_towed_vx, m_towed_vy,
                                                m_obship_model.getOSH());

      if(pass_side == "star")
        m_side_lock = "port";
      else if(pass_side == "port")
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

  if(m_draw_buff_min_poly)
  {
    mid_poly.set_active(true);
    // If the obstacle is relevant, perhaps draw filled in
    if(m_obstacle_relevance > 0)
      m_hints.setColor("mid_fill_color", "gray70");
    else
      m_hints.setColor("mid_fill_color", "off");

    applyHints(mid_poly, m_hints, "mid");
    postMessage("VIEW_POLYGON", mid_poly.get_spec(5), "mid");
  }

  else
  {
    postMessage("VIEW_POLYGON", mid_poly.get_spec_inactive(), "mid");
  }

  // =================================================
  // Part 3 - Render the rim (outermost) polygon
  // =================================================
  XYPolygon rim_poly = m_obship_model.getRimPoly();
  
  if(m_draw_buff_max_poly)
  {
  rim_poly.set_active(true);
  
  // If the obstacle is relevant, perhaps draw filled in
  if(m_obstacle_relevance > 0)
    m_hints.setColor("rim_fill_color", "gray70");
  else
    m_hints.setColor("rim_fill_color", "off");

  applyHints(rim_poly, m_hints, "rim");
  postMessage("VIEW_POLYGON", rim_poly.get_spec(5), "rim");
  }
  else
  {
    rim_poly.set_color("fill", "invisible");
    postMessage("VIEW_POLYGON", rim_poly.get_spec_inactive(), "rim");
  }
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
  // RNG macro: when tow pose is valid, report tow system range (m_rng_sys).
  // Otherwise, fall back to ObShipModel nav range (primarily for diagnostics).

  double rng = m_obship_model.getRange();
  if(m_tow_pose_valid && (m_rng_sys >= 0))
    rng = m_rng_sys;

  if(strContains(sdata, "$[RNG]"))
    sdata = macroExpand(sdata, "RNG", rng);

  // =======================================================
    
  if(strContains(sdata, "$[BNG]"))
    sdata = macroExpand(sdata, "BNG", m_obship_model.getObcentBng());

  if(strContains(sdata, "$[RBNG]"))
    sdata = macroExpand(sdata, "RBNG", m_obship_model.getObcentRelBng());

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

//-----------------------------------------------------------
// Procedure: computeRangeRelevanceFromRange()
//   Purpose: Compute relevance on [0,1] from an explicit range value,
//            using the same inner/outer distances used by the ObShipModel.
//            This is used to apply relevance to the tow-aware system range.

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

//-----------------------------------------------------------
// Procedure: getPassingSideTowAware()
//   Purpose: Determine passing side using a temporary ObShipModel with its pose
//            overridden to the tow position (and tow heading if velocity is valid).
//            Falls back to nav passing side if tow pose is unavailable/ambiguous.

string BHV_TowObstacleAvoid::getPassingSideTowAware(bool tow_pose_valid, double tow_x, double tow_y, bool tow_vel_valid,
                                                    double tow_vx, double tow_vy, double fallback_hdg) const
{
  // Always have a nav fallback
  string nav_side = m_obship_model.getPassingSide();

  // Only override if tow is actually usable
  if(!tow_pose_valid)
    return(nav_side);

  // Choose heading for the tow-posed model:
  // - If tow velocity is valid, use it (matches your refinery approach)
  // - Otherwise fall back to vessel/nav heading (passed in)
  double hdg = fallback_hdg;
  if(tow_vel_valid) {
    double spd = hypot(tow_vx, tow_vy);
    if(spd > 1e-6)
      hdg = relAng(0, 0, tow_vx, tow_vy);
  }

  ObShipModelV24 tmp = m_obship_model;
  tmp.setPose(tow_x, tow_y, hdg);
  tmp.setCachedVals(true);

  string tow_side = tmp.getPassingSide();

  // If ambiguous, fall back
  if(tow_side == "")
    return(nav_side);

  return(tow_side);
}