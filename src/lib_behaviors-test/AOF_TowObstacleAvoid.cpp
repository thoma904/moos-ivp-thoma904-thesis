/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT                                             */
/*    FILE: AOF_TowObstacleAvoid.cpp                        */
/*    DATE:                                                 */
/*                                                          */
/* NOTE: This AOF is derived from MOOS-IvP                  */
/* AOF_AvoidObstacleV24. It evaluates obstacle avoidance    */
/* utility for a towed body by forward-simulating tow       */
/* dynamics (matching pTowing) and computing minimum        */
/* distance to the obstacle over a time horizon.            */
/* Used by BHV_TowObstacleAvoid.                            */
/************************************************************/

#include <cmath>
#include <string>
#include <algorithm>
#include "AOF_TowObstacleAvoid.h"
#include "AngleUtils.h"

using namespace std;

//---------------------------------------------------------------
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
  m_tow_only      = false;

  // Tow state
  m_tow_vx         = 0;
  m_tow_vy         = 0;

  // Tow dynamics params (matching pTowing)
  m_dyn_params_set = false;
  m_cable_length   = 30;
  m_attach_offset  = 0;
  m_k_spring       = 5;
  m_cd             = 0.7;
  m_c_tan          = 2.0;

  // Forward simulation params
  m_sim_dt         = 0.2;
  m_sim_horizon    = -1;
  m_turn_rate_max  = 0.0;

  // Cable avoidance
  m_cable_sample_step = 1.0;
  m_cable_check_interval = 5;

  // Tow speed penalty (disabled by default)
  m_penalize_low_tow_spd = true;
  m_tow_spd_min          = 1.0;
  m_tow_spd_hard_min     = 0.0;
  m_tow_spd_power        = 2.0;
  m_tow_spd_floor        = 0.25;
}

//----------------------------------------------------------------
// Procedure: setParam()

bool AOF_TowObstacleAvoid::setParam(const string& param, double val)
{
  return(false);
}

bool AOF_TowObstacleAvoid::setParam(const string& param, const string& value)
{
  return(false);
}

//----------------------------------------------------------------
// Procedure: initialize()

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
  if((m_sim_horizon <= 0) && !m_obship_model.paramIsSet("allowable_ttc"))
    return(postMsgAOF("allowable_ttc or sim_horizon must be set"));

  if(!m_obship_model.getGutPoly().is_convex())
    return(postMsgAOF("m_obstacle is not convex"));

  // --- Tow state validation ---
  if(m_tow_eval) 
  { 
    if(!m_tow_pose_set)
      return(postMsgAOF("tow_eval enabled but tow pose not set"));
    if(!m_dyn_params_set)
      return(postMsgAOF("tow_eval enabled but dyn params not set"));
  }

  return(true);
}

//----------------------------------------------------------------
// Procedure: clamp01()

static double clamp01(double v)
{
  if(v < 0.0) return 0.0;
  if(v > 1.0) return 1.0;
  return v;
}

//----------------------------------------------------------------
// Procedure: evalBox()
//   Purpose: Evaluate a candidate (course, speed) box by forward-
//            simulating tow dynamics and returning utility based on
//            minimum predicted tow-to-obstacle distance.

double AOF_TowObstacleAvoid::evalBox(const IvPBox *b) const
{
  double eval_crs = 0;
  double eval_spd = 0;
  m_domain.getVal(m_crs_ix, b->pt(m_crs_ix,0), eval_crs);
  m_domain.getVal(m_spd_ix, b->pt(m_spd_ix,0), eval_spd);

  // If tow evaluation is not enabled or state is not ready, return max utility
  if(!m_tow_eval)
    return(getKnownMax());
  if(!m_tow_pose_set || !m_dyn_params_set)
    return(getKnownMax());

  // Simulation horizon: use configured value or fall back to allowable_ttc.
  // A value of -2 signals "static cable check only, no forward sim."
  bool static_only = (m_sim_horizon < -1.5);
  double T = static_only ? 0 : ((m_sim_horizon > 0) ? m_sim_horizon : m_obship_model.getAllowableTTC());
  double dt = m_sim_dt;
  int steps = (dt > 1e-6) ? (int)ceil(T / dt) : 0;
  if(steps <= 0 && !static_only)
    return(getKnownMax());

  // Initial ownship pose (drives the tow anchor point)
  double osx = m_obship_model.getOSX();
  double osy = m_obship_model.getOSY();
  double osh = m_obship_model.getOSH();

  // Initial tow state (position and velocity)
  double tx  = m_tow_x;
  double ty  = m_tow_y;
  double tvx = m_tow_vx;
  double tvy = m_tow_vy;

  // Track minimum predicted tow speed over the horizon
  double min_tow_spd = 1e9;

  XYPolygon gut = m_obship_model.getGutPoly();

  // Compute cable node layout (mirrors Cable.cpp logic)
  int num_nodes;
  if(m_cable_length < 30.0)
    num_nodes = 3;
  else
    num_nodes = std::max(3, (int)(m_cable_length / 10.0));
  double rest_length = m_cable_length / (double)(num_nodes - 1);

  // Initialize node arrays — straight line from initial anchor to tow body
  vector<double> n_x(num_nodes), n_y(num_nodes);
  vector<double> n_vx(num_nodes, 0.0), n_vy(num_nodes, 0.0);

  double hdg_rad0 = (90.0 - osh) * M_PI / 180.0;
  double ax0 = osx - m_attach_offset * cos(hdg_rad0);
  double ay0 = osy - m_attach_offset * sin(hdg_rad0);

  for(int i = 0; i < num_nodes; i++) {
    double t = (double)i / (double)(num_nodes - 1);
    n_x[i] = ax0 + t * (tx - ax0);
    n_y[i] = ay0 + t * (ty - ay0);
  }

  // Track minimum tow-to-obstacle distance over the horizon
  double min_dist = 1e9;

  // Track time-to-first-contact: when min_dist hits 0, record which
  // simulation step it happened on.  -1 means no contact predicted.
  int contact_step = -1;

  // Check all initial cable nodes for obstacle proximity (replaces straight-line sampling)
  for(int i = 0; i < num_nodes; i++) {
    double ds = gut.dist_to_poly(n_x[i], n_y[i]);
    if(ds < 0) ds = 0;
    min_dist = std::min(min_dist, ds);
    if(min_dist <= 0) {
      contact_step = 0;
      break;
    }
  }

  // Forward simulation of vessel + tow dynamics
  double vh = osh;       // vessel heading (turn-rate-limited)
  double vs = eval_spd;  // vessel speed (constant over horizon)

  for(int k = 0; k < steps; k++) {

    // If contact already occurred, stop simulating
    if(contact_step >= 0)
      break;

    // Heading update (with optional turn-rate limit)
    if(m_turn_rate_max > 0) {
      double diff = angleDiff(eval_crs, vh);
      double step_deg = m_turn_rate_max * dt;
      if(fabs(diff) <= step_deg)
        vh = eval_crs;
      else
        vh = angle360(vh + (diff > 0 ? step_deg : -step_deg));
    } else {
      vh = eval_crs;
    }

    // Vessel position update (simple kinematics)
    double hdg_rad = (90.0 - vh) * M_PI / 180.0;
    osx += vs * cos(hdg_rad) * dt;
    osy += vs * sin(hdg_rad) * dt;

    // Anchor point (stern attachment offset from vessel CG)
    double ax = osx - m_attach_offset * cos(hdg_rad);
    double ay = osy - m_attach_offset * sin(hdg_rad);

    // Step 1: Advance the tow body endpoint (matches pTowing physics)
    // This gives us the new tx/ty to use as the pinned last node
    propagateTowOneStep(ax, ay, dt, tx, ty, tvx, tvy);

    // Track minimum predicted tow speed
    double tow_spd = hypot(tvx, tvy);
    if(tow_spd < min_tow_spd)
      min_tow_spd = tow_spd;

    // Step 2: Advance the full cable shape using the updated endpoint
    // (mirrors Cable.cpp interior node dynamics)
    propagateCableOneStep(ax, ay, tx, ty, dt, num_nodes, rest_length,
                          n_x, n_y, n_vx, n_vy);

    // Check all cable nodes against obstacle every N steps
    // Always check every step if we're close (min_dist is small)
    if(k % m_cable_check_interval == 0 || min_dist < 5.0) {
      for(int i = 0; i < num_nodes; i++) {
        double d = gut.dist_to_poly(n_x[i], n_y[i]);
        if(d < 0) d = 0;
        min_dist = std::min(min_dist, d);
        if(min_dist <= 0) break;
      }
    } else {
      // Between full checks, always check the tow body endpoint
      double d = gut.dist_to_poly(tx, ty);
      if(d < 0) d = 0;
      min_dist = std::min(min_dist, d);
    }

    if(min_dist <= 0)
      contact_step = k + 1;
  }

  // Map minimum distance to utility using min/max CPA thresholds
  // Above max_util_cpa: full utility (safe)
  // Between min and max: linear ramp from sub_ceiling to max
  // Below min_util_cpa: small but nonzero gradient so the behavior
  //   stays active and prefers larger CPA even when all options are bad
  // Contact predicted (min_dist=0): use time-to-contact so the behavior
  //   prefers headings that delay contact even when it can't be avoided
  double minu = m_obship_model.getMinUtilCPA();
  double maxu = m_obship_model.getMaxUtilCPA();

  double umin = getKnownMin();
  double umax = getKnownMax();

  // Utility is divided into four continuous bands:
  //   contact predicted:  [umin .. ttc_ceiling]   based on time-to-contact
  //   CPA below min_util: [ttc_ceiling .. dist_ceiling] based on min_dist
  //   CPA in ramp zone:   [dist_ceiling .. umax]  based on min_dist
  //   CPA above max_util: umax
  // ttc_ceiling < dist_ceiling so "no contact" always beats "late contact"
  // Values must be large enough for the IvP reflector to resolve differences
  double ttc_ceiling  = umin + 0.40 * (umax - umin);  // 40% of range
  double dist_ceiling = umin + 0.50 * (umax - umin);  // 50% of range

  double u_obs = umin;

  if(contact_step >= 0) {
    // Contact was predicted. Use time-to-contact as utility:
    // later contact = higher utility, up to ttc_ceiling.
    // contact_step=0 (immediate) -> umin
    // contact_step=steps (end of horizon) -> ttc_ceiling
    double ttc_frac = (steps > 0) ? (double)contact_step / (double)steps : 0;
    u_obs = umin + ttc_frac * (ttc_ceiling - umin);
  }
  else if(min_dist >= maxu)
    u_obs = umax;
  else if(min_dist >= minu) {
    double pct = (min_dist - minu) / (maxu - minu);
    u_obs = dist_ceiling + pct * (umax - dist_ceiling);
  }
  else {
    // No contact but CPA < min_util. Ramp from ttc_ceiling to dist_ceiling.
    double pct = min_dist / minu;
    u_obs = ttc_ceiling + pct * (dist_ceiling - ttc_ceiling);
  }

  // Apply tow-speed penalty based on predicted tow speed
  if(min_tow_spd > 1e8)
    return u_obs;

  return applyTowSpeedPenalty(u_obs, min_tow_spd);
}

//----------------------------------------------------------------
// Procedure: setTowState()

void AOF_TowObstacleAvoid::setTowState(double x, double y, double vx, double vy)
{
  m_tow_x = x;
  m_tow_y = y;
  m_tow_vx = vx;
  m_tow_vy = vy;
  m_tow_pose_set = true;
}

//----------------------------------------------------------------
// Procedure: setTowDynParams()

void AOF_TowObstacleAvoid::setTowDynParams(double L, double attach,
                                          double k, double cd, double c_tan)
{
  m_cable_length  = L;
  m_attach_offset = attach;
  m_k_spring      = k;
  m_cd            = cd;
  m_c_tan         = c_tan;
  m_dyn_params_set = true;
}

//----------------------------------------------------------------
// Procedure: setSimParams()

void AOF_TowObstacleAvoid::setSimParams(double dt, double horizon, double turn_rate_max_deg)
{
  if(dt > 1e-4) m_sim_dt = dt;
  m_sim_horizon = horizon;
  m_turn_rate_max = turn_rate_max_deg;
}

//----------------------------------------------------------------
// Procedure: propagateTowOneStep()
//   Purpose: Advance the tow body state by one time step.
//            Replicates pTowing deployed-state dynamics: spring
//            tension, quadratic drag, tangential damping, and
//            rigid cable clamp.

void AOF_TowObstacleAvoid::propagateTowOneStep(double ax, double ay, double dt,
                                               double &tx, double &ty,
                                               double &tvx, double &tvy) const
{
  if(!(dt > 0))
    return;
  dt = std::max(dt, 1e-3);

  // Fallback: if cable length is invalid, integrate with drag only
  if(!(m_cable_length > 0)) {
    double spd = std::hypot(tvx, tvy);
    if((spd > 1e-6) && (m_cd > 0)) {
      tvx += -m_cd * tvx * spd * dt;
      tvy += -m_cd * tvy * spd * dt;
    }
    tx += tvx * dt;
    ty += tvy * dt;
    return;
  }

  // Vector from tow body to anchor point
  double dx = ax - tx;
  double dy = ay - ty;
  double distance = std::hypot(dx, dy);

  // If tow is essentially co-located with anchor, apply drag only
  if(distance <= 0.01) {
    double spd = std::hypot(tvx, tvy);
    if((spd > 1e-6) && (m_cd > 0)) {
      tvx += -m_cd * tvx * spd * dt;
      tvy += -m_cd * tvy * spd * dt;
    }
    tx += tvx * dt;
    ty += tvy * dt;
    return;
  }

  // Unit vector along cable (tow -> anchor)
  double d_dir = std::hypot(dx, dy);
  if(d_dir < 1e-6)
    d_dir = 1.0;
  double ux = dx / d_dir;
  double uy = dy / d_dir;

  // Tangential unit vector (perpendicular to cable)
  double nx = -uy;
  double ny =  ux;

  // Spring tension when cable is overstretched
  if((distance > m_cable_length) && (m_k_spring > 0)) {
    double overshoot = distance - m_cable_length;
    tvx += m_k_spring * overshoot * ux * dt;
    tvy += m_k_spring * overshoot * uy * dt;
  }

  // Quadratic drag
  double speed = std::hypot(tvx, tvy);
  if((speed > 1e-6) && (m_cd > 0)) {
    tvx += -m_cd * tvx * speed * dt;
    tvy += -m_cd * tvy * speed * dt;
  }

  // Tangential damping (penalizes sideways motion)
  if(m_c_tan > 0) {
    double vt = tvx * nx + tvy * ny;
    tvx += (-m_c_tan * vt) * nx * dt;
    tvy += (-m_c_tan * vt) * ny * dt;
  }

  // Euler position integration
  tx += tvx * dt;
  ty += tvy * dt;

  // Rigid cable clamp: project tow back onto cable radius
  double sx = ax - tx;
  double sy = ay - ty;
  double dist_a = std::hypot(sx, sy);

  if((dist_a > m_cable_length) && (dist_a > 1e-9)) {
    double sc = m_cable_length / dist_a;
    tx = ax - sx * sc;
    ty = ay - sy * sc;

    // Remove outward radial velocity component
    double urx = sx / dist_a;
    double ury = sy / dist_a;
    double vrad = tvx * urx + tvy * ury;
    if(vrad < 0) {
      tvx -= vrad * urx;
      tvy -= vrad * ury;
    }
  }
} 

void AOF_TowObstacleAvoid::propagateCableOneStep(
    double ax, double ay,   // anchor (node 0, pinned)
    double tx, double ty,   // tow body (last node, pinned)
    double dt,
    int num_nodes,
    double rest_length,
    vector<double> &nx_arr,
    vector<double> &ny_arr,
    vector<double> &nvx_arr,
    vector<double> &nvy_arr) const
{
  dt = std::max(dt, 1e-3);

  // Pin endpoints
  nx_arr[0]  = ax;  ny_arr[0]  = ay;
  nvx_arr[0] = 0;   nvy_arr[0] = 0;
  nx_arr[num_nodes-1]  = tx;  ny_arr[num_nodes-1]  = ty;
  nvx_arr[num_nodes-1] = 0;   nvy_arr[num_nodes-1] = 0;

  // Interior node dynamics — mirrors Cable.cpp exactly
  for(int i = 1; i < num_nodes - 1; i++) {
    // Spring from previous neighbor
    double dx_prev = nx_arr[i-1] - nx_arr[i];
    double dy_prev = ny_arr[i-1] - ny_arr[i];
    double dist_prev = hypot(dx_prev, dy_prev);
    if(dist_prev > 0.01 && dist_prev > rest_length && m_k_spring > 0) {
      double ux = dx_prev / dist_prev;
      double uy = dy_prev / dist_prev;
      double overshoot = dist_prev - rest_length;
      nvx_arr[i] += m_k_spring * overshoot * ux * dt;
      nvy_arr[i] += m_k_spring * overshoot * uy * dt;
    }

    // Spring from next neighbor
    double dx_next = nx_arr[i+1] - nx_arr[i];
    double dy_next = ny_arr[i+1] - ny_arr[i];
    double dist_next = hypot(dx_next, dy_next);
    if(dist_next > 0.01 && dist_next > rest_length && m_k_spring > 0) {
      double ux = dx_next / dist_next;
      double uy = dy_next / dist_next;
      double overshoot = dist_next - rest_length;
      nvx_arr[i] += m_k_spring * overshoot * ux * dt;
      nvy_arr[i] += m_k_spring * overshoot * uy * dt;
    }

    // Quadratic drag
    double speed = hypot(nvx_arr[i], nvy_arr[i]);
    if(speed > 1e-6 && m_cd > 0) {
      nvx_arr[i] += -m_cd * nvx_arr[i] * speed * dt;
      nvy_arr[i] += -m_cd * nvy_arr[i] * speed * dt;
    }

    // Tangential damping
    if(m_c_tan > 0) {
      double cx   = nx_arr[i+1] - nx_arr[i-1];
      double cy   = ny_arr[i+1] - ny_arr[i-1];
      double clen = hypot(cx, cy);
      if(clen > 1e-6) {
        double utx = cx / clen;
        double uty = cy / clen;
        double perpx = -uty;
        double perpy =  utx;
        double vn = nvx_arr[i] * perpx + nvy_arr[i] * perpy;
        nvx_arr[i] += (-m_c_tan * vn) * perpx * dt;
        nvy_arr[i] += (-m_c_tan * vn) * perpy * dt;
      }
    }

    // Euler integration
    nx_arr[i] += nvx_arr[i] * dt;
    ny_arr[i] += nvy_arr[i] * dt;
  }

  // Forward constraint pass
  for(int i = 1; i < num_nodes - 1; i++) {
    double dx   = nx_arr[i-1] - nx_arr[i];
    double dy   = ny_arr[i-1] - ny_arr[i];
    double dist = hypot(dx, dy);
    if(dist > rest_length && dist > 1e-9) {
      double sc = rest_length / dist;
      nx_arr[i] = nx_arr[i-1] - dx * sc;
      ny_arr[i] = ny_arr[i-1] - dy * sc;
      double urx = dx / dist;
      double ury = dy / dist;
      double vrad = nvx_arr[i] * urx + nvy_arr[i] * ury;
      if(vrad < 0) {
        nvx_arr[i] -= vrad * urx;
        nvy_arr[i] -= vrad * ury;
      }
    }
  }

  // Backward constraint pass
  for(int i = num_nodes - 2; i >= 1; i--) {
    double dx   = nx_arr[i+1] - nx_arr[i];
    double dy   = ny_arr[i+1] - ny_arr[i];
    double dist = hypot(dx, dy);
    if(dist > rest_length && dist > 1e-9) {
      double sc = rest_length / dist;
      nx_arr[i] = nx_arr[i+1] - dx * sc;
      ny_arr[i] = ny_arr[i+1] - dy * sc;
      double urx = dx / dist;
      double ury = dy / dist;
      double vrad = nvx_arr[i] * urx + nvy_arr[i] * ury;
      if(vrad < 0) {
        nvx_arr[i] -= vrad * urx;
        nvy_arr[i] -= vrad * ury;
      }
    }
  }
}

//----------------------------------------------------------------
// Procedure: applyTowSpeedPenalty()
//   Purpose: Scale down obstacle avoidance utility when predicted
//            tow speed is below acceptable thresholds. This
//            discourages maneuvers that would stall the tow body.

double AOF_TowObstacleAvoid::applyTowSpeedPenalty(double util, double tow_spd_metric) const
{
  if(!m_penalize_low_tow_spd)
    return util;

  const double umin = getKnownMin();
  const double umax = getKnownMax();
  const double urng = (umax - umin);
  if(urng <= 1e-9)
    return util;

  // Optional hard floor: if predicted tow speed drops below this, make it worst
  if((m_tow_spd_hard_min > 0.0) && (tow_spd_metric < m_tow_spd_hard_min))
    return umin;

  // Soft penalty disabled unless a meaningful threshold is set
  if(!(m_tow_spd_min > 0.0))
    return util;

  // No penalty if above threshold
  if(tow_spd_metric >= m_tow_spd_min)
    return util;

  // frac in [0..1]
  double frac = (tow_spd_metric <= 0.0) ? 0.0 : (tow_spd_metric / m_tow_spd_min);
  frac = clamp01(frac);

  // shape (quadratic by default)
  double shaped = frac;
  if(m_tow_spd_power > 0.0)
    shaped = pow(frac, m_tow_spd_power);

  // factor in [floor..1]
  double floor = clamp01(m_tow_spd_floor);
  double factor = floor + (1.0 - floor) * shaped;

  // Scale normalized utility (never increases utility)
  double unorm = (util - umin) / urng;
  unorm = clamp01(unorm);
  unorm *= factor;

  return (umin + unorm * urng);
}