/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT                                             */
/*    FILE: AOF_TowObstacleAvoid.h                          */
/*    DATE:                                                 */
/*                                                          */
/* NOTE: This AOF is derived from MOOS-IvP                  */
/* AOF_AvoidObstacleV24. It evaluates obstacle avoidance    */
/* utility for a towed body by forward-simulating tow       */
/* dynamics and computing minimum distance to the obstacle  */
/* over a time horizon.                                     */
/************************************************************/
 
#ifndef AOF_TOW_OBSTACLE_AVOID_HEADER
#define AOF_TOW_OBSTACLE_AVOID_HEADER

#include "AOF.h"
#include "ObShipModelV24.h"
#include "XYPolygon.h"

class IvPDomain;
class AOF_TowObstacleAvoid: public AOF {
public:
  AOF_TowObstacleAvoid(IvPDomain);
  ~AOF_TowObstacleAvoid() {}

 public: // virtual functions
  double evalBox(const IvPBox*) const; 
  bool   setParam(const std::string&, double);
  bool   setParam(const std::string&, const std::string&);
 public: // More virtuals defined Declare a known min/max eval range
  bool   minMaxKnown() const {return(true);}
  double getKnownMin() const {return(0);}
  double getKnownMax() const {return(100);}

  void   setObShipModel(ObShipModelV24 obm) {m_obship_model=obm;}
  bool   initialize();

  // Tow-specific methods
  void   setTowEval(bool v) {m_tow_eval = v;}
  void   setTowOnly(bool v) {m_tow_only = v;}

  //Tow Speed Penalty
  void setTowSpeedPenalty(bool v)        { m_penalize_low_tow_spd = v; }
  void setTowSpeedMin(double v)          { m_tow_spd_min = v; }          // soft threshold
  void setTowSpeedHardMin(double v)      { m_tow_spd_hard_min = v; }     // hard floor (optional)
  void setTowSpeedPenaltyPower(double p) { m_tow_spd_power = p; }        // shaping (>=1)
  void setTowSpeedPenaltyFloor(double f) { m_tow_spd_floor = f; }        // [0..1]

 private: // Config variables
  ObShipModelV24 m_obship_model;

  bool   m_tow_eval;
  double m_tow_x;
  double m_tow_y;
  bool   m_tow_pose_set;
  bool   m_tow_only;

 private: // State variables
  int    m_crs_ix;  // Index of "course" variable in IvPDomain
  int    m_spd_ix;  // Index of "speed"  variable in IvPDomain


 // Tow dynamics interface (used by BHV_TowObstacleAvoid)
 public:
  void setTowState(double x, double y, double vx, double vy);
  void setTowDynParams(double cable_len, double attach_offset,
                       double k_spring, double cd, double c_tan);
  void setSimParams(double dt, double horizon, double turn_rate_max_deg = 0.0);

 private:
  void propagateTowOneStep(double ax, double ay, double dt,
                           double &tx, double &ty,
                           double &tvx, double &tvy) const;
  double applyTowSpeedPenalty(double util, double tow_spd_metric) const;

 private:
  // Tow state (position and velocity at eval start)
  double m_tow_vx;
  double m_tow_vy;

  // Tow dynamics params (matching pTowing)
  bool   m_dyn_params_set;
  double m_cable_length;
  double m_attach_offset;
  double m_k_spring;
  double m_cd;
  double m_c_tan;

  // Forward simulation params
  double m_sim_dt;
  double m_sim_horizon;
  double m_turn_rate_max;

  // Tow speed penalty params
  bool   m_penalize_low_tow_spd;
  double m_tow_spd_min;
  double m_tow_spd_hard_min;
  double m_tow_spd_power;
  double m_tow_spd_floor;

};

#endif
