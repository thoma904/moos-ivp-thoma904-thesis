/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / MIT Cambridge MA            */
/*    FILE: AOF_TowObstacleAvoid.h                               */
/*    DATE: Sep 11, 2019 (derived from AOF_AvoidObstacle)        */
/*    DATE: Aug 5th, 2023 (AvoidObstacleX + PlatModel)           */
/*                                                               */
/* This file is part of IvP Helm Core Libs                       */
/*                                                               */
/* IvP Helm Core Libs is free software: you can redistribute it  */
/* and/or modify it under the terms of the Lesser GNU General    */
/* Public License as published by the Free Software Foundation,  */
/* either version 3 of the License, or (at your option) any      */
/* later version.                                                */
/*                                                               */
/* IvP Helm Core Libs is distributed in the hope that it will    */
/* be useful but WITHOUT ANY WARRANTY; without even the implied  */
/* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/* PURPOSE. See the Lesser GNU General Public License for more   */
/* details.                                                      */
/*                                                               */
/* You should have received a copy of the Lesser GNU General     */
/* Public License along with MOOS-IvP.  If not, see              */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/
 
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
  void   setTowPose(double x, double y, double hdg);
  void   setTowEval(bool v) {m_tow_eval = v;}
  void setTowOnly(bool v) { m_tow_only = v; }

  //Tow Speed Penalty
  void setTowSpeedPenalty(bool v)        { m_penalize_low_tow_spd = v; }
  void setTowSpeedMin(double v)          { m_tow_spd_min = v; }          // soft threshold
  void setTowSpeedHardMin(double v)      { m_tow_spd_hard_min = v; }     // hard floor (optional)
  void setTowSpeedPenaltyPower(double p) { m_tow_spd_power = p; }        // shaping (>=1)
  void setTowSpeedPenaltyFloor(double f) { m_tow_spd_floor = f; }        // [0..1]

 private: // Config variables
  ObShipModelV24 m_obship_model;

  ObShipModelV24 m_obship_model_tow; // Tow-shifted obstacle model
  bool m_tow_eval;
  double m_tow_x;
  double m_tow_y;
  double m_tow_hdg;
  bool m_tow_pose_set;
  bool m_tow_model_ready;
  bool m_tow_breached;
  bool m_tow_only;

 private: // State variables
  int    m_crs_ix;  // Index of "course" variable in IvPDomain
  int    m_spd_ix;  // Index of "speed"  variable in IvPDomain


//Updating with ptowing dynamics

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
  // Tow state (at eval start)
  double m_tow_vx = 0;
  double m_tow_vy = 0;

  // Tow dynamics params (match pTowing)
  bool   m_dyn_params_set = false;
  double m_cable_length   = 30;
  double m_attach_offset  = 0;
  double m_k_spring       = 5;
  double m_cd             = 0.7;
  double m_c_tan          = 2.0;

  // Simulation params
  double m_sim_dt         = 0.2;   // pick something reasonable
  double m_sim_horizon    = -1;    // if <0 use allowable_ttc
  double m_turn_rate_max  = 0.0;   // 0 => instantaneous heading

  //Tow Speed Penalty Params
  bool   m_penalize_low_tow_spd;
  double m_tow_spd_min;
  double m_tow_spd_hard_min;
  double m_tow_spd_power;
  double m_tow_spd_floor;

};

#endif
