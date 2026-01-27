/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_TowObstacleAvoid.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef TowObstacleAvoid_HEADER
#define TowObstacleAvoid_HEADER

#include "IvPBehavior.h"
#include "ObShipModelV24.h"
#include "XYPolygon.h"
#include "HintHolder.h"

class BHV_TowObstacleAvoid : public IvPBehavior {
public:
  BHV_TowObstacleAvoid(IvPDomain);
  ~BHV_TowObstacleAvoid() {};
  
  bool         setParam(std::string, std::string);
  void         onHelmStart();
  IvPFunction* onRunState();
  void         onIdleState();
  void         onCompleteState() {postErasablePolygons();}
  void         onSetParamComplete();
  void         onIdleToRunState();
  void         onInactiveState()  {postErasablePolygons();}
  void         onEveryState(std::string);
  bool         applyAbleFilter(std::string);
  void         postConfigStatus();
  double       getDoubleInfo(std::string);
  bool         isConstraint() {return(true);}
  std::string  expandMacros(std::string);

protected: // Local Utility functions
  
  bool   handleParamRangeFlag(std::string);

  double getRelevance();
  bool   updatePlatformInfo();
  void   postViewablePolygons();
  void   postErasablePolygons();
  void   initVisualHints();
  
  bool   applyBuffer();
  IvPFunction* buildOF();

  //Tow Specific Utilities
  double computeRangeRelevanceFromRange(double range) const;
  std::string getPassingSideTowAware(bool tow_pose_valid,
                                                    double tow_x, double tow_y,
                                                    bool tow_vel_valid,
                                                    double tow_vx, double tow_vy,
                                                    double fallback_hdg) const;

protected:
  ObShipModelV24 m_obship_model;

protected: // Configuration parameters
  bool        m_use_refinery;
  std::string m_pwt_grade;

  std::string m_resolved_obstacle_var;
  std::string m_obstacle_id;

  std::vector<double>      m_rng_thresh;
  std::vector<VarDataPair> m_rng_flags;
  std::vector<VarDataPair> m_cpa_flags;

  bool m_draw_buff_min_poly;
  bool m_draw_buff_max_poly;

  bool m_holonomic_ok;

  //Tow Specific Additions
  double m_tow_pad;

protected: // State variables
  double  m_obstacle_relevance;
  bool    m_resolved_pending;

  bool    m_valid_cn_obs_info;
  
  bool    m_closing;
  double  m_cpa_rng_sofar;
  double  m_fpa_rng_sofar;
  double  m_cpa_rng_ever;
  double  m_cpa_reported;

  std::string m_side_lock;

  bool m_allstop_on_breach;

  //Tow Specific State Variables
  // Tow state
  double m_towed_x;
  double m_towed_y;
  bool   m_use_tow_lead;
  double m_tow_lead_sec;

  double m_last_tow_x;
  double m_last_tow_y;
  double m_last_tow_time;

  double m_tow_x_eval;
  double m_tow_y_eval;

  double m_towed_vx;
  double m_towed_vy;
  bool   m_towed_vel_valid;
  bool   m_tow_pose_valid;

  double m_curr_time;

  // Cached ranges: NAV range and tow range are tracked separately.
  // System range is the tow eval/lead range when tow pose is valid.
  double      m_rng_sys;
  double      m_rng_nav;
  double      m_rng_tow;
  std::string m_rng_src;

  double m_rng_tow_actual;

  //Attempt to fix pred jumping
  double m_tow_vx_filt;
  double m_tow_vy_filt;
  bool   m_tow_vel_valid;

  double m_tow_pose_stale;
  double m_tow_lead_alpha;
  double m_tow_lead_max_speed;
  double m_tow_xy_sync_eps;

  //tow dynamics
  double m_cable_length   = 30;
  double m_attach_offset  = 0;
  double m_k_spring       = 5;
  double m_cd             = 0.7;
  double m_c_tan          = 2.0;

  double m_sim_dt = 0.1;
  double m_sim_horizon = -1;        // use allowable_ttc if <0
  double m_turn_rate_max = 15.0;    // deg/sec (tune)

protected:
  HintHolder m_hints;
  
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_TowObstacleAvoid(domain);}
}
#endif
