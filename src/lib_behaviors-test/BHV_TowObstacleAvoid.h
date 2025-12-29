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
  
protected:
  HintHolder m_hints;
  
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_TowObstacleAvoid(domain);}
}
#endif
