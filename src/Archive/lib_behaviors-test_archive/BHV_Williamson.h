/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Williamson.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef Williamson_HEADER
#define Williamson_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_Williamson : public IvPBehavior {
public:
  BHV_Williamson(IvPDomain);
  ~BHV_Williamson() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();

protected: // Local Utility functions

  IvPFunction* buildFunctionWithZAIC(double target_heading);

protected: // Configuration parameters

protected: // State variables

double m_entry_hdg;
bool   m_entry_hdg_set;
double m_target_hdg;
double m_hdg_tol;
bool   m_phase1_done;
double m_phase1_hdg;
int    m_turn_dir;        // +1 = starboard (CW/increasing hdg), -1 = port (CCW/decreasing)
double m_initial_turn;    // deg (default 60)
double m_max_step;        // deg per iteration in phase2 (default 60)
double m_capture_range;        // deg: when remaining angle <= this, command target directly
unsigned int m_settle_count;   // consecutive iterations within tolerance
unsigned int m_settle_required;

};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Williamson(domain);}
}
#endif
