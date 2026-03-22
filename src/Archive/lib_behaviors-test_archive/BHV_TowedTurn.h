/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_TowedTurn.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef TowedTurn_HEADER
#define TowedTurn_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_TowedTurn : public IvPBehavior {
public:
  BHV_TowedTurn(IvPDomain);
  ~BHV_TowedTurn() {};
  
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

protected: // Configuration parameters

protected: // State variables

  double m_turn_dir;
  double m_target_hdg;

  bool   m_entry_hdg_set;
  double m_entry_hdg;

  bool  m_phase1_done;
  double m_phase1_hdg;

};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_TowedTurn(domain);}
}
#endif
