/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT                                             */
/*    FILE: BHV_TowedTurn.h                                 */
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

protected: // Configuration parameters
  double m_jog_angle;     // degrees to jog opposite before sweeping

protected: // State variables
  int    m_turn_dir;      // +1 = starboard, -1 = port (computed from next WP)
  double m_target_hdg;    // reciprocal of entry heading
  double m_jog_hdg;       // phase 1 jog heading

  bool   m_entry_hdg_set;
  double m_entry_hdg;

  bool   m_jog_done;
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain)
  {return new BHV_TowedTurn(domain);}
}
#endif
