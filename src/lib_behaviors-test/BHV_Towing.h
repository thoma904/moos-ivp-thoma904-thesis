/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Towing.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef Towing_HEADER
#define Towing_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_Towing : public IvPBehavior {
public:
  BHV_Towing(IvPDomain);
  ~BHV_Towing() {};
  
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
IvPFunction* buildFunctionWithZAIC();

protected: // Configuration parameters

protected: // State variables
  double m_osx;
  double m_osy;
  double m_os_heading;
  double m_towed_x;
  double m_towed_y;
  double m_towed_heading;
  bool   m_tow_act;
  string m_ipf_type;
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Towing(domain);}
}
#endif
