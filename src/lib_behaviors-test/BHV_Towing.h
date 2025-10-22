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
#include "XYPolygon.h"

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

protected: // Configuration parameters

string m_obstacle_var = "OBM_GUTS";
double m_peak_width = 30;
double m_base_width = 180;
double m_summit_delta = 30;

protected: // State variables

};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Towing(domain);}
}
#endif
