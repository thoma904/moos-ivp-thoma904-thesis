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

double m_osx;
double m_osy;
double m_towx;
double m_towy;
double m_osh;
double m_osd;
bool   m_turning;

};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Williamson(domain);}
}
#endif
