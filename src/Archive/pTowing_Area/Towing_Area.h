/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Towing_Area.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef Towing_Area_HEADER
#define Towing_Area_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class Towing_Area : public AppCastingMOOSApp
{
 public:
   Towing_Area();
   ~Towing_Area();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
  double m_nav_x;
  double m_nav_y;
  double m_nav_heading;
  double m_nav_speed;
  bool   m_tow_deployed;
  double m_towed_x;
  double m_towed_y;
  double m_towed_heading;
};

#endif 
