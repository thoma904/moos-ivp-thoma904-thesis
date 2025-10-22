/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Towing_Circle.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef Towing_Circle_HEADER
#define Towing_Circle_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class Towing_Circle : public AppCastingMOOSApp
{
 public:
   Towing_Circle();
   ~Towing_Circle();

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
    double m_towed_x;
    double m_towed_y;
    double m_towed_heading;
    double m_prev_time;
    double m_radius_initial;
    double m_radius;
};

#endif 
