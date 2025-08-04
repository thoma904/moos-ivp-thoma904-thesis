/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Towing.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef Towing_HEADER
#define Towing_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYSegList.h"


class Towing : public AppCastingMOOSApp
{
 public:
   Towing();
   ~Towing();

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
  std::string join(const std::vector<std::string> &vec, const std::string &delim);

 private: // State variables
 double m_nav_x;
 double m_nav_y;
 double m_nav_heading;
 double m_towed_x;
 double m_towed_y;
 double m_cable_length;
 XYSegList m_towing_position;
 double m_start_x;
 double m_start_y;
 double m_prev_heading;
 double m_nav_speed;
 double m_prev_time;
 double m_cable_tension; 
 double m_cable_angle; 
 double m;     // dry mass of towed body  [kg]
 double mT;     // entrained (cable‑end) mass [kg]
 double xT;     // longitudinal offset of tow‑point [m]

 double ms33;   // added‑mass terms from Newman
 double ms35;   // added‑mass terms from Newman
 double ms55;   // added‑mass terms from Newman
 double M35;    // added‑mass terms from Newman
 double M55;    // added‑mass terms from Newman

 /* states propagated each Iterate() */
 double U1;   // surge of towing vehicle (input)  [m/s]
 double U3;   // sway/heave component of towed body [m/s]
 double Omega2;   // pitch‑rate (body‑axis 2)          [rad/s]

 double m_towed_x;   // output position in world frame
 double m_towed_y;
 double tow_heading; // deg, world frame yaw (uses Ω2)
};

#endif 
