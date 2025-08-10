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
 double m_towed_vx; // Velocity of towed body in x direction
 double m_towed_vy; // Velocity of towed body in y direction
 double m_prev_time; // Previous time for velocity calculation
 bool m_deployed;
 double m_cable_distance; //for trouble-shooting
 double m_nav_speed; // Vessel speed
 double m_nav_vx; // Vessel velocity in x direction
 double m_nav_vy; // Vessel velocity in y direction
};

#endif 
