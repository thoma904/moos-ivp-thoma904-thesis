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
#include <Eigen/Dense>

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
 // Hydro Coef
 double m_rho;               // water density                 [kg/m^3]
 double m_delta_r;           // rudder angle (port +)         [rad] (updated in OnNewMail)

 double m_m_t;               // added mass at stern           [kg]
 double m_x_T;               // longitudinal ref. point       [m]  (usually negative)
 double m_m;                 // dry mass                      [kg]
 double m_M_35, m_M_55;      // dry cross prod. & yaw inertia [kg·m], [kg·m^2]
 double m_ms_33, m_ms_35, m_ms_55;  // strip‑theory added‑mass terms

 // Matrix Var
 Eigen::Vector2d m_b_pos;    // world position  (x,y)         [m]
 double          m_u3;       // sway velocity U3              [m/s]
 double          m_omega2;   // yaw rate Ω2                   [rad/s]
 double          m_psi;      // yaw angle (offset from NAV)   [rad]
};

#endif 
