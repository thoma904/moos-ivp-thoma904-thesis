/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: TowTurnMgr.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef TowTurnMgr_HEADER
#define TowTurnMgr_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class TowTurnMgr : public AppCastingMOOSApp
{
 public:
   TowTurnMgr();
   ~TowTurnMgr();

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
    double m_cable_length;

 private: // State variables
    double m_nav_x;
    double m_nav_y;
    double m_nav_heading;
    double m_nav_speed;
    double m_towed_x;
    double m_towed_y;
    double m_prev_x;
    double m_prev_y;
    double m_next_x;
    double m_next_y;
    bool   m_tow_deployed;
    double m_leg_length;
    double m_dist_end;
    double m_turn_factor;
    double m_wpt_index;
    bool m_turn_active;
    std::string m_turn_dir;
    bool m_posted_direction;
    int m_next_turn_dir;
};

#endif 
