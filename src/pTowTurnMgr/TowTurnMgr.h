/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: TowTurnMgr.h                                    */
/*    DATE:                                                 */
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
   double m_turn_hdg_thresh;  // heading change threshold to trigger turn (deg)

 private: // State variables
   double m_nav_x;
   double m_nav_y;
   double m_nav_heading;
   double m_towed_x;
   double m_towed_y;

   // Waypoint info from wptflags
   double m_prev_x;    // $[X]  - waypoint just captured (turn point)
   double m_prev_y;    // $[Y]
   double m_next_x;    // $[NX] - next waypoint target
   double m_next_y;    // $[NY]
   int    m_wpt_index;
   int    m_prev_wpt_index;

   bool   m_tow_deployed;
   bool   m_turn_active;

   // Leg direction tracking
   double m_last_leg_bearing;  // bearing of the leg just completed
   bool   m_last_leg_valid;

   // Turn point state: where the vehicle was and what direction
   // it was heading when the turn was triggered
   double m_turn_pt_x;
   double m_turn_pt_y;
   double m_approach_hdg;  // heading when turn was triggered
};

#endif
