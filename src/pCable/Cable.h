/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Cable.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef Cable_HEADER
#define Cable_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <vector>
#include <cmath>

struct CableNode {
  double x, y, vx, vy;
  CableNode() : x(0), y(0), vx(0), vy(0) {}
  CableNode(double _x, double _y) : x(_x), y(_y), vx(0), vy(0) {}
};

class Cable : public AppCastingMOOSApp
{
 public:
   Cable();
   ~Cable();

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
   double m_attach_offset;
   double m_k_spring;
   double m_cd;
   double m_c_tan;

 private: // State variables
   double m_nav_x;
   double m_nav_y;
   double m_nav_heading;
   double m_towed_x;
   double m_towed_y;

   bool   m_initialized;
   int    m_num_nodes;
   double m_rest_length;
   std::vector<CableNode> m_nodes;
   double m_last_iterate_time;
};

#endif
