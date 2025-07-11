/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BEZ_Viewer.h                                         */
/*    DATE: Apr 17th, 2020                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef BEZ_VIEWER_HEADER
#define BEZ_VIEWER_HEADER

#include <vector>
#include <string>
#include "FL/Fl.H"
#include "FL/fl_draw.H"
#include "BackImg.h"
#include "ThrustMap.h"
#include "MarineViewer.h"
#include "TowBodyModel.h"
#include "XYBezier.h"

class BEZ_GUI;

class BEZ_Viewer : public MarineViewer
{
  friend class BEZ_GUI; 
 public:
  BEZ_Viewer(int x, int y, int w, int h, const char *l=0);
  ~BEZ_Viewer() {};

  // Pure virtuals that needs to be defined
  void   modColorScheme() {};

  // Virtuals defined
  void   draw();
  int    handle(int);
  void   handle_left_mouse(int, int);
  void   handle_right_mouse(int, int) {};

 public:
  void   addPostConfigParam(std::string);
  void   addConfigParam(std::string);
  bool   handlePostConfigParams();
  bool   handleConfigParams();
  
 protected: 
  void   setThrust(double);
  void   setRudder(double);
  void   addThrust(double v) {setThrust(m_thrust+v);}
  void   addRudder(double v) {setRudder(m_rudder+v);}
  void   stepForward(double secs);

  void   drawBezier();
  void   drawVehicle();
  void   drawTowBody();

  bool   setTiffFile(std::string);

  void   setDrawPoints(bool v) {m_draw_points=v;}
  void   setDrawCurve(bool v)  {m_draw_curve=v;}
  void   toggleDrawPoints() {m_draw_points=!m_draw_points;}
  void   toggleDrawCurve() {m_draw_curve=!m_draw_curve;}

protected:
  void   initCenterView();
  
private:
  
  bool   setPointColor(std::string);
  bool   setPointSize(std::string);
  
 protected: // Available as friend to BEZ_GUI

  TowBodyModel m_tbm;

  ThrustMap  m_thrust_map;
  double     m_turn_rate;
  double     m_thrust;
  double     m_rudder;
  double     m_curr_time;
  
  bool       m_init_center_view;
  
 private: // Config vars

  std::vector<std::string> m_config_params;
  std::vector<std::string> m_post_config_params;

  std::string  m_pt_color;
  double       m_pt_size;

  bool   m_draw_points;
  bool   m_draw_curve;
};

#endif 





