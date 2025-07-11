/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BEZ_GUI.h                                            */
/*    DATE: Apr 17th 2020                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef BEZ_GUI_HEADER
#define BEZ_GUI_HEADER

#include <vector>
#include <FL/Fl.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Button.H>
#include "MarineGUI.h"
#include "BEZ_Viewer.h"

class BEZ_GUI : public MarineGUI {
public:
  BEZ_GUI(int w, int h, const char *l=0);
  ~BEZ_GUI() {};
  
  void  resize(int, int, int, int);
  void  updateXY();
  int   handle(int);

  bool  addPoly(std::string);

  void  addConfigParam(std::string);
  void  addPostConfigParam(std::string);
  
 protected:
  void  augmentMenu();
  void  initWidgets();
  void  resizeWidgetsShape();
  void  resizeWidgetsText();

 public:

  BEZ_Viewer *m_bez_viewer;

 protected:

  // Panel - Column ONE 
  Fl_Output  *m_fld_ptax;
  Fl_Output  *m_fld_ptay;

  // Panel - Column TWO
  Fl_Output  *m_fld_ptzx;
  Fl_Output  *m_fld_ptzy;

  // Panel - Column THREE 
  Fl_Output  *m_fld_ptmx;
  Fl_Output  *m_fld_ptmy;
  
  // Panel - Column Four
  Fl_Output  *m_fld_clen; // curve length
  Fl_Output  *m_fld_mlen; // max length
  
  //=================================================
  // BOTTOM Panel
  //=================================================

 private:
  inline void cb_CenterView_i();
  static void cb_CenterView(Fl_Widget*);
  
  inline void cb_StepForward_i(int);
  static void cb_StepForward(Fl_Widget*, int);

  inline void cb_StepForwardPort_i(int);
  static void cb_StepForwardPort(Fl_Widget*, int);

  inline void cb_StepForwardStar_i(int);
  static void cb_StepForwardStar(Fl_Widget*, int);


  inline void cb_RotateCurve_i(int);
  static void cb_RotateCurve(Fl_Widget*, int);

  inline void cb_AltCurveX_i(int);
  static void cb_AltCurveX(Fl_Widget*, int);

  inline void cb_AltCurveY_i(int);
  static void cb_AltCurveY(Fl_Widget*, int);

  inline void cb_AltCurveLen_i(int);
  static void cb_AltCurveLen(Fl_Widget*, int);

  inline void cb_SetGeoAttr_i(int);
  static void cb_SetGeoAttr(Fl_Widget*, int);

  int m_start_wid;
  int m_start_hgt;
};
#endif







