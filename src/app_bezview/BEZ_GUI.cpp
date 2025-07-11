/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BEZ_GUI.cpp                                          */
/*    DATE: Apr 17th, 2020                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "BEZ_GUI.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "GeomUtils.h"

using namespace std;

//----------------------------------------------------------------
// Constructor

BEZ_GUI::BEZ_GUI(int wid, int hgt, const char *label)
  : MarineGUI(wid, hgt, label) 
{
  this->user_data((void*)(this));
  this->when(FL_WHEN_CHANGED);
  this->begin();
  // size_range(minw,minh, maxw=0,maxh=0)
  this->size_range(800,800, 2500,1800);

  m_start_hgt = hgt;
  m_start_wid = wid;

  augmentMenu();
  setMenuItemColors();

  initWidgets();
  resizeWidgetsShape();
  resizeWidgetsText();
  

  this->end(); 
  this->resizable(this);
  this->show();
}

//----------------------------------------------------------------
// Procedure: addConfigParam()

void BEZ_GUI::addConfigParam(string param)
{
  if(!m_bez_viewer)
    return;

  m_bez_viewer->addConfigParam(param);
}

//----------------------------------------------------------------
// Procedure: addPostConfigParam()

void BEZ_GUI::addPostConfigParam(string param)
{
  if(!m_bez_viewer)
    return;

  m_bez_viewer->addPostConfigParam(param);
}

//--------------------------------------------------------------- 
// Procedure: initWidgets()     

void BEZ_GUI::initWidgets()
{
  //Fl_Color fcolor_blue  = fl_rgb_color(140, 140, 220);
  Fl_Color fcolor_beige = fl_rgb_color(223, 219, 191);
  //Fl_Color fcolor_green = fl_rgb_color(200, 230, 190);
  //Fl_Color fcolor_dark_goldenrod  = fl_rgb_color(184, 136, 11);
  //Fl_Color fcolor_dark_goldenrodx = fl_rgb_color(136, 136, 11);

  //=================================================
  // Main Bezier Test Viewer
  //=================================================
  m_bez_viewer = new BEZ_Viewer(1, 1, 1, 1);
  m_mviewer    = m_bez_viewer;

  //=================================================
  // Column One - Pt A
  m_fld_ptax = new Fl_Output(0, 0, 1, 1, "PtA X:");
  m_fld_ptax->color(fcolor_beige);
  m_fld_ptax->clear_visible_focus();

  m_fld_ptay = new Fl_Output(0, 0, 1, 1, "PtA Y:");
  m_fld_ptay->color(fcolor_beige);
  m_fld_ptay->clear_visible_focus();

  //=================================================
  // Column Two - Pt Z
  m_fld_ptzx = new Fl_Output(0, 0, 1, 1, "PtZ X:");
  m_fld_ptzx->color(fcolor_beige);
  m_fld_ptzx->clear_visible_focus();

  m_fld_ptzy = new Fl_Output(0, 0, 1, 1, "PtZ Y:");
  m_fld_ptzy->color(fcolor_beige);
  m_fld_ptzy->clear_visible_focus();

  //=================================================
  // Column Three - Pt M
  m_fld_ptmx = new Fl_Output(0, 0, 1, 1, "PtM X:");
  m_fld_ptmx->color(fcolor_beige);
  m_fld_ptmx->clear_visible_focus();

  m_fld_ptmy = new Fl_Output(0, 0, 1, 1, "PtM Y:");
  m_fld_ptmy->color(fcolor_beige);
  m_fld_ptmy->clear_visible_focus();

  //=================================================
  // Column Four - Lengths
  m_fld_clen = new Fl_Output(0, 0, 1, 1, "Bez Len:");
  m_fld_clen->color(fcolor_beige);
  m_fld_clen->clear_visible_focus();

  m_fld_mlen = new Fl_Output(0, 0, 1, 1, "Max Len:");
  m_fld_mlen->color(fcolor_beige);
  m_fld_mlen->clear_visible_focus();
}

//---------------------------------------------------------------------- 
// Procedure: resizeWidgetsShape()     

void BEZ_GUI::resizeWidgetsShape()
{
  int extra_wid = w() - m_start_wid;
  if(extra_wid < 0)
    extra_wid = 0;
  int field_hgt = 20;

  int row0 = h() - 65;
  int row1 = row0 + 25;
  //int row2 = row1 + 25;

  int col1_pos = 60;
  int col1_wid = 85;

  int col2_pos = col1_pos + col1_wid + 20;
  int col2_wid = 85;

  int col3_pos = col2_pos + col2_wid + 20;
  int col3_wid = 120 + (extra_wid/4);

  int col4_pos = col3_pos + col3_wid + 50;
  //int col4_wid = 120 + (extra_wid/4);

  //===================================================
  // Main Viewer
  //===================================================
  m_bez_viewer->resize(0, 30, w(), h()-120);

  // Column 1  --------------------------
  int ptax_x = col1_pos;
  int ptax_y = row0;
  int ptax_wid = 50;
  m_fld_ptax->resize(ptax_x, ptax_y, ptax_wid, field_hgt);

  int ptay_x = col1_pos;
  int ptay_y = row1;
  int ptay_wid = 50;
  m_fld_ptay->resize(ptay_x, ptay_y, ptay_wid, field_hgt);

  // Column 2  --------------------------
  int ptzx_x = col2_pos;
  int ptzx_y = row0;
  int ptzx_wid = 50;
  m_fld_ptzx->resize(ptzx_x, ptzx_y, ptzx_wid, field_hgt);

  int ptzy_x = col2_pos;
  int ptzy_y = row1;
  int ptzy_wid = 50;
  m_fld_ptzy->resize(ptzy_x, ptzy_y, ptzy_wid, field_hgt);

  // Column 3  --------------------------
  int ptmx_x = col3_pos;
  int ptmx_y = row0;
  int ptmx_wid = 50;
  m_fld_ptmx->resize(ptmx_x, ptmx_y, ptmx_wid, field_hgt);

  int ptmy_x = col3_pos;
  int ptmy_y = row1;
  int ptmy_wid = 50;
  m_fld_ptmy->resize(ptmy_x, ptmy_y, ptmy_wid, field_hgt);

  // Column 4  --------------------------
  int clen_x = col4_pos;
  int clen_y = row0;
  int clen_wid = 50;
  m_fld_clen->resize(clen_x, clen_y, clen_wid, field_hgt);

  int mlen_x = col4_pos;
  int mlen_y = row1;
  int mlen_wid = 50;
  m_fld_mlen->resize(mlen_x, mlen_y, mlen_wid, field_hgt);
}
  
//---------------------------------------------------------------------- 
// Procedure: resizeWidgetsText()

void BEZ_GUI::resizeWidgetsText()
{
  int text_size  = 12;
  int label_size = 12;
  
  // Column One ------------------------
  m_fld_ptax->textsize(text_size);
  m_fld_ptax->labelsize(label_size);

  m_fld_ptay->textsize(text_size);
  m_fld_ptay->labelsize(label_size);

  // Column Two ------------------------
  m_fld_ptzx->textsize(text_size);
  m_fld_ptzx->labelsize(label_size);

  m_fld_ptzy->textsize(text_size);
  m_fld_ptzy->labelsize(label_size);

  // Column Three ------------------------
  m_fld_ptmx->textsize(text_size);
  m_fld_ptmx->labelsize(label_size);

  m_fld_ptmy->textsize(text_size);
  m_fld_ptmy->labelsize(label_size);

  // Column Four ------------------------
  m_fld_clen->textsize(text_size);
  m_fld_clen->labelsize(label_size);

  m_fld_mlen->textsize(text_size);
  m_fld_mlen->labelsize(label_size);
}

//---------------------------------------------------------- 
// Procedure: resize   

void BEZ_GUI::resize(int lx, int ly, int lw, int lh)
{
  Fl_Window::resize(lx, ly, lw, lh);
  resizeWidgetsShape();
  resizeWidgetsText();
}

//----------------------------------------------------------------
// Procedure: augmentMenu

void BEZ_GUI::augmentMenu() 
{
  //==============================================================
  // The File SubMenu
  //==============================================================
  //m_menubar->add("File/dump cmdline args", 'd',
  //		 (Fl_Callback*)BEZ_GUI::cb_DumpCmdLineArgs, (void*)0,
  //		 FL_MENU_DIVIDER);
  
  //==============================================================
  // The BackView SubMenu
  //==============================================================
  // First we remove some items at the superclass level so we can use the 
  // hot keys differently. 

  removeMenuItem("BackView/Zoom Reset");

  removeMenuItem("BackView/Pan Up (v. slow) ");
  removeMenuItem("BackView/Pan Down (v. slow) ");
  removeMenuItem("BackView/Pan Left (v. slow) ");
  removeMenuItem("BackView/Pan Right (v. slow)");

  //====================================================================
  // The Curve SubMenu
  //====================================================================
  m_menubar->add("Sim/Forward 1 Sec", ']',
		 (Fl_Callback*)BEZ_GUI::cb_StepForward, (void*)1, 0);
  m_menubar->add("Sim/Forward Port 1 Sec", FL_CTRL+']',
		 (Fl_Callback*)BEZ_GUI::cb_StepForwardPort, (void*)1, 0);
  m_menubar->add("Sim/Forward Star 1 Sec", FL_ALT + ']',
		 (Fl_Callback*)BEZ_GUI::cb_StepForwardStar, (void*)1, 0);

  m_menubar->add("Curve/RotateLeft", '{',
		 (Fl_Callback*)BEZ_GUI::cb_RotateCurve, (void*)-5, 0);
  m_menubar->add("Curve/RotateRight", '}',
		 (Fl_Callback*)BEZ_GUI::cb_RotateCurve, (void*)5, 0);

  m_menubar->add("Curve/Up", FL_SHIFT+FL_Up,
		 (Fl_Callback*)BEZ_GUI::cb_AltCurveY, (void*)10, 0);
  m_menubar->add("Curve/Down", FL_SHIFT+FL_Down,
		 (Fl_Callback*)BEZ_GUI::cb_AltCurveY, (void*)-10, 0);

  m_menubar->add("Curve/Right", FL_SHIFT+FL_Right,
		 (Fl_Callback*)BEZ_GUI::cb_AltCurveX, (void*)10, 0);
  m_menubar->add("Curve/Left", FL_SHIFT+FL_Left,
		 (Fl_Callback*)BEZ_GUI::cb_AltCurveX, (void*)-10, 0);

  m_menubar->add("Curve/Longer", ')',
		 (Fl_Callback*)BEZ_GUI::cb_AltCurveLen, (void*)2, 0);
  m_menubar->add("Curve/Shorter", '(',
		 (Fl_Callback*)BEZ_GUI::cb_AltCurveLen, (void*)-2, 0);

  m_menubar->add("Curve/curve_viewable=true",  0,
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)570,
		 FL_MENU_RADIO|FL_MENU_VALUE);
  m_menubar->add("Curve/curve_viewable=false", 0,
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)571,
		 FL_MENU_RADIO);
  m_menubar->add("Curve/Toggle Curve",      'C',
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)572,
		 FL_MENU_DIVIDER);
  m_menubar->add("Curve/Center View On Curve",      'c',
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)573,
		 FL_MENU_DIVIDER);

  m_menubar->add("Curve/curve_viewable_labels=true",  0,
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)580,
		 FL_MENU_RADIO|FL_MENU_VALUE);
  m_menubar->add("Curve/curve_viewable_labels=false", 0,
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)581, FL_MENU_RADIO);
  m_menubar->add("Curve/    Toggle Curve Label",   'C',
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)592, FL_MENU_DIVIDER);


  
  //====================================================================
  // The GeoAttr SubMenu
  //====================================================================

  // --------------------------------- Points
  m_menubar->add("GeoAttr/Points/point_viewable_all=true",  0,
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)550,
		 FL_MENU_RADIO|FL_MENU_VALUE);
  m_menubar->add("GeoAttr/Points/point_viewable_all=false", 0,
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)551,
		 FL_MENU_RADIO);
  m_menubar->add("GeoAttr/Points/    Toggle Points",      'j',
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)552,
		 FL_MENU_DIVIDER);

  m_menubar->add("GeoAttr/Points/point_viewable_labels=true",  0,
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)560,
		 FL_MENU_RADIO|FL_MENU_VALUE);
  m_menubar->add("GeoAttr/Points/point_viewable_labels=false", 0,
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)561, FL_MENU_RADIO);
  m_menubar->add("GeoAttr/Points/    Toggle Point Labels",   'J',
		 (Fl_Callback*)BEZ_GUI::cb_SetGeoAttr, (void*)562, FL_MENU_DIVIDER);

}

//----------------------------------------------------------
// Procedure: handle

int BEZ_GUI::handle(int event) 
{
  switch(event) {
  case FL_PUSH:
    Fl_Window::handle(event);
    updateXY();
    return(1);
    break;
  default:
    return(Fl_Window::handle(event));
  }
}

//----------------------------------------- StepForward
inline void BEZ_GUI::cb_StepForward_i(int amt) {
  m_bez_viewer->setRudder(0);
  m_bez_viewer->stepForward(amt);
  m_bez_viewer->initCenterView();
  m_bez_viewer->redraw();
  updateXY();
}
void BEZ_GUI::cb_StepForward(Fl_Widget* o, int v) {
  ((BEZ_GUI*)(o->parent()->user_data()))->cb_StepForward_i(v);
}

//----------------------------------------- StepForwardPort
inline void BEZ_GUI::cb_StepForwardPort_i(int amt) {
  m_bez_viewer->setRudder(-4);
  m_bez_viewer->stepForward(amt);
  m_bez_viewer->initCenterView();
  m_bez_viewer->redraw();
  updateXY();
}
void BEZ_GUI::cb_StepForwardPort(Fl_Widget* o, int v) {
  ((BEZ_GUI*)(o->parent()->user_data()))->cb_StepForwardPort_i(v);
}

//----------------------------------------- StepForwardStar
inline void BEZ_GUI::cb_StepForwardStar_i(int amt) {
  m_bez_viewer->setRudder(4);
  m_bez_viewer->stepForward(amt);
  m_bez_viewer->initCenterView();
  m_bez_viewer->redraw();
  updateXY();
}
void BEZ_GUI::cb_StepForwardStar(Fl_Widget* o, int v) {
  ((BEZ_GUI*)(o->parent()->user_data()))->cb_StepForwardStar_i(v);
}

//----------------------------------------- RotateCurve
inline void BEZ_GUI::cb_RotateCurve_i(int amt) {
  m_bez_viewer->m_tbm.rotate(amt);
  m_bez_viewer->redraw();
  updateXY();
}
void BEZ_GUI::cb_RotateCurve(Fl_Widget* o, int v) {
  ((BEZ_GUI*)(o->parent()->user_data()))->cb_RotateCurve_i(v);
}

//----------------------------------------- AltCurveX
inline void BEZ_GUI::cb_AltCurveX_i(int amt) {
  m_bez_viewer->m_tbm.shiftX(amt);
  m_bez_viewer->redraw();
  updateXY();
} 
void BEZ_GUI::cb_AltCurveX(Fl_Widget* o, int v) {
  ((BEZ_GUI*)(o->parent()->user_data()))->cb_AltCurveX_i(v);
}

//----------------------------------------- AltCurveY
inline void BEZ_GUI::cb_AltCurveY_i(int amt) {
  m_bez_viewer->m_tbm.shiftY(amt);
  m_bez_viewer->redraw();
  updateXY();
}
void BEZ_GUI::cb_AltCurveY(Fl_Widget* o, int v) {
  ((BEZ_GUI*)(o->parent()->user_data()))->cb_AltCurveY_i(v);
}


//----------------------------------------- AltCurveLen
inline void BEZ_GUI::cb_AltCurveLen_i(int amt) {
  m_bez_viewer->m_tbm.altTowLineLen(amt);
  m_bez_viewer->redraw();
  updateXY();
}
void BEZ_GUI::cb_AltCurveLen(Fl_Widget* o, int v) {
  ((BEZ_GUI*)(o->parent()->user_data()))->cb_AltCurveLen_i(v);
}


//----------------------------------------- SetGeoAttr
inline void BEZ_GUI::cb_SetGeoAttr_i(int v) {
  if(v == 550)
    m_bez_viewer->setDrawPoints(true);
  else if(v == 551)
    m_bez_viewer->setDrawPoints(false);
  else if(v == 552)
    m_bez_viewer->toggleDrawPoints();

  if(v == 570)
    m_bez_viewer->setDrawCurve(true);
  else if(v == 571)
    m_bez_viewer->setDrawCurve(false);
  else if(v == 572)
    m_bez_viewer->toggleDrawCurve();
  else if(v == 573) 
    m_bez_viewer->initCenterView();
  
  m_bez_viewer->redraw();
  updateXY();
}
void BEZ_GUI::cb_SetGeoAttr(Fl_Widget* o, int v) {
  ((BEZ_GUI*)(o->parent()->user_data()))->cb_SetGeoAttr_i(v);
}


//------------------------------------------------------
// Procedure  UpdateXY()

void BEZ_GUI::updateXY() 
{
  string sval;

  // Column (1) Pt A  ------------------------
  double ax = m_bez_viewer->m_tbm.getPtAX();
  sval = doubleToStringX(ax, 1);
  m_fld_ptax->value(sval.c_str());

  double ay = m_bez_viewer->m_tbm.getPtAY();
  sval = doubleToStringX(ay, 1);
  m_fld_ptay->value(sval.c_str());
  
  // Column (2) Pt Z  ------------------------
  double zx = m_bez_viewer->m_tbm.getPtZX();
  sval = doubleToStringX(zx, 1);
  m_fld_ptzx->value(sval.c_str());

  double zy = m_bez_viewer->m_tbm.getPtZY();
  sval = doubleToStringX(zy, 1);
  m_fld_ptzy->value(sval.c_str());

    // Column (3) Pt M  ----------------------
  double mx = m_bez_viewer->m_tbm.getPtMX();
  sval = doubleToStringX(mx, 1);
  m_fld_ptmx->value(sval.c_str());

  double my = m_bez_viewer->m_tbm.getPtMY();
  sval = doubleToStringX(my, 1);
  m_fld_ptmy->value(sval.c_str());

  // Column (4) Curve Lens -------------------
  double clen = m_bez_viewer->m_tbm.getCurveLen();
  sval = doubleToStringX(clen, 1);
  m_fld_clen->value(sval.c_str());

  double mlen = m_bez_viewer->m_tbm.getTowLineLen();
  sval = doubleToStringX(mlen, 1);
  m_fld_mlen->value(sval.c_str());
}






