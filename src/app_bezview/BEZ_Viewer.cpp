/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BEZ_Viewer.cpp                                       */
/*    DATE: Apr 17th 2020                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <unistd.h>
#include <iostream>
#include <cmath>
#include "BEZ_Viewer.h"
#include "BearingLine.h"
#include "SimEngine.h"
#include "MBUtils.h"
#include "GeomUtils.h"
#include "AngleUtils.h"
#include "XYWedge.h"


using namespace std;

//----------------------------------------------------------------
// Constructor

BEZ_Viewer::BEZ_Viewer(int x, int y, int wid, int hgt,
		       const char *label) :
  MarineViewer(x, y, wid, hgt, label)
{
  // Initialize config params
  m_pt_color = "yellow";
  m_pt_size  = 6;
  
  m_vshift_x = -80;
  m_vshift_y = -60;

  //========================================================
  // Override some default values of MarineViewer superclass
  //========================================================
  
  m_zoom = 0.6;

  m_tbm.setPtA(65,50);
  m_tbm.setPtZ(2,2);
  m_tbm.setPtM(20,45);
  m_tbm.shiftY(-100);
  m_tbm.initOwnshipToTowLine();  
  
  m_draw_points = true;
  m_draw_curve = true;

  m_thrust_map.addPair(0, 0);
  m_thrust_map.addPair(20, 1);
  m_thrust_map.addPair(40, 2);
  m_thrust_map.addPair(60, 3);
  m_thrust_map.addPair(80, 4);
  m_thrust_map.addPair(100, 5);
  m_turn_rate = 60;
  m_thrust = 50;
  m_rudder = 0;
  m_curr_time = 0;

  m_init_center_view = false;
  
  setParam("tiff_view", "on");
  setParam("hash_viewable", "false");
  setParam("hash_shade", -1.0);
  setParam("hash_shade", 0.75);
  setParam("back_shade", -1.0);
  setParam("back_shade", 0.85);
}

//-------------------------------------------------------------
// Procedure: handle()

int BEZ_Viewer::handle(int event)
{
  int vx, vy;
  switch(event) {
  case FL_PUSH:
    vx = Fl::event_x();
    vy = h() - Fl::event_y();
    if(Fl_Window::handle(event) != 1) {
      if(Fl::event_button() == FL_LEFT_MOUSE)
	handle_left_mouse(vx, vy);
      if(Fl::event_button() == FL_RIGHT_MOUSE)
	handle_right_mouse(vx, vy);
    }
    return(1);
    break;  
  default:
    return(Fl_Gl_Window::handle(event));
  }
}

//-------------------------------------------------------------
// Procedure: draw()

void BEZ_Viewer::draw()
{
  MarineViewer::draw();
  if(!m_init_center_view)
    initCenterView();  

  drawBezier();
  drawVehicle();
  drawTowBody();
}

//-------------------------------------------------------------
// Procedure: handle_left_mouse()

void BEZ_Viewer::handle_left_mouse(int vx, int vy)
{
  double ix = view2img('x', vx);
  
  double iy = view2img('y', vy);
  double mx = img2meters('x', ix);
  double my = img2meters('y', iy);
  double sx = snapToStep(mx, 0.1);
  double sy = snapToStep(my, 0.1);

  cout << "sx: " << doubleToStringX(sx,1) <<
    ", sy: " << doubleToStringX(sy,1) << endl;

  if(Fl::event_state(FL_SHIFT)) {
    m_tbm.setPtA(sx, sy);
    m_tbm.initOwnshipToTowLine();
  }
  else if(Fl::event_state(FL_ALT)) {
    m_tbm.setPtZ(sx, sy);
    m_tbm.initOwnshipToTowLine();
  }
  else {
    m_tbm.setPtM(sx, sy);
    m_tbm.initOwnshipToTowLine();
  }

  redraw();
}


//----------------------------------------------------------------
// Procedure: addConfigParam()

void BEZ_Viewer::addConfigParam(string str)
{
  m_config_params.push_back(str);
}

//----------------------------------------------------------------
// Procedure: addPostConfigParam()

void BEZ_Viewer::addPostConfigParam(string str)
{
  m_post_config_params.push_back(str);
}


//----------------------------------------------------------------
// Procedure: handleConfigParams()

bool BEZ_Viewer::handleConfigParams()
{
  if(vectorContains(m_config_params, "-v") ||
     vectorContains(m_config_params, "--verbose"))
    setVerbose(true);

  bool tif_set = false;
  
  for(unsigned int i=0; i<m_config_params.size(); i++){
    string orig = m_config_params[i];
    string argi = m_config_params[i];
    string left = biteStringX(argi, '=');
    string val  = argi;

    bool ok_param = false;
    if(strEnds(left, ".tif") || strEnds(left, ".tiff")) {
      ok_param = setTiffFile(left);
      if(ok_param)
	tif_set = true;
    }
    else if((left == "-v") || (left == "--verbose"))
      ok_param = true;
      
    if(!ok_param) {
      cout << "Bad Config param: [" << orig << "]" << endl;
      return(false);
    } 
  }

  if(!tif_set)
    setTiffFile("MIT_SP.tif");

  return(true);
}


//----------------------------------------------------------------
// Procedure: handlePostConfigParams()

bool BEZ_Viewer::handlePostConfigParams()
{
  for(unsigned int i=0; i<m_post_config_params.size(); i++) {
    string orig = m_post_config_params[i];
    string argi = m_post_config_params[i];
    string left = biteStringX(argi, '=');
    string val  = argi;

    bool ok_param = false;
    if(left == "--point_color")
      ok_param = setPointColor(val);
    else if(left == "--point_size")
      ok_param = setPointSize(val);
    
    if(!ok_param) {
      cout << "Bad PostConfig param: [" << orig << "]" << endl;
      return(false);
    }
  }
  
  return(true);
}


//-------------------------------------------------------------
// Procedure: setThrust()

void BEZ_Viewer::setThrust(double val)
{
  if(val < 0)
    val = 0;
  else if(val > 100)
    val = 100;
  m_thrust = val;
}

//-------------------------------------------------------------
// Procedure: setRudder()

void BEZ_Viewer::setRudder(double val)
{
  if(val < -100)
    val = -100;
  else if(val > 100)
    val = 100;
  m_rudder = val;
}

//-------------------------------------------------------------
// Procedure: stepForward()

void BEZ_Viewer::stepForward(double secs)
{
  NodeRecord record = m_tbm.getNodeRecord(); 

  double osh = record.getHeading();
  double osv = record.getSpeed();
  
  SimEngine engine;
  engine.propagate(record, secs, osh, osv, 0, 0);

  engine.propagateSpeed(record, m_thrust_map, secs, m_thrust,
			m_rudder, 0, 0.5);

  engine.propagateHeading(record, secs, m_rudder, m_thrust,
			  m_turn_rate, 0);

  m_curr_time += secs;
  m_tbm.update(record, m_curr_time);
}


//-------------------------------------------------------------
// Procedure: drawBezier()

void BEZ_Viewer::drawBezier()
{
  XYBezier bez = m_tbm.getTowLine();
  if(m_draw_points) {
    double ax = bez.getPtAX();
    double ay = bez.getPtAY();
    double zx = bez.getPtZX();
    double zy = bez.getPtZY();
    double mx = bez.getPtMX();
    double my = bez.getPtMY();
    
    XYPoint pta(ax,ay);
    XYPoint ptz(zx,zy);
    XYPoint ptm(mx,my);
    
    pta.set_label("a");
    ptz.set_label("z");
    ptm.set_label("m");
  
    pta.set_color("vertex", "yellow");
    ptz.set_color("vertex", "yellow");
    ptm.set_color("vertex", "yellow");
    
    pta.set_vertex_size(m_pt_size);
    ptz.set_vertex_size(m_pt_size);
    
    drawPoint(pta);
    drawPoint(ptz);
    drawPoint(ptm);
  }

  if(m_draw_curve) {
    // Build a SegList from the Bezier point cache
    bez.setPointCache(25);
    //bez.clipCurveByMaxLen(m_max_len);
    
    XYSegList segl;
    vector<double> pts_x = bez.getPointCacheX();
    vector<double> pts_y = bez.getPointCacheY();
    for(unsigned int i=0; i<pts_x.size(); i++) {
      segl.add_vertex(pts_x[i], pts_y[i]);
    }
    segl.set_label("bez");
    segl.set_label_color("invisible");
    segl.set_vertex_size(0);
    segl.set_edge_color("gray50");
    drawSegList(segl);
  }
}


//-------------------------------------------------------------
// Procedure: drawVehicle()

void BEZ_Viewer::drawVehicle()
{
  NodeRecord record = m_tbm.getNodeRecord();

  //BearingLine bng_line;

  ColorPack vehi_color("dodger_blue");
  ColorPack vname_color("white");
  bool vname_draw = false;
  
  //drawCommonVehicle(record, bng_line, vehi_color, vname_color, vname_draw, 1);
  drawCommonVehicle(record, vehi_color, vname_color, vname_draw, 1);
}

//-------------------------------------------------------------
// Procedure: drawTowBody()

void BEZ_Viewer::drawTowBody()
{
  NodeRecord record;
  record.setX(m_tbm.getTBX());
  record.setY(m_tbm.getTBY());
  record.setHeading(m_tbm.getTBH());
  record.setType("glider");
  record.setLength(9);

  // BearingLine bng_line;

  ColorPack vehi_color("white");
  ColorPack vname_color("white");
  bool vname_draw = false;
  
  //drawCommonVehicle(record, bng_line, vehi_color, vname_color, vname_draw, 1);
  drawCommonVehicle(record, vehi_color, vname_color, vname_draw, 1);
}

//---------------------------------------------------------
// Procedure: setTiffFile()

bool BEZ_Viewer::setTiffFile(string tif_file)
{
  bool ok_tiff = setParam("tiff_file", tif_file);
  if(!ok_tiff)
    return(false);

  return(true);
}


//---------------------------------------------------------
// Procedure: setPointColor()

bool BEZ_Viewer::setPointColor(string colorstr)
{
  if(colorstr == "")
    return(true);

  if(!isColor(colorstr))
    return(false);

  m_pt_color = colorstr;
  
  return(true);
}

//---------------------------------------------------------
// Procedure: setPointSize()

bool BEZ_Viewer::setPointSize(string psize)
{
  if(psize == "")
    return(true);

  double dval = atof(psize.c_str());

  if(dval < 1)
    dval = 1;
  if(dval > 20)
    dval = 20;

  m_pt_size = dval;

  return(true);
}

//----------------------------------------------------------------
// Procedure: initCenterView()

void BEZ_Viewer::initCenterView()
{
  double cx = m_tbm.getCenterX();
  double cy = m_tbm.getCenterY();

  // First determine how much we're off in terms of meters
  double delta_x = cx - m_back_img.get_x_at_img_ctr();
  double delta_y = cy - m_back_img.get_y_at_img_ctr();

  // Next determine how much in terms of pixels
  double pix_per_mtr_x = m_back_img.get_pix_per_mtr_x();
  double pix_per_mtr_y = m_back_img.get_pix_per_mtr_y();

  double x_pixels = pix_per_mtr_x * delta_x;
  double y_pixels = pix_per_mtr_y * delta_y;

  m_vshift_x = -x_pixels;
  m_vshift_y = -y_pixels;
}





