/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: XYBezier.cpp                                         */
/*    DATE: Apr 15th 2020                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cmath>
#include <cstdlib>
#include "XYBezier.h"
#include "GeomUtils.h"
#include "AngleUtils.h"
#include "MBUtils.h"

using namespace std;

//-------------------------------------------------------------
// Procedure: Constructor

XYBezier::XYBezier(double ax, double ay, double zx, double zy)
{
  m_ax  = ax;
  m_ay  = ay;
  m_mx  = ax;
  m_my  = ay;
  m_zx  = zx;
  m_zy  = zy;  

  m_sdigits = 2;
}

//-------------------------------------------------------------
// Procedure: set()

void XYBezier::set(double ax, double ay, double zx, double zy,
		   double mx, double my)
{
  m_ax = ax;
  m_ay = ay;
  m_zx = zx;
  m_zy = zy;
  m_mx = mx;
  m_my = my;
}

//-------------------------------------------------------------
// Procedure: setPtA()

void XYBezier::setPtA(double ax, double ay)
{
  m_ax = ax;
  m_ay = ay;
}

//-------------------------------------------------------------
// Procedure: setPtZ()

void XYBezier::setPtZ(double zx, double zy)
{
  m_zx = zx;
  m_zy = zy;
}

//-------------------------------------------------------------
// Procedure: setPtM()

void XYBezier::setPtM(double mx, double my)
{
  m_mx = mx;
  m_my = my;
}

//---------------------------------------------------------------
// Procedure: getMinX()

double XYBezier::getMinX() const
{
  double minx = m_ax;
  if(m_zx < minx)
    minx = m_zx;
  if(m_mx < minx)
    minx = m_mx;
  return(minx);
}

//---------------------------------------------------------------
// Procedure: getMaxX()

double XYBezier::getMaxX() const
{
  double maxx = m_ax;
  if(m_zx > maxx)
    maxx = m_zx;
  if(m_mx > maxx)
    maxx = m_mx;
  return(maxx);
}

//---------------------------------------------------------------
// Procedure: getMinY()

double XYBezier::getMinY() const
{
  double miny = m_ay;
  if(m_zy < miny)
    miny = m_zy;
  if(m_my < miny)
    miny = m_my;
  return(miny);
}

//---------------------------------------------------------------
// Procedure: getMaxY()

double XYBezier::getMaxY() const
{
  double maxy = m_ay;
  if(m_zy > maxy)
    maxy = m_zy;
  if(m_my > maxy)
    maxy = m_my;
  return(maxy);
}

//---------------------------------------------------------------
// Procedure: getCenterX()

double XYBezier::getCenterX() const
{
  double min_x = getMinX();
  double max_x = getMaxX();

  if(min_x >= max_x)
    return(min_x);

  return(((max_x - min_x)/2) + min_x);
}

//---------------------------------------------------------------
// Procedure: getCenterY()

double XYBezier::getCenterY() const
{
  double min_y = getMinY();
  double max_y = getMaxY();

  if(min_y >= max_y)
    return(min_y);

  return(((max_y - min_y)/2) + min_y);
}

//---------------------------------------------------------------
// Procedure: rotate

void XYBezier::rotate(double deg)
{
  double cx = (getMaxX() + getMinX()) / 2;
  double cy = (getMaxY() + getMinY()) / 2;
  
  rotatePoint(deg, cx, cy, m_ax, m_ay);
  rotatePoint(deg, cx, cy, m_zx, m_zy);
  rotatePoint(deg, cx, cy, m_mx, m_my);

  for(unsigned int i=0; i<m_pt_cache_x.size(); i++)
    rotatePoint(deg, cx, cy, m_pt_cache_x[i], m_pt_cache_y[i]);
}

//---------------------------------------------------------------
// Procedure: rotatePoint

void XYBezier::rotatePoint(double deg, double cx, double cy, 
			   double &px, double &py)
{
  double curr_dist  = hypot((cx-px), (cy-py));
  double curr_angle = relAng(cx, cy, px, py);
  double new_angle  = curr_angle + deg;

  double nx, ny;

  projectPoint(new_angle, curr_dist, cx, cy, nx, ny);

  px = nx; 
  py = ny;
}


//---------------------------------------------------------------
// Procedure: getCurveLen() 

double XYBezier::getCurveLen() const
{
  unsigned int cache_size = m_pt_cache_x.size();
  if(cache_size == 0)
    return(0);
  
  double len = 0;
  for(unsigned int i=0; i<cache_size; i++) {
    double cx = m_pt_cache_x[i];
    double cy = m_pt_cache_y[i];
    if(i == 0) 
      len += hypot(m_ax - cx, m_ay - cy);
    else
      len += hypot(cx - m_pt_cache_x[i-1], cy - m_pt_cache_y[i-1]);
  }

  len += hypot(m_zx - m_pt_cache_x[cache_size-1],
	       m_zy - m_pt_cache_y[cache_size-1]);
  
  return(len);  
}


//---------------------------------------------------------------
// Procedure: clipCurveByMaxLen()

double XYBezier::clipCurveByMaxLen(double maxlen)
{
  if(m_pt_cache_x.size() == 0)
    return(0);

  double orig_len = getCurveLen();
  
  vector<double> new_cache_x;
  vector<double> new_cache_y;
  
  double total_len = 0;
  for(unsigned int i=0; i<m_pt_cache_x.size(); i++) {
    double cx = m_pt_cache_x[i];
    double cy = m_pt_cache_y[i];
    double px = m_ax; // prev_x
    double py = m_ay; // prev_y
    if(i > 0) {
      px = m_pt_cache_x[i-1];
      py = m_pt_cache_y[i-1];
    }
    double leglen = hypot(cx-px, cy-py);

    // If the new point fits, just add it
    if((total_len + leglen) <= maxlen) {
      new_cache_x.push_back(cx);
      new_cache_y.push_back(cy);
      total_len += leglen;
    }
    // Otherise just add a portion of the leg
    else {
      double rem_dist = (maxlen - total_len);
      double pct = rem_dist / leglen;
      double endx, endy;
      linearComb(px,py, cx,cy, pct, endx,endy);
      new_cache_x.push_back(endx);
      new_cache_y.push_back(endy);
      total_len += rem_dist;
      break;
    }
  }

  m_pt_cache_x = new_cache_x;
  m_pt_cache_y = new_cache_y;

  return(maxlen - orig_len);  
}


//---------------------------------------------------------------
// Procedure: resetPtZToEndOfCurve()

void XYBezier::resetPtZToEndOfCurve()
{
  if((m_pt_cache_x.size() == 0) ||
     (m_pt_cache_y.size() == 0))
    return;
  
  m_zx = m_pt_cache_x.back();
  m_zy = m_pt_cache_y.back();
}

//---------------------------------------------------------------
// Procedure: getFirstSegHeading()
//   Purpose: Get the relative angle between from the last point
//            in the curve cache to the second-to-last point.

double XYBezier::getFirstSegHeading()
{
  unsigned int xlen = m_pt_cache_x.size();
  unsigned int ylen = m_pt_cache_y.size();
  
  if((xlen != ylen) || (xlen < 2))
    return(0);

  double x1 = m_pt_cache_x[1];
  double y1 = m_pt_cache_y[1];
  double x2 = m_pt_cache_x[0];
  double y2 = m_pt_cache_y[0];
  
  double ang = relAng(x1,y1, x2,y2);
  return(ang);
}

//---------------------------------------------------------------
// Procedure: getLastSegHeading()
//   Purpose: Get the relative angle between from the last point
//            in the curve cache to the second-to-last point.

double XYBezier::getLastSegHeading()
{
  unsigned int xlen = m_pt_cache_x.size();
  unsigned int ylen = m_pt_cache_y.size();
  
  if((xlen != ylen) || (xlen < 2))
    return(0);

  double x1 = m_pt_cache_x[xlen-1];
  double y1 = m_pt_cache_y[ylen-1];
  double x2 = m_pt_cache_x[xlen-2];
  double y2 = m_pt_cache_y[ylen-2];
  
  double ang = relAng(x1,y1, x2,y2);
  return(ang);
}

//---------------------------------------------------------------
// Procedure: setPointCache()

void XYBezier::setPointCache(unsigned int amt)
{
  if(amt == 0)
    return;
  m_pt_cache_x.clear();
  m_pt_cache_y.clear();
  
  m_pt_cache_x.push_back(m_ax);
  m_pt_cache_y.push_back(m_ay);
  
  double delta = 1 / (double)(amt+1);
  for(unsigned int i=0; i<amt; i++) {
    double pct = 1 - (((double)(i+1)) * delta);

    double pt1x, pt1y, pt2x, pt2y, pt3x, pt3y;
    linearComb(m_zx, m_zy, m_mx, m_my, pct, pt1x, pt1y);
    linearComb(m_mx, m_my, m_ax, m_ay, pct, pt2x, pt2y);
    linearComb(pt1x, pt1y, pt2x, pt2y, pct, pt3x, pt3y);
    
    m_pt_cache_x.push_back(pt3x);
    m_pt_cache_y.push_back(pt3y);
  }

  m_pt_cache_x.push_back(m_zx);
  m_pt_cache_y.push_back(m_zy);
}

//---------------------------------------------------------------
// Procedure: linearComb()

void XYBezier::linearComb(double px, double py, double qx, double qy,
			  double pct, double& rx, double& ry)
{
  if(pct < 0)
    pct = 0;
  else if(pct > 1)
    pct = 1;
  
  double delta_x = px - qx;
  double delta_y = py - qy;
  if(delta_x < 0)
    delta_x = -delta_x;
  if(delta_y < 0)
    delta_y = -delta_y;

  delta_x = delta_x * pct;
  delta_y = delta_y * pct;
  
  if(px == qx)
    rx = px;
  else if(px < qx)
    rx = px + delta_x;
  else
    rx = px - delta_x;
  
  if(py == qy)
    ry = py;
  else if(py < qy)
    ry = py + delta_y;
  else
    ry = py - delta_y;
}


//-------------------------------------------------------------
// Procedure: get_spec_raw()

string XYBezier::get_spec_raw(string param, unsigned precision) const
{
  string spec;
  spec += "ax=";
  spec += doubleToStringX(m_ax, precision) + ","; 
  spec += "ay=";
  spec += doubleToStringX(m_ay, precision) + ","; 
  spec += "zx=";
  spec += doubleToStringX(m_zx, precision) + ","; 
  spec += "zy=";
  spec += doubleToStringX(m_zy, precision) + ","; 
  spec += "mx=";
  spec += doubleToStringX(m_mx, precision) + ","; 
  spec += "my=";
  spec += doubleToStringX(m_my, precision); 

  string obj_spec = XYObject::get_spec(param);
  if(obj_spec != "")
    spec += ("," + obj_spec);
  
  return(spec);
}

//---------------------------------------------------------------
// Procedure: get_spec_cache()

string XYBezier::get_spec_cache(string param, unsigned int precision) const
{
  string spec;

  // Clip the precision to be at most 6
  if(precision > 6)
    precision = 6;

  unsigned int i, vsize = m_pt_cache_x.size();
  if(vsize > 0)
    spec += "pts={";

  for(i=0; i<vsize; i++) {
    spec += doubleToStringX(m_pt_cache_x[i], precision);
    spec += ",";
    spec += doubleToStringX(m_pt_cache_y[i], precision);
    if(i != vsize-1)
      spec += ":";
    else
      spec += "}";
  }
  string obj_spec = XYObject::get_spec(param);
  if(obj_spec != "") {
    if(spec != "")
      spec += ",";
    spec += obj_spec;
  }

  return(spec);
}







