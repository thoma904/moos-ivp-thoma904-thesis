/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: TowBodyModel.cpp                                     */
/*    DATE: Apr 19th 2020                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iostream>
#include "TowBodyModel.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "GeomUtils.h"

using namespace std;

//----------------------------------------------------------------
// Constructor

TowBodyModel::TowBodyModel()
{
  m_tbx = 0;
  m_tby = 0;
  m_tbh = 0;
  m_towline_len = 125;
  m_towbody_len = 5;

  m_curve_pts = 50;

  m_dist = 10;
  
  m_ownship.setLength(20);
  m_ownship.setType("kayak");
}

//----------------------------------------------------------------
// Procedure: setPtA()

void TowBodyModel::setPtA(double ax, double ay)
{
  m_towline.setPtA(ax, ay);
  m_ownship.setX(ax);
  m_ownship.setY(ay);  
}


//----------------------------------------------------------------
// Procedure: setPtM()

void TowBodyModel::setPtM(double mx, double my)
{
  m_towline.setPtM(mx, my);
}

//----------------------------------------------------------------
// Procedure: setPtZ()

void TowBodyModel::setPtZ(double zx, double zy)
{
  m_towline.setPtZ(zx, zy);
}


//----------------------------------------------------------------
// Procedure: setTowBodyLen()

void TowBodyModel::setTowBodyLen(double v)
{
  m_towbody_len = v;
  if(m_towbody_len < 1)
    m_towbody_len = 1;
}

//----------------------------------------------------------------
// Procedure: setTowLineLen()

void TowBodyModel::setTowLineLen(double v)
{
  m_towline_len = v;
  if(m_towline_len < 1)
    m_towline_len = 1;
}

//----------------------------------------------------------------
// Procedure: initOwnshipToTowLine()
//   Purpose: Normally the towline is determined by ownship, but
//            this function allows ownship position and heading to
//            be set based on the given towline.

void TowBodyModel::initOwnshipToTowLine()
{
  double osx = m_towline.getPtAX();
  double osy = m_towline.getPtAY();

  m_towline.setPointCache(m_curve_pts);
  double osh = m_towline.getFirstSegHeading();

  m_ownship.setX(osx);
  m_ownship.setY(osy);
  m_ownship.setHeading(osh);
}

//----------------------------------------------------------------
// Procedure: initTowLineToOwnship()

void TowBodyModel::initTowLineToOwnship(double osx, double osy, double osh)
{
  m_ownship.setX(osx);
  m_ownship.setY(osy);
  m_ownship.setHeading(osh);

  double mx,my;
  double aft_hdg = angle360(osh + 180);
  projectPoint(m_towline_len/2, aft_hdg, osx, osy, mx, my);
  projectPoint(m_towline_len/2, aft_hdg, osx, osy, m_tbx, m_tby);

  m_tbh = osh;

  m_towline.setPtA(osx, osy);
  m_towline.setPtM(mx, my);
  m_towline.setPtZ(m_tbx, m_tby);
  m_towline.setPointCache(m_curve_pts);

  m_delta_hdgs.clear();
  m_delta_tstamps.clear();
}

//----------------------------------------------------------------
// Procedure: initTowLine()

void TowBodyModel::initTowLine(double osx, double osy, double osh,
			       double cnx, double cny)
{
  m_ownship.setX(osx);
  m_ownship.setY(osy);
  m_ownship.setHeading(osh);

  m_tbx = cnx;
  m_tby = cny;
  
  double mx,my;
  double aft_hdg = angle360(osh + 180);
  projectPoint(aft_hdg, m_dist, osx, osy, mx, my);

  cout << "osx:" << osx << endl;
  cout << "osy:" << osy << endl;
  cout << "  mx:" << mx << endl;
  cout << "  my:" << my << endl;
  cout << "cnx:" << cnx << endl;
  cout << "cny:" << cny << endl;
  cout << "curve_pts:" << m_curve_pts << endl;
  
  m_towline.setPtA(osx, osy);
  m_towline.setPtM(mx, my);
  m_towline.setPtZ(m_tbx, m_tby);
  m_towline.setPointCache(m_curve_pts);

  m_tbh = m_towline.getLastSegHeading();

  m_towline_len = m_towline.getCurveLen();
  
  m_delta_hdgs.clear();
  m_delta_tstamps.clear();
}

//----------------------------------------------------------------
// Procedure: update()

bool TowBodyModel::update(NodeRecord record, double tstamp)
{
  // ============================================================
  // Part 1: Update ownship position and record delta heading for
  // later calculation of turn rate
  // ============================================================
  NodeRecord m_ownship_prev = m_ownship;
  m_ownship = record;

  double new_hdg = m_ownship.getHeading();
  double old_hdg = m_ownship_prev.getHeading();
  double delta = angleDiff(new_hdg, old_hdg);  

  unsigned int max_elements = 10;
  
  m_delta_hdgs.push_front(delta);
  if(m_delta_hdgs.size() > max_elements)
    m_delta_hdgs.pop_back();

  if(m_delta_tstamps.size() > 1) {
    if(tstamp < m_delta_tstamps.front()) {
      cout << "tstamp: " << tstamp << endl;
      cout << "tstamp F: " << m_delta_tstamps.front()  << endl;
      return(false);
    }
  }

  m_delta_tstamps.push_front(tstamp);
  if(m_delta_tstamps.size() > max_elements)
    m_delta_tstamps.pop_back();
  
  // ============================================================
  // Part 2: Determine the turn rate
  // ============================================================
  double total_turn_delta = 0;
  list<double>::iterator p;
  for(p=m_delta_hdgs.begin(); p!=m_delta_hdgs.end(); p++) 
    total_turn_delta += *p;

  double delta_time = 0;
  if(m_delta_tstamps.size() > 1)
    delta_time = m_delta_tstamps.front() - m_delta_tstamps.back();

  double turn_rate_degs_per_sec = 0;
  if(delta_time > 0)
    turn_rate_degs_per_sec = total_turn_delta / delta_time;

  
  // ============================================================
  // Part 3: Determine the townline midpoint. When not turning,
  //         it is half the distance of the towline, directly aft
  //         of ownship. When turning it is closer to ownship.
  // ============================================================
  double mid_dist = m_towline_len / 10;
  //double mid_dist = m_towline_len / 2;
  double max_turn_rate = 10;
  if(turn_rate_degs_per_sec > max_turn_rate)
    turn_rate_degs_per_sec = max_turn_rate;

  // Higher the turn rate, closer the mid_dist is to ownship
  double pct = turn_rate_degs_per_sec / max_turn_rate;
  if(pct < 0.1)
    pct = 0.1;
  mid_dist += pct;

  double osx  = m_ownship.getX();
  double osy  = m_ownship.getY();
  double osh  = m_ownship.getHeading();
  double oshh = angle360(osh - 180);
  double mx, my;
  projectPoint(oshh, mid_dist, osx, osy, mx, my);
  //projectPoint(oshh, m_dist, osx, osy, mx, my);

  // ============================================================
  // Part 4: Update the towline
  // ============================================================
  m_towline.setPtA(osx, osy);
  m_towline.setPtM(mx, my);
  
  m_towline.setPointCache(50);
  m_towline.clipCurveByMaxLen(m_towline_len);
  
  // ============================================================
  // Part 5: Update the towbody position and orientation
  // ============================================================
  m_towline.resetPtZToEndOfCurve();

  m_tbx = m_towline.getPtZX();
  m_tby = m_towline.getPtZY();
  m_tbh = m_towline.getLastSegHeading();
  

  return(true);
}







