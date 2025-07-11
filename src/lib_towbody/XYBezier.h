/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: XYBezier.h                                           */
/*    DATE: Apr 15th 2020                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef BEZIER_XY_HEADER
#define BEZIER_XY_HEADER

#include <string>
#include "XYObject.h"

class XYBezier : public XYObject {
public:
  XYBezier(double ax=0, double ay=0, double zx=0, double zy=0);
  virtual ~XYBezier() {}

  void   set(double ax, double ay, double zx, double zy,
	     double mx, double my);
  void   setPtA(double ax, double ay);
  void   setPtZ(double ax, double ay);
  void   setPtM(double ax, double ay);

  void   shiftX(double v) {m_ax+=v; m_zx+=v; m_mx+=v;}
  void   shiftY(double v) {m_ay+=v; m_zy+=v; m_my+=v;}
  void   rotate(double v);
  
  double getPtAX() const         {return(m_ax);}
  double getPtAY() const         {return(m_ay);}
  double getPtZX() const         {return(m_zx);}
  double getPtZY() const         {return(m_zy);}
  double getPtMX() const         {return(m_mx);}
  double getPtMY() const         {return(m_my);}

  double getMinX() const;
  double getMaxX() const;
  double getMinY() const;
  double getMaxY() const;
  double getCenterX() const;
  double getCenterY() const;

  double getCurveLen() const;
  
  std::string get_spec_raw(std::string p="", unsigned int prec=1) const;
  std::string get_spec_cache(std::string p="", unsigned int prec=1) const;
  
  void                setPointCache(unsigned int);
  std::vector<double> getPointCacheX() const {return(m_pt_cache_x);}
  std::vector<double> getPointCacheY() const {return(m_pt_cache_y);}

  double clipCurveByMaxLen(double maxlen);
  void   resetPtZToEndOfCurve();
  double getFirstSegHeading();
  double getLastSegHeading();
  
protected:
  void   rotatePoint(double deg, double cx, double cy,
		     double &px, double& py);
  void   linearComb(double px, double py, double qx, double qy,
		    double pct, double& rx, double &ry);
  
protected:
  double   m_ax;
  double   m_ay;
  double   m_zx;
  double   m_zy;
  double   m_mx;
  double   m_my;

  int      m_sdigits;
  
  std::vector<double> m_pt_cache_x;
  std::vector<double> m_pt_cache_y;

};
#endif























