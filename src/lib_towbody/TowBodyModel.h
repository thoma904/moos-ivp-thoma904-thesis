/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: TowBodyModel.h                                       */
/*    DATE: Apr 19th, 2020                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef TOW_BODY_MODEL_HEADER
#define TOW_BODY_MODEL_HEADER

#include <list>
#include <string>
#include "XYBezier.h"
#include "NodeRecord.h"

class TowBodyModel
{
 public:
  TowBodyModel();
  ~TowBodyModel() {};

  void setPtA(double ax, double ay);
  void setPtM(double mx, double my);
  void setPtZ(double zx, double zy);
  void setTowBodyLen(double);
  void setTowLineLen(double);
  void altTowLineLen(double v) {setTowLineLen(m_towline_len+v);}
  void initOwnshipToTowLine();
  void initTowLineToOwnship(double osx, double osy, double osh);
  void initTowLine(double osx, double osy, double osh,
		   double cnx, double cny);
  
  double getPtAX() const {return(m_towline.getPtAX());}
  double getPtAY() const {return(m_towline.getPtAY());}
  double getPtZX() const {return(m_towline.getPtZX());}
  double getPtZY() const {return(m_towline.getPtZY());}
  double getPtMX() const {return(m_towline.getPtMX());}
  double getPtMY() const {return(m_towline.getPtMY());}

  double getTBX() const {return(m_tbx);}
  double getTBY() const {return(m_tby);}
  double getTBH() const {return(m_tbh);}
  
  double getCenterX() const {return(m_towline.getCenterX());}
  double getCenterY() const {return(m_towline.getCenterY());}
  
  double getTowLineLen() const {return(m_towline_len);}
  double getTowBodyLen() const {return(m_towline_len);}
  double getCurveLen() const {return(m_towline.getCurveLen());}
  
  void   shiftX(double v) {m_towline.shiftX(v);}
  void   shiftY(double v) {m_towline.shiftY(v);}
  void   rotate(double v) {m_towline.rotate(v);}
  
  bool   update(NodeRecord record, double tstamp);

  XYBezier getTowLine() const {return(m_towline);}
  NodeRecord getNodeRecord() const {return(m_ownship);}
  
 protected:

  NodeRecord m_ownship;

  XYBezier   m_towline;
  double     m_towline_len;
  double     m_towbody_len;
  double     m_tbx;
  double     m_tby;
  double     m_tbh;

  double     m_dist; 
  
  unsigned int m_curve_pts;
  
  std::list<double> m_delta_hdgs;
  std::list<double> m_delta_tstamps;
  
};

#endif 
  





