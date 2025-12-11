/*****************************************************************/
/*    NAME: Tom Monaghan                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_TowSafety.h                                */
/*    DATE: Aug 7th 2006                                         */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/

#ifndef BHV_TOW_SAFETY_HEADER
#define BHV_TOW_SAFETY_HEADER

#include <list>
#include "IvPBehavior.h"

class BHV_TowSafety : public IvPBehavior {
public:
  BHV_TowSafety(IvPDomain);
  ~BHV_TowSafety() {}

  IvPFunction* onRunState();
  bool         setParam(std::string, std::string);
  bool         isConstraint() {return(true);}

protected:
  void  addHeading(double, double);
  bool  getHeadingAvg(double&);
  bool  getHeadingAvg2(double&);

private: // Configuration Parameters

  double m_memory_time;
  double m_turn_range;
  double m_min_tow_speed;

private: // State Variables

  std::list<double> m_heading_val;
  std::list<double> m_heading_time;

};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_TowSafety(domain);}
}
#endif


