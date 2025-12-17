/*****************************************************************/
/*    NAME: Tom Monaghan                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_Tow_Safety.cpp                              */
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

#ifdef _WIN32
#pragma warning(disable : 4786)
#pragma warning(disable : 4503)
#endif
#include <cmath> 
#include <cstdlib>
#include "BHV_TowSafety.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "ZAIC_HEQ.h"


using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_TowSafety::BHV_TowSafety(IvPDomain gdomain) : 
  IvPBehavior(gdomain)
{
  m_descriptor = "tow_safety";

  m_domain = subDomain(m_domain, "speed");

  m_min_tow_speed = -1;

  addInfoVars("TOWED_SPEED");
}

//-----------------------------------------------------------
// Procedure: setParam
//     Notes: We expect the "waypoint" entries will be of the form
//            "xposition,yposition".
//            The "radius" parameter indicates what it means to have
//            arrived at the waypoint.

bool BHV_TowSafety::setParam(string param, string val) 
{
  if(IvPBehavior::setParam(param, val))
    return(true);

  else if(param == "min_tow_speed") {
    double dval = atof(val.c_str());
    if((dval < 0) || (!isNumber(val)))
      return(false);
    m_min_tow_speed = dval;
    return(true);
  }

  return(false);
}


//-----------------------------------------------------------
// Procedure: onRunState

IvPFunction *BHV_TowSafety::onRunState() 
{

  if(m_min_tow_speed < 0) {
    postWMessage("Variable min_tow_speed not specified");
    return(0);
  }

  bool ok;
  double towed_speed = getBufferDoubleVal("TOWED_SPEED", ok);

  if(!ok) {
    postEMessage("No Ownship TOWED_SPEED in info_buffer");
    return(0);
  }

  ZAIC_HEQ spd_zaic(m_domain, "speed");

  // Get speed domain limits so we can safely push to max
  double dom_spd_min = m_domain.getVarLow("speed");
  double dom_spd_max = m_domain.getVarHigh("speed");

  double summit_speed;

  // How far below min are we?
  double deficit = m_min_tow_speed - towed_speed;

  if (deficit > 0) 
  {
    // Tow too slow: increase commanded speed proportionally,
    // but don't jump straight to max.
    double gain = 1.5; // tune: how aggressively we react
    summit_speed = m_min_tow_speed + gain * deficit;
  } 
  
  else 
  {
    // Tow fine: be happy around the min safe speed.
    summit_speed = m_min_tow_speed;
  }

  // Clamp to domain
  if (summit_speed < dom_spd_min)
    summit_speed = dom_spd_min;
  if (summit_speed > dom_spd_max)
    summit_speed = dom_spd_max;


  spd_zaic.setSummit(summit_speed);
  spd_zaic.setMinMaxUtil(0, 100);
  spd_zaic.setBaseWidth(2.0);

  IvPFunction *ipf = spd_zaic.extractIvPFunction();

  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}