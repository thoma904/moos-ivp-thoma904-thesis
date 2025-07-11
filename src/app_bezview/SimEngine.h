/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: SimEngine.h                                          */
/*    DATE: Mar 8th, 2005 just another day at CSAIL              */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef SIM_ENGINE_HEADER
#define SIM_ENGINE_HEADER

#include "NodeRecord.h"
#include "ThrustMap.h"

class SimEngine
{
public:
  SimEngine() {}
  ~SimEngine() {}
  
public:
  void propagate(NodeRecord&, double delta_time, double prior_heading,
		 double prior_speed, double drift_x, double drift_y);
  
  void propagateSpeed(NodeRecord&, const ThrustMap&, double delta_time, 
		      double thrust, double rudder,
		      double max_accel, double max_decel);

  void propagateHeading(NodeRecord&, double delta_time, double rudder,
			double thrust, double turn_rate, 
			double rotate_speed);
};

#endif









