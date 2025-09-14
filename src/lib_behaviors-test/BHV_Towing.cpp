/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Towing.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_Towing.h"
#include "ZAIC_PEAK.h"
#include "AngleUtils.h"
#include "OF_Coupler.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_Towing::BHV_Towing(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  //Parameters
  m_osx = 0;
  m_osy = 0;
  m_os_heading = 0;
  m_towed_x = 0;
  m_towed_y = 0;
  m_towed_heading = 0;
  m_tow_act = false;
  m_ipf_type  = "zaic";

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING, TOWED_X, TOWED_Y, TOWED_HEADING");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Towing::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "foo") && isNumber(val)) {
    // Set local member variables here
    return(true);
  }
  else if (param == "bar") {
    // return(setBooleanOnString(m_my_bool, val));
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_Towing::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_Towing::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_Towing::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_Towing::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_Towing::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_Towing::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_Towing::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_Towing::onRunState()
{
  // Part 1: Get vehicle position from InfoBuffer and post a
  bool ok1, ok2, ok3, ok4, ok5, ok6;
    
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  m_os_heading = getBufferDoubleVal("DESIRED_HEADING", ok3);

  if(!ok1 || !ok2 || !ok3 ) 
  {
    postWMessage("No ownship X/Y/Heading info in info_buffer.");
    return(0);
  }

  m_towed_x = getBufferDoubleVal("TOWED_X", ok4);
  m_towed_y = getBufferDoubleVal("TOWED_Y", ok5);
  m_towed_heading = getBufferDoubleVal("TOWED_HEADING", ok6);

  if(!ok4 || !ok5 || !ok6 ) 
  {
    postWMessage("No towed X/Y/Heading info in info_buffer.");
    return(0);
  }
  
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;
  if(m_tow_act)
  {
    if(m_ipf_type == "zaic")
      ipf = buildFunctionWithZAIC();
    if(ipf == 0) 
      postWMessage("Problem Creating the IvP Function");
  }

  // Part N: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}

