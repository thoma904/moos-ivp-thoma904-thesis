/************************************************************/
/*    NAME: Tom Monaghan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Towing.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef Towing_HEADER
#define Towing_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class Towing : public AppCastingMOOSApp
{
 public:
   Towing();
   ~Towing();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
};

#endif 
