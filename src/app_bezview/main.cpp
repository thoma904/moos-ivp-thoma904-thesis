/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: main.cpp                                             */
/*    DATE: Apr 17th 2020                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

// -*-Menlo-normal-normal-normal-*-12-*-*-*-m-0-iso10646-1

#include <iostream>
#include <FL/Fl.H>
#include "MBUtils.h"
#include "BEZ_GUI.h"

using namespace std; 
   
void showHelpAndExit();
 
//--------------------------------------------------------
// Procedure: idleProc() 

void idleProc(void *) 
{
  Fl::flush();
  millipause(10);
}
   
//--------------------------------------------------------
// Procedure: main()
  
int main(int argc, char *argv[])
{
  Fl::add_idle(idleProc);
 
  BEZ_GUI  gui(1000, 800, "MIT Bezier Viewer");

  for(int i=1; i<argc; i++) { 
    string argi  = argv[i];

    bool handled = true;
    if((argi == "-h") || (argi == "--help"))
      showHelpAndExit();
     
    else if((argi == "-v") || (argi == "--verbose"))
      gui.addConfigParam(argi);
    else if(strEnds(argi, ".tif"))
      gui.addConfigParam(argi);

    else if(strBegins(argi, "--ax=")) 
      gui.addConfigParam(argi);
    else if(strBegins(argi, "--ay=")) 
      gui.addConfigParam(argi);
    else if(strBegins(argi, "--zx=")) 
      gui.addConfigParam(argi);
    else if(strBegins(argi, "--zy=")) 
      gui.addConfigParam(argi);
    else if(strBegins(argi, "--mx=")) 
      gui.addConfigParam(argi);
    else if(strBegins(argi, "--my=")) 
      gui.addConfigParam(argi);
    
    if(!handled) {
      cout << "bezview: Bad Arg: " << argi << endl;
      exit(1);
    }      
  }

  gui.m_bez_viewer->handleConfigParams();
  //gui.m_bez_viewer->applyTiffFiles();
  gui.m_bez_viewer->handlePostConfigParams();

  gui.updateXY();
  
  return Fl::run();
}

 
//--------------------------------------------------------
// Procedure: showHelpAndExit() 

void showHelpAndExit()
{
  cout << "Usage:                                              " << endl;
  cout << "  bezview [OPTIONS] image.tif                       " << endl;
  cout << "                                                    " << endl;
  cout << "Synopsis:                                           " << endl;
  cout << "  The bezview utility renders                       " << endl;
  cout << "                                                    " << endl;
  cout << "Options:                                            " << endl;
  cout << "  -h,--help      Displays this help message         " << endl;
  cout << "                                                    " << endl;
  cout << "Examples:                                           " << endl;
  cout << "  bezview                                           " << endl;
  exit(0);  
}
  






