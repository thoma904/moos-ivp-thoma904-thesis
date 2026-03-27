/************************************************************/
/*    NAME: Tom Monaghan                                    */
/*    ORGN: MIT                                             */
/*    FILE: main.cpp (AOF benchmark harness)                */
/*    DATE: Mar 2026                                        */
/*                                                          */
/* Standalone test harness for AOF_TowObstacleAvoid.        */
/* Iterates over every (course, speed) pair in the domain   */
/* and reports how many evalBox() calls per second the      */
/* AOF can sustain.                                         */
/************************************************************/

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <string>
#include "IvPDomain.h"
#include "IvPBox.h"
#include "AOF_TowObstacleAvoid.h"
#include "ObShipModelV24.h"
#include "XYPolygon.h"
#include "MBTimer.h"

using namespace std;

int main(int argc, char *argv[])
{
  // -----------------------------------------------------------
  // Configurable parameters (defaults, overridable via args)
  // -----------------------------------------------------------
  unsigned int crs_pts  = 360;   // course domain points (1-deg)
  unsigned int spd_pts  = 21;    // speed domain points
  double       spd_max  = 3.0;   // max speed (m/s)
  double       sim_hz   = 25.0;  // sim_horizon (seconds)
  double       sim_dt   = 0.1;   // sim time step
  int          reps     = 100;   // number of full-domain sweeps
  double       cable_len    = 30;   // cable length (m)
  int          cable_check  = 5;    // cable check interval (steps)
  int          cable_start  = 0;    // cable start node
  double       turn_rate    = 15.0; // turn rate max (deg/s)

  // Simple arg parsing: --key=value
  for(int i = 1; i < argc; i++) {
    string arg = argv[i];
    if(arg.find("--crs_pts=") == 0)
      crs_pts = atoi(arg.substr(10).c_str());
    else if(arg.find("--spd_pts=") == 0)
      spd_pts = atoi(arg.substr(10).c_str());
    else if(arg.find("--spd_max=") == 0)
      spd_max = atof(arg.substr(10).c_str());
    else if(arg.find("--sim_horizon=") == 0)
      sim_hz = atof(arg.substr(14).c_str());
    else if(arg.find("--sim_dt=") == 0)
      sim_dt = atof(arg.substr(9).c_str());
    else if(arg.find("--reps=") == 0)
      reps = atoi(arg.substr(7).c_str());
    else if(arg.find("--cable_len=") == 0)
      cable_len = atof(arg.substr(12).c_str());
    else if(arg.find("--cable_check=") == 0)
      cable_check = atoi(arg.substr(14).c_str());
    else if(arg.find("--cable_start=") == 0)
      cable_start = atoi(arg.substr(14).c_str());
    else if(arg.find("--turn_rate=") == 0)
      turn_rate = atof(arg.substr(12).c_str());
    else {
      cout << "Usage: aof_bench [options]" << endl;
      cout << "  --crs_pts=N       course domain points  (default 360)" << endl;
      cout << "  --spd_pts=N       speed domain points   (default 21)"  << endl;
      cout << "  --spd_max=V       max speed m/s         (default 3.0)" << endl;
      cout << "  --sim_horizon=T   sim horizon seconds    (default 25)" << endl;
      cout << "  --sim_dt=T        sim time step          (default 0.1)" << endl;
      cout << "  --reps=N          full-domain sweeps     (default 100)" << endl;
      cout << "  --cable_len=L     cable length meters    (default 30)" << endl;
      cout << "  --cable_check=N   cable check interval   (default 5)"  << endl;
      cout << "  --cable_start=N   cable start node       (default 0)"  << endl;
      cout << "  --turn_rate=D     turn rate max deg/s    (default 15)" << endl;
      return 0;
    }
  }

  // -----------------------------------------------------------
  // 1) Build the IvP domain
  // -----------------------------------------------------------
  IvPDomain domain;
  domain.addDomain("course", 0, 359, crs_pts);
  domain.addDomain("speed",  0, spd_max, spd_pts);

  int crs_ix = domain.getIndex("course");
  int spd_ix = domain.getIndex("speed");
  unsigned int num_crs = domain.getVarPoints(crs_ix);
  unsigned int num_spd = domain.getVarPoints(spd_ix);

  cout << "Domain: course [0,359] " << num_crs << " pts, "
       << "speed [0," << spd_max << "] " << num_spd << " pts" << endl;
  cout << "Sim horizon: " << sim_hz << " s, dt: " << sim_dt << " s, "
       << "steps/eval: " << (int)ceil(sim_hz / sim_dt) << endl;
  cout << "Cable: len=" << cable_len << " m, check_interval=" << cable_check
       << ", start_node=" << cable_start
       << ", nodes=" << (cable_len < 30 ? 3 : max(3, (int)(cable_len / 10.0))) << endl;
  cout << "Turn rate max: " << turn_rate << " deg/s" << endl;
  cout << "Reps: " << reps << endl;

  // -----------------------------------------------------------
  // 2) Set up the obstacle ship model
  // -----------------------------------------------------------
  ObShipModelV24 obm;
  obm.setPose(0, 0, 45);      // ownship at origin heading 045
  obm.setMinUtilCPA(2.5);
  obm.setMaxUtilCPA(5.0);
  obm.setAllowableTTC(15.0);

  // Simple square obstacle ahead of ownship
  XYPolygon obs;
  obs.add_vertex(50, 50);
  obs.add_vertex(55, 50);
  obs.add_vertex(55, 55);
  obs.add_vertex(50, 55);
  obm.setGutPoly(obs);
  obm.setCachedVals(true);

  // -----------------------------------------------------------
  // 3) Build and configure the AOF
  // -----------------------------------------------------------
  AOF_TowObstacleAvoid aof(domain);
  aof.setObShipModel(obm);

  aof.setTowEval(true);
  aof.setTowOnly(true);
  aof.setTowState(-25, -10, 0.5, 0.3);
  aof.setTowDynParams(cable_len, 5, 5.0, 0.7, 2.0);
  aof.setSimParams(sim_dt, sim_hz, turn_rate);
  aof.setCableCheckInterval(cable_check);
  aof.setCableStartNode(cable_start);

  bool ok = aof.initialize();
  if(!ok) {
    cout << "AOF initialize() failed." << endl;
    return 1;
  }
  cout << "AOF initialized successfully." << endl;

  // -----------------------------------------------------------
  // 4) Time the full-domain sweep
  // -----------------------------------------------------------
  unsigned int total_evals = num_crs * num_spd * reps;

  // Sanity check: single eval before timed loop
  IvPBox *box = new IvPBox(2);
  box->setPTS(crs_ix, 0, 0);
  box->setPTS(spd_ix, 0, 0);
  double test_val = aof.evalBox(box);
  cout << "Sanity check evalBox(crs=0,spd=0) = " << test_val << endl;
  cout << flush;

  MBTimer timer;
  timer.start();

  for(int r = 0; r < reps; r++) {
    for(unsigned int ci = 0; ci < num_crs; ci++) {
      for(unsigned int si = 0; si < num_spd; si++) {
        box->setPTS(crs_ix, ci, ci);
        box->setPTS(spd_ix, si, si);
        aof.evalBox(box);
      }
    }
  }

  timer.stop();
  delete box;

  // -----------------------------------------------------------
  // 5) Report results
  // -----------------------------------------------------------
  float elapsed = timer.get_float_wall_time();
  double evals_per_sec = (elapsed > 0) ? total_evals / (double)elapsed : 0;

  cout << "---------------------------------------" << endl;
  cout << "Total evals:   " << total_evals << endl;
  cout << "Wall time:     " << elapsed << " sec" << endl;
  cout << "Evals/sec:     " << (long)evals_per_sec << endl;
  cout << "Time per eval: " << (elapsed / total_evals) * 1e6 << " us" << endl;
  cout << "---------------------------------------" << endl;

  return 0;
}
