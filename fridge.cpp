#include "math.h"
#include "fridge.h"

byte fridgeState[2] = { IDLE, IDLE };      // [0] - current fridge state; [1] - fridge state (t - 1) history
double peakEstimator = 20;    // to predict COOL overshoot; units of deg C per hour (always positive)
                              //  manually set to 20 as default. Appears to be a reaonsalbe start point.  Will adjust while running.  
double peakEstimate = 0;      // to determine prediction error = (estimate - actual)
int stopTime = 0;
int runTime;


void updateFridge() {// maintain fridge at temperature set by mainPID -- COOLing with predictive differential, HEATing with output % proportioned heatPID
  
  switch (fridgeState[0]) {  // MAIN switch -- IDLE/peak detection, COOL, HEAT routines
    
    case IDLE:
      if (fridgeState[1] == IDLE) {   // only switch to HEAT/COOL if not waiting for COOL peak
        if ((curFridgeTemp > Output + fridgeIdleDiff) && (Input  > Setpoint + 0.05) && ((Time.now() - stopTime) > coolMinOff)) {  // switch to COOL only if temp exceeds IDLE range and min off time met
          startTime = Time.now(); 
          updateFridgeState(COOL);    // update current fridge status and t - 1 history
          digitalWrite(relay1, HIGH);  // close relay 1; supply power to fridge compressor
                // record COOLing start time
        }
        else if ((curFridgeTemp < Output - fridgeIdleDiff) && (Input  < Setpoint - 0.05) && ((Time.now() - stopTime) > heatMinOff)) {  // switch to HEAT only if temp below IDLE range and min off time met
          updateFridgeState(HEAT);
         // if (programState & 0b010000) 
          heatSetpoint = Output;  // update heat PID setpoint if in automatic mode
          heatPID.initHistory();
     //     heatPID.Compute();      // compute new heat PID output, update timings to align PID and time proportioning routine
          startTime = Time.now();   // start new time proportioned window
        }
      }
      else if (fridgeState[1] == COOL) {  // do peak detect if waiting on COOL
        if (fridge.peakDetect()) {        // negative peak detected...
          tuneEstimator(&peakEstimator, peakEstimate - curFridgeTemp);  // (error = estimate - actual) positive error requires larger estimator; negative:smaller
          startTime = Time.now();
          updateFridgeState(IDLE);  
          // stop peak detection until next COOL cycle completes
          
        }
        else {                                                               // no peak detected
          int offTime = Time.now() - stopTime;      // IDLE time in seconds
          if (offTime < peakMaxWait) break;                                  // keep waiting for filter confirmed peak if too soon
          tuneEstimator(&peakEstimator, peakEstimate - curFridgeTemp);  // temp is drifting in the right direction, but too slowly; update estimator
          startTime = Time.now();
          updateFridgeState(IDLE);  // stop peak detection
           
          
        }
      }
      break;

    case COOL:  // run compressor until peak predictor lands on controll
      runTime = Time.now() - startTime;
      if (runTime < coolMinOn) break;     // ensure minimum compressor runtime
      if (curFridgeTemp < Output - fridgeIdleDiff) {  // temp already below output - idle differential: most likely cause is change in setpoint or long minimum runtime
        digitalWrite(relay1, LOW);       // open relay 1; power down fridge compressor
        stopTime = Time.now();  // record idle start
        startTime = Time.now();  // for disply only.  Does not affect fridge control from COOL to IDLE, IDLE
        updateFridgeState(IDLE, IDLE);    // go IDLE, ignore peaks
        break;
      }
    
      if (curFridgeTemp - ((double)min(runTime, peakMaxTime)/3600*peakEstimator) < Output - fridgeIdleDiff) {  // if estimated peak exceeds Output - differential, set IDLE and wait for actual peak
        peakEstimate = curFridgeTemp - ((double)min(runTime, peakMaxTime)/3600*peakEstimator);   // record estimated peak prediction
        digitalWrite(relay1, LOW);
        stopTime = Time.now();
        startTime = Time.now();  // for disply only.  Does not affect fridge control from COOL to IDLE, IDLE
        updateFridgeState(IDLE);     // go IDLE, wait for peak
      }
      if (runTime > coolMaxOn) {  // if compressor runTime exceeds max on time, skip peak detect, go IDLE
        digitalWrite(relay1, LOW);
        stopTime = Time.now();
        startTime = Time.now();  // for disply only.  Does not affect fridge control from COOL to IDLE, IDLE
        updateFridgeState(IDLE, IDLE);
      }
      break;

    case HEAT:  // run HEAT using time proportioning
      heatSetpoint = Output;
      if (heatPID.Compute()) {  // Every compute update the PWM output.  
          analogWrite(relay2, heatOutput*2.55, 1);
      }
      if (curFridgeTemp > Output + fridgeIdleDiff) {  // temp exceeds setpoint, go to idle to decide if it is time to COOL
        analogWrite(relay2, 0, 1);
        heatOutput = 0;
        heatPID.Initialize();  //Reinialize the heat PID so it is ready for the next cycle.  If not done over time the iterm builds.  
        stopTime = Time.now();
        startTime = Time.now();  // for disply only.  Does not affect fridge control from COOL to IDLE, IDLE
        updateFridgeState(IDLE, IDLE);
      }
      break; 
    
  }
}

void tuneEstimator(double* estimator, double error) {  // tune fridge overshoot estimator
  if (fabs(error) <= fridgePeakDiff) return;            // leave estimator unchanged if error falls within contstrained peak differential
  if (error > 0) *estimator *= constrain(1.2 + 0.03 * fabs(error), 1.2, 1.5);                 // if positive error; increase estimator 20% - 50% relative to error
    else *estimator = max(0.05, *estimator / constrain(1.2 + 0.03 * fabs(error), 1.2, 1.5));  // if negative error; decrease estimator 17% - 33% relative to error, constrain to non-zero value
 
}

void updateFridgeState(int state) {  // update current fridge state
  fridgeState[1] = fridgeState[0];
  fridgeState[0] = state;
  webPublish("");
}

void updateFridgeState(int state0, int state1) {  // update current fridge state and history
  fridgeState[1] = state1;
  fridgeState[0] = state0;
  webPublish("");
}

