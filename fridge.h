#ifndef FRIDGE_H
#define FRIDGE_H

#include "application.h"
#include "probe.h"
#include "PID_v1.h"
#include "EEPROMio.h"

enum opState {  // fridge operation states
  IDLE,
  COOL,
  HEAT,
};


const double fridgeIdleDiff = 0.5;       // constrain fridge temperature to +/- 0.5 deg C (0.9 deg F) differential
const double fridgePeakDiff = 0.25;      // constrain allowed peak error to +/- 0.25 deg C (0.45 deg F) differential
const int coolMinOff = 300;     // minimum compressor off time, seconds (5 min)
const int coolMinOn = 90;       // minimum compressor on time, seconds (1.5 min)
const int coolMaxOn = 2700;     // maximum compressor on time, seconds (45 min)
const int peakMaxTime = 1200;   // maximum runTime to consider for peak estimation, seconds (20 min)
const int peakMaxWait = 1800;   // maximum wait on peak, seconds (30 min)
const int heatMinOff = 300;     // minimum HEAT off time, seconds (5 min)


extern byte fridgeState[2];      // [0] - current fridge state; [1] - fridge state t - 1 history
extern double peakEstimator;     // to predict COOL overshoot; units of deg C per hour (always positive)
extern double peakEstimate;      // to determine prediction error = (estimate - actual)
extern int stopTime, runTime;

extern probe fridge, beer;  // external variables declared in globals.h
extern int startTime;  // timing variables for enforcing min/max cycling times
extern double Input, Setpoint, Output, heatSetpoint, heatOutput, curFridgeTemp;
extern const byte relay1, relay2;
extern byte programState;
extern PID heatPID;
extern int webPublish(String);

void updateFridge();  // core functions
void tuneEstimator(double* estimator, double error);
void updateFridgeState(int state);
void updateFridgeState(int state0, int state1);

inline byte getFridgeState(byte index) { return fridgeState[index]; };  // inlines for accessing fridge variables
inline double getPeakEstimator() { return peakEstimator; };
inline double* getPeakEstimatorAddr() { return &peakEstimator; };
inline unsigned long getStartTime() { return startTime; };
inline unsigned long getStopTime() { return stopTime; };

#endif