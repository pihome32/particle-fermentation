#ifndef GLOBALS_H
#define GLOBALS_Hhttps://build.particle.io/build/58054f3c4748893bf0001525/tab/globals.h#

#define ONE_WIRE_BUS A0  //Photon pin the one wire temperature probes are connected.
#define TEMPERATURE_PRECISION 12



// double peakest; //Used while monidoring the peak estimation value in development.  

const byte EEPROM_VER = 0X01;  // eeprom data tracking



void mainUpdate();  // update sensors, PID output, fridge state, write to log, run profiles
int webPublish(String); //update current temerature and status to web. 


double lastOutput = 0;
double oldInput = 0;
double curFridgeTemp, curBeerTemp;
int curFridgeState, pastFridgeState;

void EEPROMReadSettings();   // read saved settings from EEPROM
void EEPROMWriteSettings();  // write current settings to EEPROM
void EEPROMWritePresets();   // write default settings to EEPROM
void PIDUpdate();
int setModeFunctionHandler (String x);

unsigned long fireChartLastupdate = millis();  //used to limit data written to firebase for the historical chart
unsigned long fireWebLastupdate = millis();  //used to refesh web data on a semi regualar basis
double webLastupdate;  //used to monitor chamber temp and update when changes. 


// Fridge actuator controls  Used to help make mode stae code more readable.  
enum modeState {  // fridge operation states
  OFF,
  BEER_CONSTANT,
  FRIDGE_CONSTANT,
  PROFILE,
  MAN
};


int mode;

int profileStart = 0;
String profileName = "";

struct profileStep {  // struct to encapsulate temperature and duration for fermentation profiles
  double day;
  double temp;
};

std::vector<profileStep> profile_v;

int startTime = 0;  // timing variables for enforcing min/max cycling times

const byte relay1 = D1;       // relay 1 (fridge compressor)
const byte relay2 = D2;       // relay 2 (heating element)


// DS18B20 Thermometer Stuff
OneWire onewire(ONE_WIRE_BUS);  // declare instance of the OneWire class to communicate with onewire sensors
probe beer(&onewire), fridge(&onewire);


byte programState;  // 6 bit-flag program state -- (mainPID manual/auto)(heatPID manual/auto)(temp C/F)(fermentation profile on/off)(data capture on/off)(file operations) = 0b000000

#define MAIN_PID_MODE 0b100000
#define HEAT_PID_MODE 0b010000
#define DISPLAY_UNIT  0b001000
#define TEMP_PROFILE  0b000100
#define DATA_LOGGING  0b000010
#define FILE_OPS      0b000001

double Input, Setpoint, Output, Kp, Ki, Kd;  // SP, PV, CO, tuning params for main PID
double heatInput, heatOutput, heatSetpoint, heatKp, heatKi, heatKd;  // SP, PV, CO tuning params for HEAT PID
PID mainPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // main PID instance for beer temp control (DIRECT: beer temperature ~ fridge(air) temperature)
PID heatPID(&heatInput, &heatOutput, &heatSetpoint, heatKp, heatKi, heatKd, DIRECT);   // create instance of PID class for cascading HEAT control (HEATing is a DIRECT process)
String PIDsetting = "";

#endif

