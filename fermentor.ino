// This #include statement was automatically added by the Particle IDE.
#include <OneWire.h>



#include "math.h"
#include <vector>
#include "application.h"
#include "EEPROMio.h"
#include "probe.h"
#include "PID_v1.h"
#include "fridge.h"
#include "globals.h"



#define DEBUG true  // debug flag for including debugging code



void setup() {
    
  // zone(-7);  //set timezone.  -7 is for MST in the US.
    
  pinMode(relay1, OUTPUT);  // configure relay pins and write default LOW (relay open)
    digitalWrite(relay1, LOW); //Off for fridge
  pinMode(relay2, OUTPUT);
    analogWrite(relay2, 0, 1); //Off for heat relay

  
  mode = OFF; //init on powerup in the off state.
  profileStart = 0;
  
  #if DEBUG == true  //start serial at 9600 baud for debuging
    Serial.begin(9600);
  #endif


 
  // Load settings from EEPROM.  
  byte ver;
  EEPROMRead(0, &ver, EEBYTE); //Check to ensure EEPROM_VER constant matches curretn version in EEPROM.  
  //  Allows a way to force changes to EEPROM defaults as well as ensure defaults are loaded on new Photon.  
  if (EEPROM_VER != ver){
      EEPROMWritePresets();  // if version # is outdated, write presets
  }
  
  EEPROMReadSettings();    // load program settings from EEPROM
  
  
  #if DEBUG == true
    Serial.println(F("Settings loaded from EEPROM:"));
  #endif

  //  Sensor setup
  // If one sensor is not conneced or otherwise fails to comunicate fridge and beer will read the saem temperature off the one working sensor
  // lacking a better fail safe this works pretty good.  
  fridge.init();
  beer.init();
  
  probe::startConv();  // start conversion for all sensors
  delay(1500);  
  fridge.update();
  beer.update();
  Output = beer.getTemp();
  webLastupdate = 0;
  
 // setup the initial PID settings
  startTime = Time.now();
  mainPID.SetTunings(Kp, Ki, Kd);    // set tuning params
  mainPID.SetSampleTime(1000);       // (ms) matches sample rate (1 hz)
  mainPID.SetOutputLimits(0.3, 38);  // deg C (~32.5 - ~100 deg F)
  Setpoint = 20.0;
 
  mainPID.SetMode(AUTOMATIC);
  mainPID.setOutputType(FILTERED);
  mainPID.setFilterConstant(10);
  mainPID.initHistory();

  heatPID.SetTunings(heatKp,heatKi,heatKd);  //Kp, Ki, Kd loaded from EEPROM.
  heatPID.SetSampleTime(1000);       // sampletime = time proportioning window length 1 second
  heatPID.SetOutputLimits(0, 100);  // heatPID output = duty % 0 to 100
  heatPID.SetMode(AUTOMATIC);
  heatPID.initHistory();

  // Particle variables are mainly used for debug only.  Actual Partical data is shared to the web through particle.publish 
  // and webhooks.
    Particle.variable("mode", mode); 
    Particle.variable("fridgeState", curFridgeState);
    Particle.variable("beerTemp", Input);
    Particle.variable("fridgeTemp", curFridgeTemp);
    Particle.variable("setpoint", Setpoint); 
    Particle.variable("output", Output);
    Particle.variable("Kp", Kp);
    Particle.variable("Ki", Ki);
    Particle.variable("Kd", Kd);
    Particle.variable("heatInput", heatInput);
    Particle.variable("heatOutput", heatOutput);
    Particle.variable("heatSetpoint", heatSetpoint);
    Particle.variable("heatKp", heatKp);
    Particle.variable("heatKi", heatKi);
    Particle.variable("heatKd", heatKd);
    //  Particle.variable("peakest", peakest);  //Good for debug purposes. 
    
    // String for sending pid setting data to web app
    PIDsetting = "{\"Kp\":"+String(Kp)+",\"Ki\":"+String(Ki)+",\"Kd\":"+String(Kd)+",\"hKp\":"+String(heatKp)+",\"hKi\":"+String(heatKi)
    +",\"hKd\":"+String(heatKd)+",\"output\":"+String(Output)+",\"hOutput\":"+String(heatOutput)+",\"mainMode\":"+String(mainPID.GetMode())
    +",\"heatMode\":"+String(heatPID.GetMode())+"}";
    Particle.variable("PIDsetting", PIDsetting);
 
    Particle.function("webPublish", webPublish);  // init a publish of data for the web app.
    Particle.function("profileSetup", setupProfile); // setup the data for a beer profile.
    Particle.function("setSetpoint", setSetpointFunctionHandler); // send the beer setpoint from the cloud.
    Particle.function("setMode", setModeFunctionHandler); // send the operating mode from the web app.
    Particle.function("PIDSetup",PIDSettingHandler);  //  change PID settings from the web app.
    Particle.function("PIDSetMode",PIDSetModeHandler); // Change the PID mode for Manual tuning.  
}


void loop() {
    if (mode == OFF){
        if (fridgeState[0] != IDLE){
            updateFridgeState(IDLE, IDLE);
        }
        digitalWrite(relay1, LOW); //Off for fridge
        analogWrite(relay2, 0, 1); //Off for heat relay
        mainUpdate();
        profileStart = 0;
    }
    if (mode == BEER_CONSTANT){
        mainUpdate();  // subroutines manage their own timings, call every loop
        PIDUpdate();
    }
    if (mode == FRIDGE_CONSTANT){
        mainUpdate();  // subroutines manage their own timings, call every loop
        Input = curFridgeTemp;
        PIDUpdate();
    }
    if (mode == PROFILE){
        tempProfile();
        mainUpdate();  // subroutines manage their own timings, call every loop
        PIDUpdate();
    }
    if (mode == MAN){
        mainUpdate();
        if (heatPID.GetMode() == MANUAL){
            analogWrite(relay2, heatOutput*2.55, 1);
        }
        if (mainPID.GetMode() == MANUAL ){
            updateFridge(); 
        }
    }
}


// call all temperature update subroutines adn assign updated temperatures to PID variables.
void mainUpdate()  {
  probe::startConv();               // start conversion for all sensors
      if (probe::isReady()) {       // update sensors when conversion complete
        fridge.update();
        beer.update();
        Input = beer.getFilter();
        curBeerTemp = Input;
        curFridgeTemp = fridge.getFilter();
        heatInput = curFridgeTemp;
      }
  unsigned long now = millis();
  if ((now - fireWebLastupdate > 120000)){   //Update the firebase database with latest data once every 2 minute.
        fireWebLastupdate = now;
        PIDsetting = "{\"Kp\":"+String(Kp)+",\"Ki\":"+String(Ki)+",\"Kd\":"+String(Kd)+",\"hKp\":"+String(heatKp)+",\"hKi\":"+String(heatKi)
        +",\"hKd\":"+String(heatKd)+",\"output\":"+String(Output)+",\"hOutput\":"+String(heatOutput)+",\"mainMode\":"+String(mainPID.GetMode())
        +",\"heatMode\":"+String(heatPID.GetMode())+"}";
        webPublish("");
  }  
  if ((now - fireChartLastupdate > 600000)&& (mode != OFF)){  //When running in any temperature control mode send data for history chart once every 10 min.
        fireChartLastupdate = now;
        Particle.publish("chartData", String::format("{\"1\": \"%f\",\"2\": \"%f\",\"3\": \"%f\",\"4\": \"%d\"}"
        ,curBeerTemp,curFridgeTemp,Output,fridgeState[0]));
  }
  pastFridgeState = getFridgeState(1);
  curFridgeState = getFridgeState(0);
  if ((curFridgeTemp - webLastupdate) > 0.2 || (curFridgeTemp - webLastupdate) < -0.2){  //Additional firebase database update if temperatures are moving fast.
        webLastupdate = curFridgeTemp;
        webPublish("");
  }
}

void PIDUpdate(){
    mainPID.Compute();   // update main PID
    updateFridge();     // update fridge status
}

// Function to handle particle cloud sending an updated target fridge chamber temperature
double setSetpointFunctionHandler (String x) {
    Setpoint = x.toFloat();  // Extract float
    webPublish("");  //update the firebase database with realtime changes.
    return Setpoint;
}

// Function to handle particle cloud sending a change to the operating mode. 
// "0" = Off, "1" = Beer Const, "2" = Chamber Const, "3" = Profile follow
int setModeFunctionHandler (String x) {
    int newMode = x.toInt();  // Extract float
    if(mode == newMode){  
        return -1;  //do nothing and return if attempting to set the mode to the current mode.  
    }
    mode = newMode;
    if (mode == PROFILE){
        profileStart = Time.now(); 
    }
    startTime = Time.now();
    stopTime = Time.now();
    webPublish("");  //Update firebase database with realtime changes.  
    return mode;
}

// read settings from EEPROM
void EEPROMReadSettings() {  
  EEPROMRead(2, &Setpoint, EEDOUBLE);
  EEPROMRead(10, &Output, EEDOUBLE);
  EEPROMRead(18, &Kp, EEDOUBLE);
  EEPROMRead(26, &Ki, EEDOUBLE);
  EEPROMRead(34, &Kd, EEDOUBLE);
  EEPROMRead(42, &heatOutput, EEDOUBLE);
  EEPROMRead(50, &heatKp, EEDOUBLE);
  EEPROMRead(58, &heatKi, EEDOUBLE);
  EEPROMRead(66, &heatKd, EEDOUBLE);
}
 
// write current settings to EEPROM
void  EEPROMWriteSettings() {  
  EEPROMWrite(18, Kp, EEDOUBLE);
  EEPROMWrite(26, Ki, EEDOUBLE);
  EEPROMWrite(34, Kd, EEDOUBLE);
  EEPROMWrite(42, heatOutput, EEDOUBLE);
  EEPROMWrite(50, heatKp, EEDOUBLE);
  EEPROMWrite(58, heatKi, EEDOUBLE);
  EEPROMWrite(66, heatKd, EEDOUBLE);
}




void EEPROMWritePresets() {      // save defaults to eeprom
  byte temp = EEPROM_VER;
  EEPROMWrite(0, temp, EEBYTE);                // update EEPROM version
  EEPROMWrite(2, (double)20.00, EEDOUBLE);       // default main Setpoint
  EEPROMWrite(10, (double)20.00, EEDOUBLE);       // default main Output for manual operation
  EEPROMWrite(18, (double)10.00, EEDOUBLE);      // default main Kp
  EEPROMWrite(26, (double)5E-4, EEDOUBLE);       // default main Ki
  EEPROMWrite(34, (double)0, EEDOUBLE);     // default main Kd
  EEPROMWrite(42, (double)00.00, EEDOUBLE);      // default HEAT Output for manual operation
  EEPROMWrite(50, (double)20.00, EEDOUBLE);      // default HEAT Kp
  EEPROMWrite(58, (double)5E-4, EEDOUBLE);      // default HEAT Ki
  EEPROMWrite(66, (double)0, EEDOUBLE);      // default HEAT Kd
  
}


// accepts a comma seperated string from the web.  MUST be in exact order and length of "Kp,Ki,Kd,heatKp,heatKi,headKd,"  
int PIDSettingHandler(String PIDval){
    Kp= (PIDval.substring(0, PIDval.indexOf(","))).toFloat();
    //EEPROMWrite(18, Kp, EEDOUBLE);
    PIDval.remove(0,PIDval.indexOf(",")+1);
    Ki= (PIDval.substring(0, PIDval.indexOf(","))).toFloat();
   // EEPROMWrite(26, Ki, EEDOUBLE);
    PIDval.remove(0,PIDval.indexOf(",")+1);
    Kd= (PIDval.substring(0, PIDval.indexOf(","))).toFloat();
    //EEPROMWrite(34, Kd, EEDOUBLE);
    PIDval.remove(0,PIDval.indexOf(",")+1);
    heatKp= (PIDval.substring(0, PIDval.indexOf(","))).toFloat();
   // EEPROMWrite(50, heatKp, EEDOUBLE);
    PIDval.remove(0,PIDval.indexOf(",")+1);
    heatKi= (PIDval.substring(0, PIDval.indexOf(","))).toFloat();
   // EEPROMWrite(58, heatKi, EEDOUBLE);
    PIDval.remove(0,PIDval.indexOf(",")+1);
    heatKd= (PIDval.substring(0, PIDval.indexOf(","))).toFloat();
    //EEPROMWrite(66, heatKd, EEDOUBLE);
    PIDval.remove(0,PIDval.indexOf(",")+1);
    mainPID.SetTunings(Kp, Ki, Kd);
    heatPID.SetTunings(heatKp,heatKi,heatKd); 
    EEPROMWriteSettings();
    PIDsetting = "{\"Kp\":"+String(Kp)+",\"Ki\":"+String(Ki)+",\"Kd\":"+String(Kd)+",\"hKp\":"+String(heatKp)+",\"hKi\":"+String(heatKi)
        +",\"hKd\":"+String(heatKd)+",\"output\":"+String(Output)+",\"hOutput\":"+String(heatOutput)+",\"mainMode\":"+String(mainPID.GetMode())
        +",\"heatMode\":"+String(heatPID.GetMode())+"}";
    return 1;
}

// Function to handle mode change request from web app.
// data MUST be comma seperated and sent in this order mainPID(auto(1)/manual(0)),heatPID(auto(1)/manual(0)),output,heatoutput
int PIDSetModeHandler(String PIDval){
    int state;
    state = (PIDval.substring(0, PIDval.indexOf(","))).toInt();
    mainPID.SetMode(state);
    PIDval.remove(0,PIDval.indexOf(",")+1);
    state = (PIDval.substring(0, PIDval.indexOf(","))).toInt();
    heatPID.SetMode(state);
    PIDval.remove(0,PIDval.indexOf(",")+1);
    Output = (PIDval.substring(0, PIDval.indexOf(","))).toFloat();
    PIDval.remove(0,PIDval.indexOf(",")+1);
    heatOutput = (PIDval.substring(0, PIDval.indexOf(","))).toFloat();
    PIDval.remove(0,PIDval.indexOf(",")+1);
    startTime = Time.now();
    stopTime = Time.now();
    mode = MAN;
    PIDsetting = "{\"Kp\":"+String(Kp)+",\"Ki\":"+String(Ki)+",\"Kd\":"+String(Kd)+",\"hKp\":"+String(heatKp)+",\"hKi\":"+String(heatKi)
        +",\"hKd\":"+String(heatKd)+",\"output\":"+String(Output)+",\"hOutput\":"+String(heatOutput)+",\"mainMode\":"+String(mainPID.GetMode())
        +",\"heatMode\":"+String(heatPID.GetMode())+"}";
    
    return 1;
}


//Function to load a new profile from the web app. 
// data MUST be comma seperated and sent in this order profilename,day,temp,day,temp.......day,temp,  the ending comma is required.
int setupProfile(String profile) {
    profile_v.clear();
    int i = 0;
    profileName = (profile.substring(0, profile.indexOf(",")));
    profile.remove(0,profile.indexOf(",")+1);
    while (profile.length() > 1){
        profile_v.push_back(profileStep());
        profile_v[i].day = (profile.substring(0, profile.indexOf(","))).toFloat();
        profile.remove(0,profile.indexOf(",")+1);
        profile_v[i].temp = profile.substring(0, profile.indexOf(",")).toFloat();
        profile.remove(0,profile.indexOf(",")+1);
        i++;
    }
    profileStart = Time.now();
    
    return profile_v.size();
}

// function to update the target temp based on the profile.  system will hold last temp in profile until manually stopped.  
void tempProfile() {
    int nowTime = Time.now();
    double t0, t1;
  
    for (int i = 0; i < profile_v.size(); i++){
        if (((profile_v[i].day)*86400+profileStart)>nowTime){
            t0 = profile_v[i-1].day*86400+profileStart;
            t1 = profile_v[i].day*86400+profileStart;
            double newSetpoint =(((nowTime-t0)/(t1-t0))*(profile_v[i].temp-profile_v[i-1].temp))+profile_v[i-1].temp;
            if ((newSetpoint - Setpoint) > 0.1 || (newSetpoint - Setpoint) < -0.1){
                String sp = String(newSetpoint);  //convert the new setpoint to a string.
                setSetpointFunctionHandler(sp);  //use the function handler to update Setpoint variable.  This will also send updated data back to web page.  
            }
            i = profile_v.size();
        } 
    }
}


// function to send latest temperature and status data to firebase database. 
int webPublish(String x){
    //Particle.publish("webPage", String::format("{\"1\": \"%f\",\"2\": \"%f\",\"3\": \"%f\",\"4\": \"%d\",\"5\": \"%d\",\"6\": \"%d\",\"7\": \"%d\",\"8\": \"%f\",\"9\": \"%s\",\"10\": \"%d\"}"
   // ,curBeerTemp,curFridgeTemp,Output,fridgeState[0],fridgeState[1],startTime,mode,Setpoint,profileName.c_str(),profileStart));
    
    Particle.publish("PubSub", String::format("{\"1\": \"%f\",\"2\": \"%f\",\"3\": \"%f\",\"4\": \"%d\",\"5\": \"%d\",\"6\": \"%d\",\"7\": \"%d\",\"8\": \"%f\",\"9\": \"%s\",\"10\": \"%d\"}"
    ,curBeerTemp,curFridgeTemp,Output,fridgeState[0],fridgeState[1],startTime,mode,Setpoint,profileName.c_str(),profileStart));
    
   
 
    return 1;
}





























