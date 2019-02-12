//-------------------------------------------------------------------
// Temperature Controller
// Based on: Sous Vide Controller
// Bill Earl - for Adafruit Industries
//
// Needs the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregard
//
// BME 4/517 modifications by Urs Utzinger
//------------------------------------------------------------------

// Arduino to temp
// read value is between 157 .. 637
// See corresdponding Matlab program where Manufacturer data and serial resistor is used to compute analog input value.
//
const int Table[] = { 
   5011, 4991, 4972, 4952, 4933, 4914, 4895, 4876, 4857, 4838, 4819, 4801, 4782, 4763, 4745, 4727, 4708, 4690, 4672, 4655,
   4637, 4620, 4602, 4585, 4568, 4551, 4534, 4517, 4500, 4483, 4467, 4450, 4433, 4417, 4400, 4384, 4368, 4352, 4336, 4320,
   4304, 4288, 4272, 4256, 4241, 4225, 4210, 4194, 4179, 4164, 4148, 4133, 4118, 4103, 4088, 4073, 4058, 4044, 4029, 4014,
   4000, 3985, 3971, 3956, 3942, 3928, 3913, 3899, 3885, 3871, 3857, 3843, 3829, 3815, 3802, 3788, 3774, 3761, 3747, 3734,
   3720, 3707, 3693, 3680, 3666, 3653, 3640, 3626, 3613, 3600, 3587, 3574, 3561, 3549, 3536, 3523, 3511, 3498, 3485, 3473,
   3460, 3448, 3435, 3423, 3411, 3398, 3386, 3374, 3361, 3349, 3337, 3325, 3313, 3301, 3289, 3277, 3265, 3253, 3241, 3229,
   3218, 3206, 3194, 3183, 3171, 3159, 3148, 3136, 3125, 3113, 3102, 3090, 3079, 3067, 3056, 3045, 3034, 3022, 3011, 3000,
   2989, 2978, 2967, 2955, 2944, 2933, 2922, 2911, 2901, 2890, 2879, 2868, 2857, 2846, 2836, 2825, 2814, 2804, 2793, 2782,
   2772, 2761, 2750, 2740, 2729, 2719, 2708, 2698, 2688, 2677, 2667, 2657, 2646, 2636, 2626, 2615, 2605, 2595, 2585, 2575,
   2564, 2554, 2544, 2534, 2524, 2514, 2504, 2494, 2484, 2474, 2464, 2454, 2444, 2434, 2425, 2415, 2405, 2395, 2385, 2376,
   2366, 2356, 2346, 2337, 2327, 2317, 2308, 2298, 2289, 2279, 2269, 2260, 2250, 2241, 2231, 2222, 2212, 2203, 2193, 2184,
   2175, 2165, 2156, 2146, 2137, 2128, 2118, 2109, 2100, 2091, 2081, 2072, 2063, 2054, 2044, 2035, 2026, 2017, 2008, 1999,
   1990, 1980, 1971, 1962, 1953, 1944, 1935, 1926, 1917, 1908, 1899, 1890, 1881, 1872, 1863, 1854, 1845, 1837, 1828, 1819,
   1810, 1801, 1792, 1783, 1775, 1766, 1757, 1748, 1739, 1731, 1722, 1713, 1704, 1696, 1687, 1678, 1670, 1661, 1652, 1644,
   1635, 1626, 1618, 1609, 1600, 1592, 1583, 1575, 1566, 1557, 1549, 1540, 1532, 1523, 1515, 1506, 1498, 1489, 1481, 1472,
   1464, 1455, 1447, 1438, 1430, 1421, 1413, 1404, 1396, 1388, 1379, 1371, 1362, 1354, 1346, 1337, 1329, 1321, 1312, 1304,
   1296, 1287, 1279, 1271, 1262, 1254, 1246, 1237, 1229, 1221, 1212, 1204, 1196, 1188, 1179, 1171, 1163, 1155, 1146, 1138,
   1130, 1122, 1114, 1105, 1097, 1089, 1081, 1073, 1064, 1056, 1048, 1040, 1032, 1024, 1015, 1007,  999,  991,  983,  975,
   967, 958, 950, 942, 934, 926, 918, 910, 902, 894, 885, 877, 869, 861, 853, 845, 837, 829, 821, 813, 805, 797, 789,
   780, 772, 764, 756, 748, 740, 732, 724, 716, 708, 700, 692, 684, 676, 668, 660, 652, 644, 636, 628, 620, 612, 604,
   596, 588, 579, 571, 563, 555, 547, 539, 531, 523, 515, 507, 499, 491, 483, 475, 467, 459, 451, 443, 435, 427, 419,
   411, 403, 395, 387, 379, 371, 363, 355, 347, 339, 331, 323, 315, 307, 299, 291, 283, 275, 267, 258, 250, 242, 234,
   226, 218, 210, 202, 194, 186, 178, 170, 162, 154, 146, 138, 130, 121, 113, 105,  97,  89,  81,  73,  65,  57,  48,
   40, 32, 24, 16, 8, 0};
      
#define DEBUG false

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
// So we can save and retrieve settings
#include <EEPROM.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay // will also blink LED
#define RelayPin 13
#define SensorPin A0
int SensorValue = 0;

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;
volatile long onTime = 0;

// pid tuning parameters
double Kp = 2;
double Ki = 0.5;
double Kd = 2;

// EEPROM addresses for persisted data storage
// Tuning Parameters
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 5 second Time Proportional Output window
volatile int WindowSize = 5000; 
volatile unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember        =2;    //
double aTuneStep              =1000; // between 0 and 5000 (Window Size)
double aTuneNoise             =1;    // 1 C grade
unsigned int aTuneLookBack    =120;  // 120 secs

boolean tuning   = false;
boolean poweroff = false;

// specify auto tune structure
PID_ATune aTune(&Input, &Output);

const int logInterval = 10000; // log every 10 seconds
long      lastLogTime = 0;
long      currentTime = 0;

// ************************************************
// States for state machine
// ************************************************

volatile enum operatingState { STARTUP = 0, RUN, INPUT_HANDLE, STATUS, OFF};
operatingState opState = OFF;

IntervalTimer myTimer;

// ************************************************
// Setup and display initial screen
// ************************************************

void setup()
{
   Serial.begin(115200);
   Serial.println("417 - 517 Temperature Controller");
   Serial.println("================================");
   Serial.println("Setting up...");

   // Initialize Relay Control:

   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start

   // Initialize the PID and related variables
   LoadParameters(); // from EEPROM
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetSampleTime(1);
   myPID.SetOutputLimits(0, WindowSize);

  // Run timer interrupt every 15 ms 
  myTimer.begin(Relay, 15000);
  
  opState = STARTUP;
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
void Relay() 
{
  if ((opState == STARTUP) | (poweroff == true))
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   switch (opState)
   {
   case STARTUP:
      Startup();
      break;
   case RUN:
      Run();
      break;
   case INPUT_HANDLE:
      InputHandle();
      break;
   case STATUS:
      DisplayStatus();
      break;
   default:
      Serial.println("Invalid State. Program should never be here.");
      break;
   }
}

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Startup()
{
   Serial.println("Starting up...");

   myPID.SetMode(MANUAL);
   
   // Prepare to transition to the RUN state
   SensorValue = analogRead(SensorPin);
   // Convert to temperature
   if (SensorValue < 157) { Input = 99.9; }
   else if (SensorValue > 637) { Input = -99.9;}
   else { Input = double(Table[(SensorValue-157)])/100.0; } // Lookup table is stored in interger, pgm_read is needed as we read from program memory
   
   //turn the PID on 
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = RUN; // start control
}

// ************************************************
// Execute the running state
// ************************************************
void Run()
{
   if (poweroff == false) {
     myPID.SetTunings(Kp,Ki,Kd);
     DoControl();
   }

   // initially had periodically loggin here
   currentTime = millis();
   if (currentTime - lastLogTime > logInterval)  
   {
     lastLogTime = currentTime;
     SaveParameters(); // probably do not need to write to EEPROM all the time
     if (DEBUG) {Serial.println("Saving Parameters.");}
     DisplayStatus();
   }

  if(Serial.available()) {
      opState = INPUT_HANDLE;
      if (DEBUG) {Serial.println("Serial input available.");}
  }
  else { 
      opState = RUN; 
  }

  delay(100);
}

// ************************************************
// Handle serial input
// ************************************************
void InputHandle()
{
  char inBuff[ ] = "--------";
  double tmp;
  int bytesread;
  String value = "0.000000";
  String command = " ";
  
  if (Serial.available()) {
    Serial.setTimeout(1 ); // Serial read timeout
    bytesread=Serial.readBytesUntil('\n', inBuff, 9); // Read from serial until CR is read or timeout exceeded
    inBuff[bytesread]='\0';
    String instruction = String(inBuff);
    command = instruction.substring(0,1); 
    if (bytesread > 1) { // we have also a value
       value = instruction.substring(1,bytesread); 
    }
    if (DEBUG) {Serial.print(command); Serial.print(" "); Serial.println(value);}

    if (command =="s") { // set setpoint
       tmp = value.toFloat();
       if (((tmp > 0.0) & (tmp < 50.0))) {
         Setpoint = tmp; }
       else { 
         Serial.println("Setpoint out of valid Range"); 
       }
       Serial.print("Setpoint is:  ");
       Serial.println(Setpoint);
       DoControl();
       opState = RUN;
     } else if (command == "p") { //set kP
       tmp=value.toFloat();
       Kp = tmp;
       Serial.print("Kp is:  ");
       Serial.println(Kp);
       DoControl();
       opState = RUN;
    } else if (command == "i") { // set kI
       tmp=value.toFloat();
       Ki = tmp;
       Serial.print("Ki is:  ");
       Serial.println(Ki);
       DoControl();
       opState = RUN;
    } else if (command == "d") {  // set kD
       tmp=value.toFloat();
       Kd = tmp;
       Serial.print("Kd is:  ");
       Serial.println(Kd);
       DoControl();
       opState = RUN;
    } else if (command == "A") { // set autotune 
        if (abs(float(Input) - float(Setpoint)) <= 0.5)  // Should be at steady-state
        {
          StartAutoTune();
          Serial.print("Tuning is:  ");
          Serial.println(tuning);
        }
        else {
          Serial.println("Can not start tuning. Need to be within 0.5 of Setpoint.");
        }
        opState = RUN;
    } else if (command == "a") { // set autotune 
        aTune.Cancel();
        myPID.SetMode(ATuneModeRemember);
        tuning = false;
        Serial.print("Tuning is:  ");
        Serial.println(tuning);
        opState = RUN;
    } else if (command == "o") { // power off
        Serial.println("Switched to OFF.");
        poweroff = true;
        opState = RUN;
    } else if (command == "O") { // power ON
        Serial.println("Switched to OFF.");
        poweroff = false;
        opState = RUN;
    } else { // invalid input, display status
        opState = STATUS;        
    } // end if else switch
  } // end serial available
} // end Input

// ************************************************
// Display status on serial line
// ************************************************
void DisplayStatus()
{
  Serial.print("Temperature: ");
  Serial.print(Input);
  Serial.print(", Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(", SensorValue: ");
  Serial.print(SensorValue);
  Serial.print(", Output is: ");
  Serial.print(Output);
  Serial.print(", Kp: ");
  Serial.print(Kp);
  Serial.print(", Ki: ");
  Serial.print(Ki);
  Serial.print(", Kd: ");
  Serial.print(Kd);
  Serial.print(", Tuning: ");
  Serial.println(tuning);
  opState = RUN;
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  SensorValue = analogRead(SensorPin);
  if (SensorValue < 157) { Input = 99.9; }
  else if (SensorValue > 637) { Input = -99.9;}
  else { Input = double(pgm_read_word(&Table[(SensorValue-157)]))/100; }
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if((now - windowStartTime)>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     digitalWrite(RelayPin,LOW);
  }
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}
