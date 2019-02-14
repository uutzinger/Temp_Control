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
      
// ************************************************
// Pin definitions
// ************************************************

// Output Relay // will also blink LED
#define RelayPin1 13
#define RelayPin2 12
#define SensorPin A0
int SensorValue = 0;
bool Relay1;
bool Relay2;

char   inBuff[] = "----------------";
double Setpoint = 25.0;
int    bytesread;
String value    = "25.0";
String command  = "o";

#define loopInterval      50 // check 200 times per second
#define controlInterval 1000 // change controler once a second
#define readInterval     100 // read temperature 10 times a second
#define relayInterval   2000 // cool or heat changes once every two seconds

unsigned long      lastRead    = 0;
unsigned long      lastControl = 0;
unsigned long      onTime      = 0;
unsigned long      cycleStart  = 0;
unsigned long      currentTime = 0;
long               waitTime    = 0;
float              dutyCycle;
float              error;
float              Kp          = 20.0;
float              Temperature; 

// ************************************************
// Setup and display initial screen
// ************************************************


void printHelp() {
  Serial.println("417 - 517 Temperature Controller");
  Serial.println("================================");
  Serial.println("Set Setpoint: s25.0");
  Serial.println("Show Setpoint: S");
  Serial.println("Set Proportional:  p25.0");
  Serial.println("Turn off: o");
  Serial.println("Turn on: O");
}

void setup()
{
  while (!Serial && millis() < 3000) ;
  Serial.begin(Serial.baud());
  Serial.println("Temperature Contoller");
  printHelp();
  
   // Initialize Relay Control:
   pinMode(RelayPin1, OUTPUT);    // Output mode to drive relay
   pinMode(RelayPin2, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin1, LOW);  // make sure it is off to start
   digitalWrite(RelayPin2, LOW);  // make sure it is off to start
   // We will measure sensor here
   pinMode(SensorPin, INPUT);
   cycleStart = millis(); 
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   currentTime = millis();

   // Reading Sensor, it will read and display every readInterval ms
   /////////////////////////////////////////////////////////////////
   if ((currentTime - lastRead) > readInterval)  {
     // Read Temperature
     lastRead = currentTime;
     SensorValue = analogRead(SensorPin);
     // Convert to temperature
     if (SensorValue < 157) { Temperature = 99.9; }
     else if (SensorValue > 637) { Temperature = -99.9;}
     else { Temperature = double(Table[(SensorValue-157)])/100.0; } // Lookup table is stored in interger, pgm_read is needed as we read from program memory
     Serial.printf("Temperature: %f.2\n",Temperature);              // http://www.cplusplus.com/reference/cstdio/printf/
   }

   // Update cooling, it will change cooling or heating every controlInterval ms
   /////////////////////////////////////////////////////////////////////////////
   if ( (currentTime - lastControl) >= controlInterval ) {
     lastControl = currentTime;
     error = Setpoint - Temperature;
     if (Error < 0.0) {
         Relay1 = true;
         Relay2 = false;
     } else {
         Relay1 = false;
         Relay2 = true;
     }
     dutyCycle = abs(Error)* Kp;
     if (dutyCycle > 100) { dutyCycle = 100; }
     onTime  = relayInterval * long(dutyCycle/100.0);  
   }
   
   // On/Off Cycle, The sequence restarts every dutyCyle (ms)
   // This makes PWM waveform to control the heater/cooler
   //////////////////////////////////////////////////////////
   if ( (currentTime - cycleStart) <= onTime )  
   {                                                    // Cooler or Heater on
     if (digitalRead(RelayPin1)!= Relay1) { digitalWrite(RelayPin1, Relay1); }
     if (digitalRead(RelayPin2)!= Relay2) { digitalWrite(RelayPin2, Relay2); }
   } else if ((currentTime -cycleStart) <= dutyCycle) { // Cooler or Heater off
     if (digitalRead(RelayPin1)!= false)  { digitalWrite(RelayPin1, false); } 
     if (digitalRead(RelayPin2)!= false)  { digitalWrite(RelayPin2, false); } 
   } else {                                             // Start cycle again
    cycleStart = currentTime;
   }

   // Check serial input/output
   ///////////////////////////////////////////////////////////
   if (Serial.available()) {
     Serial.setTimeout(1);                             // Serial read timeout
     bytesread=Serial.readBytesUntil('\n', inBuff, 16); // Read from serial until CR is read or timeout exceeded
     inBuff[bytesread]='\0';
     String instruction = String(inBuff);
     if (bytesread >0)  { command = instruction.substring(0,1); } 
     if (bytesread > 1) {   value = instruction.substring(1,bytesread); }
     if (command =='s') { // set setpoint
       Setpoint = value.toFloat();
       if (((Setpoint < 0.0) || (Setpoint > 50.0))) { Serial.println("Setpoint out of valid Range"); }
       Serial.printf("Setpoint is: %f\n", Setpoint);
     } else if (command =='S') { // set setpoint
       Serial.printf("Setpoint is: %f\n", Setpoint);
     } else if (command == "p") { //set kP
       Kp = value.toFloat();
       Serial.printf("Kp is: %f.2", Kp);
     } else if (command == "o") { // power off
        Serial.println("Switched to OFF.");
     } else if (command == "O") { // power ON
        Serial.println("Switched to OFF.");
     } else { // invalid input, display status
      printHelp();
     } // end if else switch
   } // end serial available

   // Do nothing
   ///////////////////////////////////////////////////////
   waitTime = loopInterval - (millis() - currentTime);
   if (waitTime > 0) { delay(waitTime); }
}
