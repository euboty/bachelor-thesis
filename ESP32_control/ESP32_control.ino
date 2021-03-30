/*
************************************************************************************************************
   This program is the main program for the mechatronic gripper
   Author: Florian Geissler
************************************************************************************************************
*/

/*
 * **********************************************************************************************************
 * ********************************************  LIBRARIES **************************************************
 * **********************************************************************************************************
*/

#include <HX711_ADC.h>            //https://github.com/olkal/HX711_ADC
#include <ServoEasing.h>          //https://github.com/ArminJo/ServoEasing
#include <PID_v1.h>               //https://github.com/br3ttb/Arduino-PID-Library
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ModbusIP_ESP8266.h>     //https://github.com/emelianov/modbus-esp8266

/*
 * **********************************************************************************************************
 * *********************************************  GLOBAL ****************************************************
 * **********************************************************************************************************
*/

// LED Pinout
const byte LED_BUILTIN = 1;

// Endswitch Pinout
const byte ENDSWITCH_RIGHT = 17;
const byte ENDSWITCH_LEFT = 16;

// Test Bench Pinout
const byte TESTBENCH = 13;

// Loadcell Pinout
const byte LOADCELL_RIGHT_DOUT_PIN = 22;
const byte LOADCELL_RIGHT_SCK_PIN = 23;
const byte LOADCELL_LEFT_DOUT_PIN = 18;
const byte LOADCELL_LEFT_SCK_PIN = 19;

// Servo Pinout
const byte OS_RIGHT_PIN = 27;
const byte OS_LEFT_PIN = 26;
const byte KSA_RIGHT_PIN = 33;
const byte KSA_LEFT_PIN = 32;

// Loadcell Setup Vars
// calibration value for each loadcell - if loadcells deliver wrong measurements: recalibrate with dedicated program "Loadcell_calibration" in this folder
const float LOADCELL_CAL_RIGHT = 1985.14;
const float LOADCELL_CAL_LEFT = 1948.98;
long stabilizingtime = 2000; // setup tare precision can be improved by adding a few seconds of stabilizing time
boolean _tare = true; // set this to false if you don't want tare to be performed in setup

/*
   Servo Callibration:
   Following values describe the opening and closing positions of the servos.
   The sketch "Servo_callibration" was used to figure those values.
*/

// OS Servo Callibration - Degrees
double os_pos_right;   // tracks is position in software
const double OS_POS_OPEN_RIGHT = 114;
const double OS_POS_CLOSED_RIGHT = 52;
double os_pos_left;    // tracks is position in software
const double OS_POS_OPEN_LEFT = 47;
const double OS_POS_CLOSED_LEFT = 106;

// KSA Servo Callibration - Degrees
double ksa_position;
const double KSA_POS_EXT = 0;
const double KSA_POS_PULL = -8.4;
const double KSA_MOTION_RANGE = KSA_POS_EXT + KSA_POS_PULL;
const double KSA_POS_MID_RIGHT = 92;
const double KSA_POS_MID_LEFT = 86;

// Modbus Registers Offsets
// Coil (READ&WRITE)
// none

// Discrete Input (READ)
const int SETUP_FAILURE_ISTS = 0;
const int OS_ENDSWITCH_RIGHT_ISTS = 1;
const int OS_ENDSWITCH_LEFT_ISTS = 2;

// Holding Register (READ&WRITE)
const int MODE_HREG = 0;
const int OS_OFFSET_HREG = 1;
const int KSA_POSITION_TARGET_HREG = 2;
const int MOVERSPEED_HREG = 3;
const int KSA_WAY_LENGTH_HREG = 4;
const int FORCE_TARGET_HREG = 5;
const int KP_HREG = 6;
const int KI_HREG = 7;
const int KD_HREG = 8;

// Input Register (READ)
const int FORCE_COMBINED_REAL_IREG = 0;
const int FORCE_RIGHT_REAL_IREG = 1;
const int FORCE_LEFT_REAL_IREG = 2;

// VARS
bool os_endswitch_right;
bool os_endswitch_left;
byte mode_ = 0; // program for boot -> do nothing
double os_offset;
double ksa_position_target;
double moverspeed;
double ksa_way_length;
double ksa_duration;
double force_MilliN_target;
double Kp, Ki, Kd;
double force_milliN_combined;
double force_milliN_right;
double force_milliN_left;

// Loadcell HX711 Constructor (dout pin, sck pin)
HX711_ADC Loadcell_Right(LOADCELL_RIGHT_DOUT_PIN, LOADCELL_RIGHT_SCK_PIN);
HX711_ADC Loadcell_Left(LOADCELL_LEFT_DOUT_PIN, LOADCELL_LEFT_SCK_PIN);

// Servos
ServoEasing Os_Right;
ServoEasing Os_Left;
ServoEasing Ksa_Right;
ServoEasing Ksa_Left;

// PID
PID Ksa_PID_Adaptive(&force_milliN_combined, &ksa_position, &force_MilliN_target, Kp, Ki, Kd, REVERSE); // Input, Output, Setpoint, Kp, Ki, Kd

// WIFI
WiFiManager wifiManager;

// Modbus object
ModbusIP mb;

/*
 * **********************************************************************************************************
 * **********************************************  SETUP  ***************************************************
 * **********************************************************************************************************
*/

void setup() {
  /*
     Setup Program - just runs once on startup. Initializes led, endswitches, servos, loadcells and PID and sets start vars.
     Status is displayed through buildin LED.
  */

  bool setup_failure = false;

  /* *************************************************************
   * ************************** GENERAL: *************************
   * *************************************************************/

  Serial.begin(115200);     // baudrate (communication speed) of serial port (:= 115,2 kbps)
  delay(10);

  /* *************************************************************
   * *************************** WIFI: ***************************
   * *************************************************************/

  Serial.println();
  Serial.println();
  Serial.println("Establishing wifi connection");
  wifiManager.autoConnect("KontiBat_Greifer_WlanConfig");   // access @ 192.168.4.1

  /* **************************************************************
   * ************** LED & ENDSWITCHES & Testbench: ****************
   * **************************************************************/

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(TESTBENCH, OUTPUT);
  digitalWrite(TESTBENCH, HIGH);
  pinMode(ENDSWITCH_RIGHT, INPUT_PULLUP);
  pinMode(ENDSWITCH_LEFT, INPUT_PULLUP);

  /* **************************************************************
   * ************************* SERVOS: ****************************
   * **************************************************************/

  // configuration of servos
  if (Os_Right.attach(OS_RIGHT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Os_Right"));
    setup_failure = true;
  }
  if (Os_Left.attach(OS_LEFT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Os_Right"));
    setup_failure = true;
  }
  if (Ksa_Right.attach(KSA_RIGHT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Ksa_Right"));
    setup_failure = true;
  }
  if (Ksa_Left.attach(KSA_LEFT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Ksa_Left"));
    setup_failure = true;
  }

  Os_Right.setEasingType(EASE_SINE_OUT); // position curve type
  Os_Left.setEasingType(EASE_SINE_OUT);  // position curve type

  // move servos to starting position and save position
  Os_Right.write(OS_POS_OPEN_RIGHT);
  Os_Left.write(OS_POS_OPEN_LEFT);
  double alpha = ksaPosToDegree(KSA_POS_EXT);
  Ksa_Right.write(KSA_POS_MID_RIGHT - alpha);
  Ksa_Left.write(KSA_POS_MID_LEFT + alpha);

  os_pos_right = OS_POS_OPEN_RIGHT;
  os_pos_left = OS_POS_OPEN_LEFT;

  /* **************************************************************
   * *********************** LOADCELLS: ***************************
   * **************************************************************/

  ledDelayBlink(3000, 3); // indicator loadcells are getting tared

  // loadcell startup
  byte loadcell_right_rdy = false;
  byte loadcell_left_rdy = false;
  Loadcell_Right.begin();
  Loadcell_Left.begin();
  // run startup, stabilization and tare, both modules simultaniously
  while (!(loadcell_left_rdy && loadcell_right_rdy)) {
    if (!loadcell_right_rdy) loadcell_right_rdy = Loadcell_Right.startMultiple(stabilizingtime, _tare);
    if (!loadcell_left_rdy) loadcell_left_rdy = Loadcell_Left.startMultiple(stabilizingtime, _tare);
  }
  if (Loadcell_Right.getTareTimeoutFlag() || Loadcell_Right.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 right wiring and pin designations");
    setup_failure = true;
  }
  if (Loadcell_Left.getTareTimeoutFlag() || Loadcell_Left.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 left wiring and pin designations");
    setup_failure = true;
  }
  Loadcell_Right.setCalFactor(LOADCELL_CAL_RIGHT);
  Loadcell_Left.setCalFactor(LOADCELL_CAL_LEFT);

  /* **************************************************************
   * ******************** PID Controller: *************************
   * **************************************************************/

  Ksa_PID_Adaptive.SetMode(AUTOMATIC);
  Ksa_PID_Adaptive.SetSampleTime(12.5);  // calculate PID every millisecond, move higher to stabelize

  /* **************************************************************
   * ************************ MODBUS: *****************************
   * **************************************************************/

  mb.server();  // start modbus server

  // Discrete Input - bool - read only
  mb.addIsts(SETUP_FAILURE_ISTS);
  mb.addIsts(OS_ENDSWITCH_RIGHT_ISTS);
  mb.addIsts(OS_ENDSWITCH_LEFT_ISTS);

  // Holding Register - int - read and write
  mb.addHreg(MODE_HREG);
  mb.addHreg(OS_OFFSET_HREG);
  mb.addHreg(KSA_POSITION_TARGET_HREG);
  mb.addHreg(MOVERSPEED_HREG);
  mb.addHreg(KSA_WAY_LENGTH_HREG);
  mb.addHreg(FORCE_TARGET_HREG);
  mb.addHreg(KP_HREG);
  mb.addHreg(KI_HREG);
  mb.addHreg(KD_HREG);

  // Input Register - int - read only
  mb.addIreg(FORCE_COMBINED_REAL_IREG);
  mb.addIreg(FORCE_RIGHT_REAL_IREG);
  mb.addIreg(FORCE_LEFT_REAL_IREG);

  // Initial Values
  mb.Hreg(MOVERSPEED_HREG, moverspeed);
  mb.Hreg(KSA_WAY_LENGTH_HREG, ksa_way_length);
  mb.Hreg(KP_HREG, Kp);


  /* **************************************************************
   * ********************** SETUP DONE: ***************************
   * **************************************************************/

  mb.Ists(SETUP_FAILURE_ISTS, setup_failure);  // write setup_failure state in MODBUS output
  ledDelayBlink(3000, 5); // setup routine done indicator
  Serial.println("Setup complete!");
}

/*
 * **********************************************************************************************************
 * **********************************************  LOOP  ****************************************************
 * **********************************************************************************************************
*/

void loop() {
  /*
     Main loop of program
  */
  getLoadcells();
  getEndswitches();
  setModbus();  // write device data to modbus regs
  mb.task();  // call once inside loop - all modbus magic here
  getModbus();  // copy input Modbus regs to local vars

  // state machine
  if (mode_ == 1) {
    openOS();
  }
  else if (mode_ == 2) {
    closeOS();
  }
  else if (mode_ == 3) {
    tareLoadcells();  // sets mode back to 0 after taring is done
    mb.Hreg(MODE_HREG, 0);  // otherwise it keeps taring over and over which causes problems
  }
  else if (mode_ == 4) {
    controlledKSA();
  }
  else if (mode_ == 5) {
    if (controlledConfiguredKSA()) {
      mb.Hreg(MODE_HREG, 0);  // one ksa execution is over, go back to nothing
    }
  }
  else if (mode_ == 6) {
    adaptiveKSA();
  }
  else if (mode_ == 7) {
    adaptiveConfiguredKSA();  // sets mode back to 0 after one ksa iteration
  }
  else if (mode_ == 8) {
    debugViaSerial();
  }
  //catchLooptime();  // uncommand for validation only
}

/*
 * **********************************************************************************************************
 * *************************************  STATE MACHINE FUNCTIONS  ******************************************
 * **********************************************************************************************************
*/

void openOS() {
  /*
     Tests necessary: What is the fastest speed the servos can follow?
  */
  if ( os_pos_right != OS_POS_OPEN_RIGHT || os_pos_left != OS_POS_OPEN_LEFT) {  // check if already open
    Os_Right.setEaseToD(OS_POS_OPEN_RIGHT, 120); // (postion, time to get there)
    Os_Left.setEaseToD(OS_POS_OPEN_LEFT, 120); // (postion, time to get there)
    synchronizeAllServosStartAndWaitForAllServosToStop(); // moves in blocking mode
    os_pos_right = OS_POS_OPEN_RIGHT;
    os_pos_left = OS_POS_OPEN_LEFT;
  }
}

void closeOS() {
  /*
     Tests necessary: What is the fastest speed the servos can follow?
  */
  if (os_pos_right != (OS_POS_CLOSED_RIGHT + os_offset) || os_pos_left != (OS_POS_CLOSED_LEFT - os_offset)) { // check if already closed
    Os_Right.setEaseToD((OS_POS_CLOSED_RIGHT + os_offset), 180); // (postion, time to get there)
    Os_Left.setEaseToD((OS_POS_CLOSED_LEFT - os_offset), 180); // (postion, time to get there)
    synchronizeAllServosStartAndWaitForAllServosToStop(); // moves in blocking mode
    os_pos_right = OS_POS_CLOSED_RIGHT + os_offset;
    os_pos_left = OS_POS_CLOSED_LEFT - os_offset;
  }
}

void tareLoadcells() {
  /*
     Tares both loadcells.
  */
  Loadcell_Right.tareNoDelay();
  Loadcell_Left.tareNoDelay();

  //check if last tare operation is complete
  while (Loadcell_Right.getTareStatus() == false) {
    // wait
    Serial.println("Taring...");
  }
  while (Loadcell_Left.getTareStatus() == false) {
    // wait
    Serial.println("Taring...");
  }
}

void controlledKSA() {
  double alpha = ksaPosToDegree(ksa_position_target);
  Ksa_Right.write(KSA_POS_MID_RIGHT - alpha);
  Ksa_Left.write(KSA_POS_MID_LEFT + alpha);
  
  // for validation only
  Serial.print(ksa_position_target);
  Serial.print(";");
  Serial.print(force_milliN_combined);
  Serial.print(";");
  Serial.print(force_milliN_right);
  Serial.print(";");
  Serial.println(force_milliN_left);
}

bool controlledConfiguredKSA() {
  /*
     Positioning of KSA servos with following desired curve
  */
  static bool in_action_flag = false; // indication if a ksa movement ist taking place already
  static unsigned long t_start = 0;

  if (!in_action_flag) {
    in_action_flag = true;
    digitalWrite(TESTBENCH, LOW);  // uncommand for testing
    delayMicroseconds(20);           // uncommand for testing
    digitalWrite(TESTBENCH, HIGH); // uncommand for testing
    t_start = millis();
  }

  double ff = 1 / ksa_duration; // frequency
  unsigned long t = millis();

  if ((t - t_start) <= ksa_duration) {
    ksa_position = KSA_MOTION_RANGE * sin((PI * ff) * (t - t_start));
    double alpha = ksaPosToDegree(ksa_position);
    Ksa_Right.write(KSA_POS_MID_RIGHT - alpha);
    Ksa_Left.write(KSA_POS_MID_LEFT + alpha);
    // for validation only
    Serial.print(ksa_position);
    Serial.print(";");
    Serial.print(force_milliN_combined);
    Serial.print(";");
    Serial.print(force_milliN_right);
    Serial.print(";");
    Serial.println(force_milliN_left);
    return false; // ksa movement not done yet
  }
  else {
    in_action_flag = false;
    return true;  // ksa movement done
  }
}

void adaptiveKSA() {
  /*
     Positioning of ksa servos with pure pid control
  */
  Ksa_PID_Adaptive.SetOutputLimits(KSA_POS_PULL, KSA_POS_EXT);
  Ksa_PID_Adaptive.SetTunings(Kp, Ki, Kd);
  Ksa_PID_Adaptive.Compute();  //PID
  double alpha = ksaPosToDegree(ksa_position);
  Ksa_Right.write(KSA_POS_MID_RIGHT - alpha);
  Ksa_Left.write(KSA_POS_MID_LEFT + alpha);
  
  // for validation only
  Serial.print(ksa_position);
  Serial.print(";");
  Serial.print(force_milliN_combined);
  Serial.print(";");
  Serial.print(force_milliN_right);
  Serial.print(";");
  Serial.println(force_milliN_left);
}

bool adaptiveConfiguredKSA() {
  /*
     Positioning of ksa servos with pure pid control follwówing configured force target curve
  */
  // placeholder program, implement just like controlledConfiguredKSA but with adaptiveKSA() and force_MilliN_target, instead of controlledKSA() and ws_position
}

void debugViaSerial() {
  serialPrintSystemData();  // command out what you dont want to see
  serialPrintReceivedData();  // command out what you dont want to see
}

void serialPrintSystemData() {
  Serial.print(" Setup failure occured: ");
  Serial.print(mb.Ists(SETUP_FAILURE_ISTS));
  Serial.print(" Endswitch Right: ");
  Serial.print(os_endswitch_right);
  Serial.print(" Endswitch Left: ");
  Serial.print(os_endswitch_left);
  Serial.print(" Force MilliN Combined: ");
  Serial.print(force_milliN_combined);
  Serial.print(" Force MilliN Right: ");
  Serial.print(force_milliN_right);
  Serial.print(" Force MilliN Left: ");
  Serial.print(force_milliN_left);
}

void serialPrintReceivedData() {
  Serial.print(" Mode: ");
  Serial.print(mode_);
  Serial.print(" OS Offset: ");
  Serial.print(os_offset);
  Serial.print(" KSA Position Target: ");
  Serial.print(ksa_position_target);
  Serial.print(" Moverspeedn: ");                             
  Serial.print(moverspeed);
  Serial.print(" KSA Way Length: ");
  Serial.print(ksa_way_length);
  Serial.print(" Force MilliN Target: ");
  Serial.print(force_MilliN_target);
  Serial.print(" Kp, Ki, Kd: ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print(Ki);
  Serial.print(", ");
  Serial.println(Kd);
}

/*
 * **********************************************************************************************************
 * ****************************************  GENERAL FUNCTIONS  *********************************************
 * **********************************************************************************************************
*/

void ledDelayBlink(int delay_time, int blink_times) {
  /*
     Indicates Setup Progress with blinking. Acts blocking.
  */
  for (int i = 0; i < blink_times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delay_time / (blink_times * 2));
    digitalWrite(LED_BUILTIN, LOW);
    delay(delay_time / (blink_times * 2));
  }
}

void getLoadcells() {
  static boolean newDataReady = false;

  // check for new data/start next conversion:
  if (Loadcell_Right.update()) {
    newDataReady = true;
    //catchTimeBetweenMeasurements(); // uncommand for validation only
  }

  Loadcell_Left.update();

  if ((newDataReady)) {     // you could filter data with moving average but it would intruduce lag
    force_milliN_right = Loadcell_Right.getData() * 9.81;
    force_milliN_left = Loadcell_Left.getData() * 9.81;
    force_milliN_combined = (force_milliN_right + force_milliN_left);
    newDataReady = false;
  }
}

void getEndswitches() {
  os_endswitch_right = !digitalRead(ENDSWITCH_RIGHT);  // internal pullup resistor inverts logic
  os_endswitch_left = !digitalRead(ENDSWITCH_LEFT);    // internal pullup resistor inverts logic
}

void setModbus() {
  mb.Ists(OS_ENDSWITCH_RIGHT_ISTS, os_endswitch_right);
  mb.Ists(OS_ENDSWITCH_LEFT_ISTS, os_endswitch_left);
  mb.Ireg(FORCE_COMBINED_REAL_IREG, short(force_milliN_combined));
  mb.Ireg(FORCE_RIGHT_REAL_IREG, short(force_milliN_right));  // have to be ints thats why we scale up on serverside to not lose precision
  mb.Ireg(FORCE_LEFT_REAL_IREG, short(force_milliN_left));
}

void getModbus() {
  // conversions because MODBUS cant handle floats
  mode_ = mb.Hreg(MODE_HREG);
  os_offset = (double)((short)mb.Hreg(OS_OFFSET_HREG)) / 1000;
  ksa_position_target = (double)((short)mb.Hreg(KSA_POSITION_TARGET_HREG)) / 1000;
  moverspeed = (double)((short)mb.Hreg(MOVERSPEED_HREG));
  ksa_way_length = (double)((short)mb.Hreg(KSA_WAY_LENGTH_HREG));
  force_MilliN_target = (double)((short)mb.Hreg(FORCE_TARGET_HREG));
  Kp = (double)((short)mb.Hreg(KP_HREG)) / 100000;
  Ki = (double)((short)mb.Hreg(KI_HREG)) / 100000;
  Kd = (double)((short)mb.Hreg(KD_HREG)) / 100000;
  ksa_duration = (ksa_way_length/moverspeed)*1000;  // in ms
}

double ksaPosToDegree(double desired_ksa_pos) {
  /*
     Inverse kin with LookUpTable and Binary Search
  */

  double const lut_ksa[141] = { -12.95, -12.88, -12.80, -12.73, -12.66, -12.58, -12.51, -12.43, -12.36, -12.28, -12.21, -12.13, -12.05, -11.97, -11.89, -11.82, -11.74, -11.66, -11.58, -11.50, -11.41, -11.33, -11.25, -11.17, -11.09, -11.00, -10.92, -10.83, -10.75, -10.67, -10.58, -10.49, -10.41, -10.32, -10.23, -10.15, -10.06, -9.97, -9.88, -9.79, -9.70, -9.62, -9.53, -9.43, -9.34, -9.25, -9.16, -9.07, -8.98, -8.89, -8.79, -8.70, -8.61, -8.51, -8.42, -8.33, -8.23, -8.14, -8.04, -7.95, -7.85, -7.76, -7.66, -7.56, -7.47, -7.37, -7.27, -7.18, -7.08, -6.98, -6.89, -6.79, -6.69, -6.59, -6.49, -6.39, -6.30, -6.20, -6.10, -6.00, -5.90, -5.80, -5.70, -5.60, -5.50, -5.40, -5.30, -5.20, -5.10, -5.00, -4.90, -4.80, -4.70, -4.60, -4.50, -4.40, -4.30, -4.20, -4.10, -4.00, -3.90, -3.80, -3.70, -3.60, -3.50, -3.40, -3.30, -3.20, -3.10, -3.00, -2.90, -2.80, -2.70, -2.60, -2.50, -2.40, -2.30, -2.20, -2.10, -2.01, -1.91, -1.81, -1.71, -1.61, -1.52, -1.42, -1.32, -1.22, -1.13, -1.03, -0.94, -0.84, -0.74, -0.65, -0.55, -0.46, -0.37, -0.27, -0.18, -0.08, 0.01};
  int const lut_ksa_length = 141;
  double const start_degree = -48.5;
  double const steps = 0.5;

  if (desired_ksa_pos < lut_ksa[0]) {
    return start_degree;
  }
  if (desired_ksa_pos > lut_ksa[lut_ksa_length - 1]) {
    return start_degree + steps * (lut_ksa_length - 1);
  }

  int lo = 0;
  int hi = lut_ksa_length - 1;

  while (lo <= hi) {
    int mid = (hi + lo) / 2;

    if (desired_ksa_pos < lut_ksa[mid]) {
      hi = mid - 1;
    } else if (desired_ksa_pos > lut_ksa[mid]) {
      lo = mid + 1;
    } else {
      return start_degree + steps * mid;
    }
  }
  // desired_ksa_pos not in array
  if ((lut_ksa[lo] - desired_ksa_pos) < (desired_ksa_pos - lut_ksa[hi])) {
    return start_degree + steps * lo;
  } else {
    return start_degree + steps * hi;
  }
}

/*
 * **********************************************************************************************************
 * **************************************  VALIDATION FUNCTIONS  ********************************************
 * **********************************************************************************************************
*/

void catchLooptime() {
  static unsigned long prev_millis;
  unsigned long looptime;
  looptime = millis() - prev_millis;
  prev_millis = millis();
  Serial.print("Looptime in ms: ");
  Serial.println(looptime);
}

void catchTimeBetweenMeasurements() {
  static unsigned long prev_millis;
  unsigned long time_between_measurements;
  time_between_measurements = millis() - prev_millis;
  prev_millis = millis();
  Serial.print("Time between Loadcell Measurements: ");
  Serial.println(time_between_measurements);
}
