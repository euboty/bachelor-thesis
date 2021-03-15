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
const byte ENDSWITCH_RIGHT = 16;
const byte ENDSWITCH_LEFT = 17;

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
   Perspective is mover to foil:
   Os_Right (OS Right)
    open 112
    closed 53
   Os_Left (OS Left)
    open 47
    closed 106
   //DEEGREES FOR KSA
   Ksa_Left (KSA Left)
    extended 135
    pull 33
    middle 86
   Ksa_Right (KSA Right)
    extended 43
    pull 148
    middle 88
*/

// OS Servo Callibration - Degrees
int os_pos_right;   // tracks is position in software
const int OS_POS_OPEN_RIGHT = 112;
const int OS_POS_CLOSED_RIGHT = 53;
int os_pos_left;    // tracks is position in software
const int OS_POS_OPEN_LEFT = 47;
const int OS_POS_CLOSED_LEFT = 106;

// KSA Servo Callibration - Degrees
const double KSA_DURATION = 200; // desired ksa time in milliseconds
const double KSA_POS_EXT = 8.4;
const double KSA_POS_PULL = 0;
const double KSA_MOTION_RANGE = KSA_POS_EXT - KSA_POS_PULL;
const double KSA_POS_MID_RIGHT = 88;
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
const int FORCE_TARGET_HREG = 1;
const int POSITION_KSA_HREG = 2;
const int KP_HREG = 3;
const int I_HREG = 4;
const int D_HREG = 5;
const int PID_INFLUENCE_HREG = 6; // how much influence can the PID Controller take on KSA position in %
const int OS_OFFSET_HREG = 7;                                                                                                             // DO MEEEEEEEEEEEEEEEEEEEEEEE

// Input Register (READ)
const int FORCE_RIGHT_REAL_IREG = 0;
const int FORCE_LEFT_REAL_IREG = 1;
const int FORCE_COMBINED_REAL_IREG = 2;                                                                                                   // DO MEEEEEEEEEEEEEEEEEEEEEEEE

// VARS
bool os_endswitch_right;
bool os_endswitch_left;
byte mode_ = 0; // program for boot -> do nothing
double loadcell_target;
double ksa_position;
double Kp = 0, Ki = 0, Kd = 0; // set through modbus client
double pid_influence;  // in percent of ksa motion range for semiAdaptive program
double os_offset = 0;                                                                                                                          // DO MEEEEEEEEEEEEEEEEEEEEEEEE
double loadcell_value_right;
double loadcell_value_left;
double loadcell_value_combined;                                                                                                                      // DO MEEEEEEEEEEEEEEEEEEEEEEEE

// Loadcell HX711 Constructor (dout pin, sck pin)
HX711_ADC Loadcell_Right(LOADCELL_RIGHT_DOUT_PIN, LOADCELL_RIGHT_SCK_PIN);
HX711_ADC Loadcell_Left(LOADCELL_LEFT_DOUT_PIN, LOADCELL_LEFT_SCK_PIN);

// Servos
ServoEasing Os_Right;
ServoEasing Os_Left;
ServoEasing Ksa_Right;
ServoEasing Ksa_Left;

// PID
PID Ksa_PID_Adaptive(&loadcell_value_combined, &ksa_position, &loadcell_target, Kp, Ki, Kd, DIRECT); // Input, Output, Setpoint, Kp, Ki, Kd
double ksa_position_change;
PID Ksa_PID_SemiAdaptive(&loadcell_value_combined, &ksa_position_change, &loadcell_target, Kp, Ki, Kd, DIRECT); // Input, Output, Setpoint, Kp, Ki, Kd

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

  Serial.begin(57600);     // baudrate (communication speed) of serial port (:= 57,6 kbps)
  delay(10);

  /* *************************************************************
   * *************************** WIFI: ***************************
   * *************************************************************/

  Serial.println();
  Serial.println();
  Serial.println("Establishing wifi connection");
  wifiManager.autoConnect("KontiBat_Greifer_WlanConfig");   // access @ 192.168.4.1

  /* **************************************************************
   * ******************** LED & ENDSWITCHES: **********************
   * **************************************************************/

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
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

  Os_Right.setEasingType(EASE_CUBIC_OUT); // position curve type
  Os_Left.setEasingType(EASE_CUBIC_OUT);  // position curve type

  // move servos to starting position and save position
  Os_Right.write(OS_POS_OPEN_RIGHT);
  Os_Left.write(OS_POS_OPEN_LEFT);
  double alpha = ksaPosToDegree(KSA_POS_EXT);
  Ksa_Right.write(KSA_POS_MID_RIGHT + alpha);
  Ksa_Left.write(KSA_POS_MID_LEFT - alpha);

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

  // initialize pids
  Ksa_PID_Adaptive.SetMode(AUTOMATIC);
  Ksa_PID_Adaptive.SetSampleTime(1);  // calculate PID every millisecond
  Ksa_PID_SemiAdaptive.SetMode(AUTOMATIC);
  Ksa_PID_SemiAdaptive.SetSampleTime(1);  // calculate PID every millisecond

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
  mb.addHreg(FORCE_TARGET_HREG);
  mb.addHreg(POSITION_KSA_HREG);
  mb.addHreg(KP_HREG);
  mb.addHreg(I_HREG);
  mb.addHreg(D_HREG);
  mb.addHreg(PID_INFLUENCE_HREG);
  mb.addHreg(OS_OFFSET_HREG);

  // Input Register - int - read only
  mb.addIreg(FORCE_RIGHT_REAL_IREG);
  mb.addIreg(FORCE_LEFT_REAL_IREG);
  mb.addIreg(FORCE_COMBINED_REAL_IREG);

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
  }
  else if (mode_ == 4) {
    configuredKSA();  // sets mode back to 0 after one ksa iteration
  }
  else if (mode_ == 5) {
    adaptiveKSA();
  }
  else if (mode_ == 6) {
    semiAdaptiveKSA();  // sets mode back to 0 after one ksa iteration
  }
  else if (mode_ == 7) {
    controlledKSA();
  }
  else if (mode_ == 8) {
    debugViaSerial();
  }
  //catchLooptime();  // for testing only
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
    Os_Right.setEaseToD(OS_POS_OPEN_RIGHT, 1000); // (postion, time to get there)
    Os_Left.setEaseToD(OS_POS_OPEN_LEFT, 1000); // (postion, time to get there)
    synchronizeAllServosStartAndWaitForAllServosToStop(); // moves in blocking mode
    os_pos_right = OS_POS_OPEN_RIGHT;
    os_pos_left = OS_POS_OPEN_LEFT;
  }
}

void closeOS() {
  /*
     Tests necessary: What is the fastest speed the servos can follow?
  */
  if (os_pos_right != OS_POS_CLOSED_RIGHT || os_pos_left != OS_POS_CLOSED_LEFT) { // check if already closed
    Os_Right.setEaseToD(OS_POS_CLOSED_RIGHT, 1000); // (postion, time to get there)
    Os_Left.setEaseToD(OS_POS_CLOSED_LEFT, 1000); // (postion, time to get there)
    synchronizeAllServosStartAndWaitForAllServosToStop(); // moves in blocking mode
    os_pos_right = OS_POS_CLOSED_RIGHT;
    os_pos_left = OS_POS_CLOSED_LEFT;
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

  mb.Hreg(MODE_HREG, 0);  // otherwise it keeps taring over and over which causes problems
}

void configuredKSA() {
  /*
     Positioning of KSA servos with following desired curve
  */
  static bool in_action_flag = false; // indication if a ksa movement ist taking place already
  static unsigned long t_start = 0;

  if (!in_action_flag) {
    t_start = millis();
    in_action_flag = true;
  }

  double ff = 1 / (KSA_DURATION * 1000); // frequency
  unsigned long t = millis();

  if ((t - t_start) <= KSA_DURATION) {
    ksa_position = KSA_MOTION_RANGE * sin((PI * ff) * (t - t_start));
    double alpha = ksaPosToDegree(ksa_position);
    Ksa_Right.write(KSA_POS_MID_RIGHT + alpha);
    Ksa_Left.write(KSA_POS_MID_LEFT - alpha);
  }
  else {
    in_action_flag = false;
    mb.Hreg(MODE_HREG, 0);  // one ksa execution is over, go back to do nothing
  }
}

void adaptiveKSA() {
  /*
     Positioning of ksa servos with pure pid control
  */
  Ksa_PID_Adaptive.SetOutputLimits(KSA_POS_PULL, KSA_POS_EXT);                                                                        // CHECKEN OB ICH HIER WAS VERTAUSCHT HABEEEEE
  Ksa_PID_Adaptive.SetTunings(Kp, Ki, Kd);
  Ksa_PID_Adaptive.Compute();  //PID
  double alpha = ksaPosToDegree(ksa_position);
  Ksa_Left.write(KSA_POS_MID_RIGHT + alpha);
  Ksa_Right.write(KSA_POS_MID_LEFT - alpha);
}

void semiAdaptiveKSA() {
  /*
     Positioning of ksa servos with configuredKSA and error handling with pid
  */
  configuredKSA();  // starts timing and takes care of ending the programm after one iteration
  double pid_motion_range = (KSA_MOTION_RANGE * pid_influence);
  Ksa_PID_SemiAdaptive.SetOutputLimits(-(pid_motion_range/2), (pid_motion_range/2));                                                                           // CHECKEN OB ICH HIER WAS VERTAUSCHT HABEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
  Ksa_PID_SemiAdaptive.SetTunings(Kp, Ki, Kd);
  Ksa_PID_SemiAdaptive.Compute();  //PID

  double alpha = ksaPosToDegree(ksa_position + ksa_position_change);
  Ksa_Left.write(KSA_POS_MID_RIGHT + alpha);
  Ksa_Right.write(KSA_POS_MID_LEFT - alpha);
}

void controlledKSA() {
  double alpha = ksaPosToDegree(ksa_position);
  Ksa_Right.write(KSA_POS_MID_RIGHT + alpha);
  Ksa_Left.write(KSA_POS_MID_LEFT - alpha);
}

void debugViaSerial() {
  serialPrintSystemData();  // command out what you dont want to see
  serialPrintReceivedData();  // command out what you dont want to see
}

void serialPrintSystemData() {
  Serial.print(" Setup failure occured: ");
  Serial.print(mb.Ists(SETUP_FAILURE_ISTS));
  Serial.print(" Loadcell Left: ");
  Serial.print(loadcell_value_left);
  Serial.print(" Loadcell Right: ");
  Serial.print(loadcell_value_right);
  Serial.print(" Loadcell Mean: ");
  Serial.print(loadcell_value_combined);
  Serial.print(" Endswitch Left: ");
  Serial.print(os_endswitch_left);
  Serial.print(" Endswitch Right: ");
  Serial.print(os_endswitch_right);
}

void serialPrintReceivedData() {
  Serial.print(" Mode: ");
  Serial.print(mode_);
  Serial.print(" Loadcell Target: ");
  Serial.print(loadcell_target);
  Serial.print(" Demanded KSA Position: ");
  Serial.print(ksa_position);
  Serial.print(" Kp, Ki, Kd: ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print(Ki);
  Serial.print(", ");
  Serial.print(Kd);
  Serial.print(" PID Influence: ");
  Serial.println(pid_influence);
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
    //catchTimeBetweenMeasurements(); // uncommand if needed
  }

  Loadcell_Left.update();

  if ((newDataReady)) {     // you could filter data with moving average but it would intruduce lag
    loadcell_value_right = Loadcell_Right.getData();
    loadcell_value_left = Loadcell_Left.getData();
    loadcell_value_combined = (loadcell_value_right + loadcell_value_left);
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
  mb.Ireg(FORCE_RIGHT_REAL_IREG, short(loadcell_value_right * 10));  // have to be ints thats why we scale up on serverside to not lose precision
  mb.Ireg(FORCE_LEFT_REAL_IREG, short(loadcell_value_left * 10));
}

void getModbus() {
  mode_ = mb.Hreg(MODE_HREG);
  loadcell_target = mb.Hreg(FORCE_TARGET_HREG) / 9.81;  //--> mN to gramm
  ksa_position = mb.Hreg(POSITION_KSA_HREG);
  Kp = (double)mb.Hreg(KP_HREG) / 1000;
  Ki = (double)mb.Hreg(I_HREG) / 1000;
  Kd = (double)mb.Hreg(D_HREG) / 1000;
  pid_influence = (double)mb.Hreg(PID_INFLUENCE_HREG) / 10; // passed in promille
}

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

double ksaPosToDegree(double desired_ksa_pos) {
  //modified binary search in look up table

  double const lut_ksa[82] = {0.091, 0.190, 0.289, 0.389, 0.489, 0.589, 0.689, 0.790, 0.890, 0.991, 1.093, 1.194, 1.296, 1.398, 1.500, 1.602, 1.704, 1.807, 1.910, 2.013, 2.116, 2.219, 2.323, 2.426, 2.530, 2.634, 2.737, 2.842, 2.946, 3.050, 3.154, 3.258, 3.363, 3.467, 3.572, 3.677, 3.781, 3.886, 3.991, 4.095, 4.200, 4.305, 4.409, 4.514, 4.619, 4.723, 4.828, 4.933, 5.037, 5.141, 5.246, 5.350, 5.454, 5.558, 5.662, 5.766, 5.870, 5.974, 6.077, 6.180, 6.283, 6.386, 6.489, 6.592, 6.694, 6.797, 6.899, 7.000, 7.102, 7.203, 7.304, 7.405, 7.506, 7.606, 7.706, 7.806, 7.905, 8.004, 8.103, 8.201, 8.300, 8.397};
  int const lut_ksa_length = 82;
  double const start_degree = -20;
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
