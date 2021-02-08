/************************************************************************************************************
   This program is the main program for the bachelor thesis Kontibat mechatronic gripper of Florian Geissler.
   Version: ESP32 in Developement
   Author: Florian Geissler
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

// Program Mode
byte mode_ = 0;

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

// Loadcell HX711 Constructor (dout pin, sck pin)
HX711_ADC Loadcell_Right(LOADCELL_RIGHT_DOUT_PIN, LOADCELL_RIGHT_SCK_PIN);
HX711_ADC Loadcell_Left(LOADCELL_LEFT_DOUT_PIN, LOADCELL_LEFT_SCK_PIN);

// Loadcell Setup Vars
// calibration value for each loadcell - if loadcells deliver wrong measurements: recalibrate with dedicated program "Loadcell_calibration" in Code folder
const float LOADCELL_CAL_RIGHT = 1985.14;
const float LOADCELL_CAL_LEFT = 1948.98;
long stabilizingtime = 2000; // setup tare preciscion can be improved by adding a few seconds of stabilizing time
boolean _tare = true; // set this to false if you don't want tare to be performed in setup

// Loadcell Vars
double loadcell_value_right;
double loadcell_value_left;
double loadcell_value_mean;

// Servo Pinout
const byte OS_RIGHT_PIN = 27;
const byte OS_LEFT_PIN = 26;
const byte WS_RIGHT_PIN = 32;
const byte WS_LEFT_PIN = 33;

// Servos
ServoEasing Os_Right;
ServoEasing Os_Left;
ServoEasing Ws_Right;
ServoEasing Ws_Left;

/*
   Callibration:
   Following values describe the opening and closing positions of the servos (PWM signal).
   The sketch "Servo_callibration" was used to figure those values.
   Perspective is mover to foil:
   //MICROSECONDS FOR OS CAUSE IT WORKS BETTER
   Os_Right (OS Right)
    open 112
    closed 53
   Os_Left (OS Left)
    open 48
    closed 105
   //DEEGREES FOR WS
   Ws_Left (WS Left)
    extended 135
    pull 33
    middle 86
   Ws_Right (WS Right)
    extended 43
    pull 148
    middle 88

    Those are set as postions and a motion range is set (f.e. OS_MOTION_RANGE = 1700 - 1094). This way both servos have to move the same amount.
*/

// OS Servo Callibration - Degrees - servo steps is in 1 degrees --> could also be ints
int os_pos_right;                                                         // tracks is position in software
const int OS_POS_OPEN_RIGHT = 112;
const int OS_POS_CLOSED_RIGHT = 53;
int os_pos_left;                                                          // tracks is position in software
const int OS_POS_OPEN_LEFT = 47;
const int OS_POS_CLOSED_LEFT = 106;

// WS Servo Callibration - Degrees - servo steps is in 1 degrees --> could also be ints
double ws_pos_right;
const double WS_POS_EXT_RIGHT = 41;
const double WS_POS_MID_RIGHT = 88;
//const double WS_POS_PULL_RIGHT = 148;
double ws_pos_left;
const double WS_POS_EXT_LEFT = 135;
const double WS_POS_MID_LEFT = 86;
//const double WS_POS_PULL_LEFT = 33;
const double WS_MOTION_RANGE_2EXT = 48;
const double WS_MOTION_RANGE_2PULL = 53;

unsigned int position_ws;   // demanded ws position given by Modbus Client

// PID
double loadcell_target, pid_output_ws;
double max_influence_controller;  // in percent of ws motion range for semi adaptive program
double Kp = 3, Ki = 22, Kd = 2; // those parameters are just examples and not tuned yet
PID Ws_PID(&loadcell_value_mean, &pid_output_ws, &loadcell_target, Kp, Ki, Kd, DIRECT); // Input, Output, Setpoint, Kp, Ki, Kd

// WIFI
WiFiManager wifiManager;

// Modbus Registers Offsets

// Coil (READ&WRITE)
// none

// Discrete Input (READ)
const int SETUP_FAILURE_ISTS = 0;
const int OS_ENDSWITCH_LEFT_ISTS = 1;
const int OS_ENDSWITCH_RIGHT_ISTS = 2;

// Holding Register (READ&WRITE)
const int MODE_HREG = 0;
const int FORCE_TARGET_HREG = 1;
const int POSITION_WS_HREG = 2;
const int P_HREG = 3;
const int I_HREG = 4;
const int D_HREG = 5;
const int MAX_INFLUENCE_CONTROLLER_HREG = 6; // how much influence can the PID Controller take on WS position in %

// Input Register (READ)
const int FORCE_LEFT_REAL_IREG = 0;
const int FORCE_RIGHT_REAL_IREG = 1;

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
  wifiManager.autoConnect("KontiBat_Greifer_WlanConfig");   //access @ 192.168.4.1

  /* **************************************************************
   * ************************** LED: ******************************
   * **************************************************************/

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  /* **************************************************************
   * ********************** ENDSWITCHES: **************************
   * **************************************************************/

  pinMode(ENDSWITCH_LEFT, INPUT_PULLUP);
  pinMode(ENDSWITCH_RIGHT, INPUT_PULLUP);

  /* **************************************************************
   * ************************* SERVOS: ****************************
   * **************************************************************/

  // configuration of servos
  if (Os_Right.attach(OS_RIGHT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Os_Right"));
    mb.Ists(SETUP_FAILURE_ISTS, true);
  }
  if (Os_Left.attach(OS_LEFT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Os_Right"));
    mb.Ists(SETUP_FAILURE_ISTS, true);
  }
  if (Ws_Right.attach(WS_RIGHT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Ws_Right"));
    mb.Ists(SETUP_FAILURE_ISTS, true);
  }
  if (Ws_Left.attach(WS_LEFT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Ws_Left"));
    mb.Ists(SETUP_FAILURE_ISTS, true);
  }

  Os_Right.setEasingType(EASE_CUBIC_OUT); // position curve type
  Os_Left.setEasingType(EASE_CUBIC_OUT);  // position curve type

  // move servos to starting position and save position
  Os_Right.write(OS_POS_OPEN_RIGHT);
  Os_Left.write(OS_POS_OPEN_LEFT);
  Ws_Right.write(WS_POS_EXT_RIGHT);
  Ws_Left.write(WS_POS_EXT_LEFT);

  os_pos_right = OS_POS_OPEN_RIGHT;
  os_pos_left = OS_POS_OPEN_LEFT;
  ws_pos_right = WS_POS_EXT_RIGHT;
  ws_pos_left = WS_POS_EXT_LEFT;
  delay(1000); // time to move and stabilize

  /* **************************************************************
   * *********************** LOADCELLS: ***************************
   * **************************************************************/

  ledDelayBlink(2000, 5); // indicator loadcells are getting tared

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
    mb.Ists(SETUP_FAILURE_ISTS, true);
    while (true);
  }
  if (Loadcell_Left.getTareTimeoutFlag() || Loadcell_Left.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 left wiring and pin designations");
    mb.Ists(SETUP_FAILURE_ISTS, true);
    while (true);
  }
  Loadcell_Right.setCalFactor(LOADCELL_CAL_RIGHT);
  Loadcell_Left.setCalFactor(LOADCELL_CAL_LEFT);

  ledDelayBlink(1000, 5); // indicator loadcells are tared

  /* **************************************************************
   * ******************** PID Controller: *************************
   * **************************************************************/

  //turn the PID on
  Ws_PID.SetMode(AUTOMATIC);
  Ws_PID.SetSampleTime(1);

  /* **************************************************************
   * ************************ MODBUS: *****************************
   * **************************************************************/  

  mb.server();  // start modbus server

  // Discrete Input - bool - read only
  mb.addIsts(SETUP_FAILURE_ISTS);
  mb.addIsts(OS_ENDSWITCH_LEFT_ISTS);
  mb.addIsts(OS_ENDSWITCH_RIGHT_ISTS);

  // Holding Register - int - read and write
  mb.addHreg(MODE_HREG);
  mb.addHreg(FORCE_TARGET_HREG);
  mb.addHreg(POSITION_WS_HREG);
  mb.addHreg(P_HREG);
  mb.addHreg(I_HREG);
  mb.addHreg(D_HREG);
  mb.addHreg(MAX_INFLUENCE_CONTROLLER_HREG);

  // Input Register - int - read only
  mb.addIreg(FORCE_LEFT_REAL_IREG);
  mb.addIreg(FORCE_RIGHT_REAL_IREG);

  /* **************************************************************
   * ********************** SETUP DONE: ***************************
   * **************************************************************/

  ledDelayBlink(3000, 3); // setup routine step indicator
  Serial.println("Startup is complete");

}

/*
 * **********************************************************************************************************
 * **********************************************  LOOP  ****************************************************
 * **********************************************************************************************************
*/

/*
 *   mb.Ists(SETUP_FAILURE, setup_failure);
 *   mb.Hreg(MAX_INFLUENCE_CONTROLLER)
 */

void loop() {
  /*
     Main loop with communication with web-server, loadcell measurements and state machine.
  */
  getLoadcells();
  getEndswitches();

  mb.task();  // call once inside loop - all modbus magic here
  
  // state machine
  mode_ = mb.Hreg(MODE_HREG);
  if (mode_ == 0) {
    // do nothing rest state
  }
  else if (mode_ == 1) {
    openOS();
  }
  else if (mode_ == 2) {
    closeOS();
  }
  else if (mode_ == 3) {
    tareLoadcells();
  }
  else if (mode_ == 4) {
    //configuredWSInverseKin();
  }
  else if (mode_ == 4) {
    //configuredWSDirectDrive();
  }
  else if (mode_ == 5) {
    //adaptiveWS();
  }
  else if (mode_ == 6) {
    //semiAdaptiveWSInverseKin();
  }
  else if (mode_ == 7) {
    //semiAdaptiveWSDirectDrive();
  }
  else if (mode_ == 8) {
    demoWS();
  }
  else if (mode_ == 9) {
    debugViaSerial();
  }
  //catchLooptime();
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

  if ( os_pos_right != OS_POS_OPEN_RIGHT || os_pos_left != OS_POS_OPEN_LEFT) {
    Os_Right.setEaseToD(OS_POS_OPEN_RIGHT, 1000); // (postion, time to get there)
    Os_Left.setEaseToD(OS_POS_OPEN_LEFT, 1000); // (postion, time to get there)
    synchronizeAllServosStartAndWaitForAllServosToStop(); // moves in blocking mode

    os_pos_right = OS_POS_OPEN_RIGHT; // set position
    os_pos_left = OS_POS_OPEN_LEFT;   // set position
  }
}

void closeOS() {
  /*
     Tests necessary: What is the fastest speed the servos can follow?
  */

  if (os_pos_right != OS_POS_CLOSED_RIGHT || os_pos_left != OS_POS_CLOSED_LEFT) {
    Os_Right.setEaseToD(OS_POS_CLOSED_RIGHT, 1000); // (postion, time to get there)
    Os_Left.setEaseToD(OS_POS_CLOSED_LEFT, 1000); // (postion, time to get there)
    synchronizeAllServosStartAndWaitForAllServosToStop(); // moves in blocking mode

    os_pos_right = OS_POS_CLOSED_RIGHT; // set position, might unnecessary with new library
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
  
  mb.Hreg(MODE_HREG, 0);  // otherwise it keeps taring over and over till mode is changed on client side
}

void configuredWSInverseKin() {
  /*
     positioning of ws servos with inverse kinematics following desired ws_position
  */
  const static double bbb = 10;  // in mm axis to axis
  const static double ccc = 20;  // in mm
  double alpha = -acos(
               (2 * pow(bbb, 4) - pow(bbb, 2) * pow(ccc, 2) + pow(bbb, 2) * pow(position_ws, 2) - sqrt(
                  4 * pow(bbb, 4) * pow(ccc, 2) * pow(position_ws, 2) - pow(bbb, 2) * pow(ccc, 4) * pow(position_ws, 2) + 2 * pow(bbb, 2) * pow(ccc, 2) * pow(position_ws, 4) - pow(bbb, 2) * pow(position_ws, 6))
               ) / (2 * (pow(bbb, 4) + pow(bbb, 2) * pow(position_ws, 2)))
             );
  ws_pos_right = WS_POS_MID_LEFT + alpha;
  ws_pos_left = WS_POS_MID_LEFT - alpha;
  Serial.println(alpha);
  // move servos and check for endstops
  // time for servos to move
}

void adaptiveWS() {
  /*
     positioning of ws servos with pure pid control
  */
  Ws_PID.SetOutputLimits(0, (WS_MOTION_RANGE_2EXT + WS_MOTION_RANGE_2PULL));
  Ws_PID.SetTunings(Kp, Ki, Kd);
  Ws_PID.Compute();  //PID
  Ws_Left.write(WS_POS_EXT_RIGHT + pid_output_ws);
  Ws_Right.write(WS_POS_EXT_LEFT - pid_output_ws);
  // time for servos to move
}

void semiAdaptiveWSInverseKin() {
  /*
     positioning of ws servos with position_ws and error handling with pid
  */
  configuredWSInverseKin();
  static double controller_range = ((WS_MOTION_RANGE_2EXT + WS_MOTION_RANGE_2PULL) * max_influence_controller);
  Ws_PID.SetOutputLimits(-controller_range / 2, controller_range / 2);
  Ws_PID.SetTunings(Kp, Ki, Kd);
  Ws_PID.Compute();  //PID

  if ((ws_pos_right + pid_output_ws) >= WS_POS_EXT_RIGHT && (ws_pos_right + pid_output_ws) <= WS_POS_EXT_RIGHT + (WS_MOTION_RANGE_2EXT + WS_MOTION_RANGE_2PULL)) {   // if motion is in motion bounds
    Ws_Left.write(ws_pos_right + pid_output_ws);
    Ws_Right.write(ws_pos_left - pid_output_ws);
  }
  else if ((ws_pos_right + pid_output_ws) < WS_POS_EXT_RIGHT) {
    Ws_Left.write(WS_POS_EXT_LEFT);
    Ws_Right.write(WS_POS_EXT_RIGHT);
  }
  else {
    Ws_Left.write(WS_POS_EXT_LEFT - (WS_MOTION_RANGE_2EXT + WS_MOTION_RANGE_2PULL));  //ws_pull
    Ws_Right.write(WS_POS_EXT_RIGHT + (WS_MOTION_RANGE_2EXT + WS_MOTION_RANGE_2PULL));  //ws_pull
  }
}

void demoWS() {
  /*
     positioning of ws servos following a sine accelaration curve for demo
  */
  Ws_Right.setEasingType(EASE_SINE_IN_OUT); // position curve type
  Ws_Left.setEasingType(EASE_SINE_IN_OUT);  // position curve type

  Ws_Right.setEaseToD(WS_POS_EXT_RIGHT + (WS_MOTION_RANGE_2EXT + WS_MOTION_RANGE_2PULL), 1000);  // move to WS_POS_PULL_LEFT with total motion range so both servos move the same amount
  Ws_Left.setEaseToD(WS_POS_EXT_LEFT - (WS_MOTION_RANGE_2EXT + WS_MOTION_RANGE_2PULL), 1000);  // move to WS_POS_PULL_RIGHT motion range
  synchronizeAllServosStartAndWaitForAllServosToStop();
  Ws_Right.setEaseToD(WS_POS_EXT_RIGHT, 1000);
  Ws_Left.setEaseToD(WS_POS_EXT_LEFT, 1000);
  synchronizeAllServosStartAndWaitForAllServosToStop();
}

void debugViaSerial() {
  serialPrintSystemData();
  serialPrintReceivedData();  // disable in esp32 version
}

void serialPrintSystemData() {
  Serial.print(" Setup Fehlermeldung: ");
  Serial.print(mb.Ists(SETUP_FAILURE_ISTS));
  Serial.print(" Loadcell Left: ");
  Serial.print(loadcell_value_left);
  Serial.print(" Loadcell Right: ");
  Serial.print(loadcell_value_right);
  Serial.print(" Loadcell Mean: ");
  Serial.print(loadcell_value_mean);
  Serial.print(" Endswitch Left: ");
  Serial.print(mb.Ists(OS_ENDSWITCH_LEFT_ISTS));
  Serial.print(" Endswitch Right: ");
  Serial.print(mb.Ists(OS_ENDSWITCH_RIGHT_ISTS));
}

void serialPrintReceivedData() {
  Serial.print(" Mode: ");
  Serial.print(mode_);
  Serial.print(" Demanded Position Ws: ");
  Serial.print(loadcell_target);
  Serial.print(" Loadcell Target: ");
  Serial.print(position_ws);
  Serial.print(" Kp, Ki, Kd: ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print(Ki);
  Serial.print(", ");
  Serial.print(Kd);
  Serial.print(" Max Influence Controller: ");
  Serial.println(max_influence_controller);
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

  if ((newDataReady)) {
    loadcell_value_right = Loadcell_Right.getData();
    loadcell_value_left = Loadcell_Left.getData();
    loadcell_value_mean = (loadcell_value_right + loadcell_value_left) / 2;
    newDataReady = false;
  }
}

void getEndswitches() {
  mb.Ists(OS_ENDSWITCH_RIGHT_ISTS, !digitalRead(ENDSWITCH_RIGHT));    // due to utilized internal pullup resistors switch read would be always high and low on press, thats why we negate
  mb.Ists(OS_ENDSWITCH_LEFT_ISTS, !digitalRead(ENDSWITCH_LEFT));
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
