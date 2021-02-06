/************************************************************************************************************
   This program is the main program for the bachelor thesis Kontibat mechatronic gripper of Florian Geissler.
   Version: Pre Final Http-Serial Version.
   Author: Florian Geissler
*/

/*
 * **********************************************************************************************************
 * ********************************************  LIBRARIES **************************************************
 * **********************************************************************************************************
*/

#include <HX711_ADC.h>    //https://github.com/olkal/HX711_ADC
#include <ServoEasing.h>    //https://github.com/ArminJo/ServoEasing
#include <PID_v1.h>     //https://github.com/br3ttb/Arduino-PID-Library
#include <Wire.h>

/*
 * **********************************************************************************************************
 * *********************************************  GLOBAL ****************************************************
 * **********************************************************************************************************
*/

// Program Mode
byte mode_ = 0;

// Failure Alert
bool setup_failure = 0;

// Endswitch Pinout
const byte ENDSWITCH_RIGHT = 7;
const byte ENDSWITCH_LEFT = 8;

// Endswitch Vars
bool endswitch_right;
bool endswitch_left;

// Loadcell Pinout
const byte LOADCELL_RIGHT_DOUT_PIN = 3;
const byte LOADCELL_RIGHT_SCK_PIN = 4;
const byte LOADCELL_LEFT_DOUT_PIN = 11;
const byte LOADCELL_LEFT_SCK_PIN = 12;

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
const byte OS_RIGHT_PIN = 5;
const byte OS_LEFT_PIN = 6;
const byte WS_RIGHT_PIN = 9;
const byte WS_LEFT_PIN = 10;

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
   Os_Left (OS Left)
    open 1020
    closed 1620
   Os_Right (OS Right)
    open 1700
    closed 1094
   Ws_Left (WS Left)
    extended 2260
    pull 1200
   Ws_Right (WS Right)
    extended 985
    pull 2104

    Those are set as postions and a motion range is set (f.e. OS_MOTION_RANGE = 1700 - 1094). This way both servos have to move the same amount.
*/

// OS Servo Callibration
int os_pos_right;                                                         // tracks is position in software
const int OS_POS_OPEN_RIGHT = 1700;
const int OS_POS_CLOSED_RIGHT = 1094;
int os_pos_left;                                                          // tracks is position in software
const int OS_POS_OPEN_LEFT = 1020;
const int OS_POS_CLOSED_LEFT = 1620;

// WS Servo Callibration
int ws_pos_right;
const int WS_POS_EXT_RIGHT = 985;
//const int WS_POS_PULL_RIGHT = 2104;
int ws_pos_left;
const int WS_POS_EXT_LEFT = 2260;
//const int WS_POS_PULL_LEFT = 1220;
const int WS_MOTION_RANGE = 1040;
unsigned int position_ws;   // demanded ws position given by PLC

// PID
double loadcell_target, pid_output_ws;
double max_influence_controller;  // in percent of ws motion range for semi adaptive program
double Kp = 3, Ki = 22, Kd = 2; // those parameters are just examples and not tuned yet
PID Ws_PID(&loadcell_value_mean, &pid_output_ws, &loadcell_target, Kp, Ki, Kd, DIRECT); // Input, Output, Setpoint, Kp, Ki, Kd

//I2C
byte payload [2];

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

  ledDelayBlink(1000, 1); // setup routine step indicator

  // configuration of servos
  if (Os_Right.attach(OS_RIGHT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Os_Right"));
    setup_failure = true;
  }
  if (Os_Left.attach(OS_LEFT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Os_Right"));
    setup_failure = true;
  }
  if (Ws_Right.attach(WS_RIGHT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Ws_Right"));
    setup_failure = true;
  }
  if (Ws_Left.attach(WS_LEFT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Ws_Left"));
    setup_failure = true;
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

  ledDelayBlink(2000, 2); // setup routine step indicator

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
    while (true);
  }
  if (Loadcell_Left.getTareTimeoutFlag() || Loadcell_Left.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 left wiring and pin designations");
    setup_failure = true;
    while (true);
  }
  Loadcell_Right.setCalFactor(LOADCELL_CAL_RIGHT);
  Loadcell_Left.setCalFactor(LOADCELL_CAL_LEFT);

  /* **************************************************************
   * ******************** PID Controller: *************************
   * **************************************************************/

  //turn the PID on
  Ws_PID.SetMode(AUTOMATIC);
  Ws_PID.SetOutputLimits(0, WS_MOTION_RANGE);

  /* **************************************************************
   * ******************** I2C Communication: **********************
   * **************************************************************/

  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event (interrupt)
  Wire.onRequest(requestEvent); // register event (interrupt)

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

void loop() {
  /*
     Main loop with communication with web-server, loadcell measurements and state machine.
  */
  getLoadcells();
  getEndswitches();
  serialPrintSystemData();
  serialPrintReceivedData();

  // state machine
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
    semiAdaptiveWS();
  }
  else if (mode_ == 4) {
    adaptiveWS();
  }
  else if (mode_ == 5) {
    demoWS();
  }
  else if (mode_ == 6) {
    tare_();
  }

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
  if (Loadcell_Right.update()) newDataReady = true;
  Loadcell_Left.update();

  //get smoothed? value from data set
  if ((newDataReady)) {
    loadcell_value_right = Loadcell_Right.getData();
    loadcell_value_left = Loadcell_Left.getData();
    loadcell_value_mean = (loadcell_value_right + loadcell_value_left) / 2;
    /*
      Serial.print(loadcell_value_right);
      Serial.print("  ");
      Serial.println(loadcell_value_left);
    */
    newDataReady = false;
  }
}

void getEndswitches() {
  endswitch_right = !digitalRead(ENDSWITCH_RIGHT);    // due to utilized internal pullup resistors switch read would be always high and low on press, thats why we negate
  endswitch_left = !digitalRead(ENDSWITCH_LEFT);
}

void serialPrintSystemData() {
  Serial.print(" Setup Fehlermeldung: ");
  Serial.print(setup_failure);
  Serial.print(" Loadcell Left: ");
  Serial.print(loadcell_value_left);
  Serial.print(" Loadcell Right: ");
  Serial.print(loadcell_value_right);
  Serial.print(" Loadcell Mean: ");
  Serial.print(loadcell_value_mean);
  Serial.print(" Endswitch Left: ");
  Serial.print(endswitch_left);
  Serial.print(" Endswitch Right: ");
  Serial.print(endswitch_right);
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
 * ******************************************  I2C Connection ***********************************************
 * **********************************************************************************************************
*/

void receiveEvent() {
  /*
     Function that executes whenever data is received from I2C master.
     This function is registered as an event, see setup()
  */
  mode_ = Wire.read ();

  byte buffer_[2];
  buffer_[0] = Wire.read ();
  buffer_[1] = Wire.read ();
  short force_target_i2c = ((short)(buffer_[0]) << 8) + buffer_[1];
  loadcell_target = (double)force_target_i2c / 9.81;  // Millinewton/ 9.81 = gramms

  buffer_[0] = Wire.read ();
  buffer_[1] = Wire.read ();
  position_ws = ((short)(buffer_[0]) << 8) + buffer_[1];

  buffer_[0] = Wire.read ();
  buffer_[1] = Wire.read ();
  short Kp_i2c = ((short)(buffer_[0]) << 8) + buffer_[1];
  Kp = (double)Kp_i2c / 1000; // modbus and i2c sends ints we want floats, thats why we send int * 1000 in the first place

  buffer_[0] = Wire.read ();
  buffer_[1] = Wire.read ();
  short Ki_i2c = ((short)(buffer_[0]) << 8) + buffer_[1];
  Ki = (double)Ki_i2c / 1000;

  buffer_[0] = Wire.read ();
  buffer_[1] = Wire.read ();
  short Kd_i2c = ((short)(buffer_[0]) << 8) + buffer_[1];
  Kd = (double)Kd_i2c / 1000;

  buffer_[0] = Wire.read ();
  buffer_[1] = Wire.read ();
  short max_influence_controller_i2c = ((short)(buffer_[0]) << 8) + buffer_[1];
  max_influence_controller = max_influence_controller_i2c / 10; // controller influence in promille
}

void requestEvent() {
  /*
    Function that executes whenever data is requested from I2C master.
    This function is registered as an event, see setup()
  */
  Wire.write(setup_failure);
  Wire.write(endswitch_left);
  Wire.write(endswitch_right); //this 3 values could be combined to one byte for further perfomance improvements

  int force_left_real = int(loadcell_value_left * 10); // we wanna send e.g. 300.1g --> we make it 3001 and recast it to 300.1 in Twincat
  int2bytes(force_left_real);
  Wire.write (payload, 2);
  int force_right_real = int(loadcell_value_right * 10); // we wanna send e.g. 300.1g --> we make it 3001 and recast it to 300.1 in Twincat
  int2bytes(force_right_real);
  Wire.write (payload, 2);
}

void int2bytes(int split_me) {
  /*
     For I2C Communication: Splits int into an array of bytes which is defined out of this function,
     since C++ does not support return of arrays.
  */
  payload[0] = highByte(split_me);
  payload[1] = lowByte(split_me);
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

void demoWS() {
  closeOS();
  // positioning of ws servos following a sine accelaration curve for testing
  Ws_Right.setEasingType(EASE_SINE_IN_OUT); // position curve type
  Ws_Left.setEasingType(EASE_SINE_IN_OUT);  // position curve type

  Ws_Right.setEaseToD(WS_POS_EXT_RIGHT + WS_MOTION_RANGE, 1000);  // move to WS_POS_PULL_LEFT with motion range
  Ws_Left.setEaseToD(WS_POS_EXT_LEFT - WS_MOTION_RANGE, 1000);  // move to WS_POS_PULL_RIGHT motion range
  synchronizeAllServosStartAndWaitForAllServosToStop();
  Ws_Right.setEaseToD(WS_POS_EXT_RIGHT, 1000);
  Ws_Left.setEaseToD(WS_POS_EXT_LEFT, 1000);
  synchronizeAllServosStartAndWaitForAllServosToStop();
}

void adaptiveWS() {
  closeOS();
  Ws_PID.Compute();  //PID
  Ws_Left.writeMicroseconds(WS_POS_EXT_RIGHT + pid_output_ws);
  Ws_Right.writeMicroseconds(WS_POS_EXT_LEFT - pid_output_ws);
}

void semiAdaptiveWS() {
  // in progress
}

void tare_() {
  /*
     Tares both loadcells.
  */

  Loadcell_Right.tareNoDelay();
  Loadcell_Left.tareNoDelay();

  //check if last tare operation is complete
  while (Loadcell_Right.getTareStatus() == false) {
    Serial.println("Taring...");
  }
  while (Loadcell_Left.getTareStatus() == false) {
    Serial.println("Taring...");
  }
}
