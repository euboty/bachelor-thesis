/************************************************************************************************************
 * This program is the main program for thebachelor thesis Kontibat mechatronic gripper.
 * Author: Florian Geissler
 */
 
// Libraries
#include <HX711_ADC.h>
#include <ServoEasing.h>   

/*
 * **********************************************************************************************************
 * ********************************************  GLOBAL  ****************************************************
 * **********************************************************************************************************
 */

// Pi Value
#define PI 3.1415926535897932384626433832795        
              
// Timing Vars
unsigned long prevMillis;

// Program (Mode)
int Program = 1;

// Force-Sensor Pinout
const byte LOADCELL_RIGHT_DOUT_PIN = 3;
const byte LOADCELL_RIGHT_SCK_PIN = 4;
const byte LOADCELL_LEFT_DOUT_PIN = 11; 
const byte LOADCELL_LEFT_SCK_PIN = 12;

// Loadcell HX711 Constructor (dout pin, sck pin)
HX711_ADC LOADCELL_RIGHT(LOADCELL_RIGHT_DOUT_PIN, LOADCELL_RIGHT_SCK_PIN);
HX711_ADC LOADCELL_LEFT(LOADCELL_LEFT_DOUT_PIN, LOADCELL_LEFT_SCK_PIN); 

// Loadcell Vars
long loadcell_prev_value_right = 0;
long loadcell_value_right = 0;
long loadcell_prev_value_left = 0;
long loadcell_value_left = 0;
long loadcell_prev_value_avg;
long loadcell_value_avg;

// Servo Pinout
const byte OS_RIGHT_PIN = 5;
const byte OS_LEFT_PIN = 6;
const byte WS_RIGHT_PIN = 9;
const byte WS_LEFT_PIN = 10;

// Servos
ServoEasing OS_RIGHT;                      
ServoEasing OS_LEFT;                      
ServoEasing WS_RIGHT;                  
ServoEasing WS_LEFT; 

/*
 * Callibration:
 * Following values describe the opening and closing positions of the servos (PWM signal).
 * The sketch "Servo_callibration" was used to figure those values.
 * Perspective is mover to foil:
 * OS_LEFT (OS Left)
 *  open 1020
 *  closed 1620
 * OS_RIGHT (OS Right)
 *  open 1700
 *  closed 1094
 * WS_LEFT (WS Left)
 *  extended 2260
 *  pull 1200
 * WS_RIGHT (WS Right)
 *  extended 985
 *  pull 2104
 *  
 *  Those are set as postions and a motion range is set (f.e. OS_MOTION_RANGE = 1700 - 1094). This way both servos have to move the same amount.
 */

// OS Servo Callibration
int os_pos_right;                                                         // tracks is position, not necessary
const int OS_POS_OPEN_RIGHT = 1700;                                             
const int OS_POS_CLOSED_RIGHT = 1094;                                         
int os_pos_left;                                                          // tracks is position, not necessary
const int OS_POS_OPEN_LEFT = 1020;                                           
const int OS_POS_CLOSED_LEFT = 1620;                                            

// WS Servo Callibration
int ws_pos_right;
const int WS_POS_EXT_RIGHT = 985;
const int WS_POS_PULL_RIGHT = 2104; 
int ws_pos_left;
const int WS_POS_EXT_LEFT = 2260;
const int WS_POS_PULL_LEFT = 1200; 

/*
 * **********************************************************************************************************
 * ********************************************  FUNCTIONS  *************************************************
 * **********************************************************************************************************
 */

void setup() {
  /*
   * Setup Program - just runs once on startup. Initializes servos and loadcells and sets start vars.
   * Status is displayed through buildin LED.
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
   * ************************* SERVOS: ****************************
   * **************************************************************/
   
  ledDelayBlink(1000, 1); // setup routine step indicator
  
  // configuration of servos
  if (OS_RIGHT.attach(OS_RIGHT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo OS_RIGHT"));
  }
  if (OS_LEFT.attach(OS_LEFT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo OS_RIGHT"));
  }
  if (WS_RIGHT.attach(WS_RIGHT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo WS_RIGHT"));
  }
  if (WS_LEFT.attach(WS_LEFT_PIN) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo WS_LEFT"));
  }

  OS_RIGHT.setEasingType(EASE_CUBIC_OUT); // position curve type
  OS_LEFT.setEasingType(EASE_CUBIC_OUT);  // position curve type
  
  // move servos to starting position and save position
  OS_RIGHT.write(OS_POS_OPEN_RIGHT);
  OS_LEFT.write(OS_POS_OPEN_LEFT);
  WS_RIGHT.write(WS_POS_EXT_RIGHT);
  WS_LEFT.write(WS_POS_EXT_LEFT);
  
  os_pos_right = OS_POS_OPEN_RIGHT;
  os_pos_left = OS_POS_OPEN_LEFT;
  ws_pos_right = WS_POS_EXT_RIGHT;
  ws_pos_left = WS_POS_EXT_LEFT;
  delay(500); // time to move and stabilize

  /* **************************************************************
   * *********************** LOADCELLS: ***************************
   * **************************************************************/
   
  ledDelayBlink(2000, 2); // setup routine step indicator

  // loadcell setup vars
  // calibration value for each loadcell - if loadcells delivers wrong measurements: recalibrate with dedicated program "Loadcell_calibration"
  float calibrationValue_right = 1985.14;
  float calibrationValue_left = 1948.98;
  long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; // set this to false if you don't want tare to be performed in the next step
    
  // loadcell startup
  byte loadcell_right_rdy = false;
  byte loadcell_left_rdy = false;
  LOADCELL_RIGHT.begin();
  LOADCELL_LEFT.begin();
  // run startup, stabilization and tare, both modules simultaniously
  while (!(loadcell_left_rdy && loadcell_right_rdy)) {
    if (!loadcell_right_rdy) loadcell_right_rdy = LOADCELL_RIGHT.startMultiple(stabilizingtime, _tare);
    if (!loadcell_left_rdy) loadcell_left_rdy = LOADCELL_LEFT.startMultiple(stabilizingtime, _tare);
  }
  if (LOADCELL_RIGHT.getTareTimeoutFlag()|| LOADCELL_RIGHT.getSignalTimeoutFlag()) {             
    Serial.println("Timeout, check MCU>HX711 right wiring and pin designations");
    while(true);
  } 
  if (LOADCELL_LEFT.getTareTimeoutFlag()|| LOADCELL_LEFT.getSignalTimeoutFlag()) {               
    Serial.println("Timeout, check MCU>HX711 left wiring and pin designations");
    while(true);
  }
  LOADCELL_RIGHT.setCalFactor(calibrationValue_right);
  LOADCELL_LEFT.setCalFactor(calibrationValue_left);

  /* **************************************************************
   * ********************** SETUP DONE: ***************************
   * **************************************************************/
   
  ledDelayBlink(3000, 3); // setup routine step indicator
  Serial.println("Startup is complete");

}
 
void loop() {
  /*
   * Main loop with communication with web-server, loadcell measurements and state machine. 
   */
   
  recvOneChar();  // communication with web-server

  // state machine
  if  (Program == 1) {
    openOS();
  } 
 
  if  (Program == 2) {
    closeOS();
  } 
  
  else if (Program == 3) {
    //adaptiveWS();
  }
  
  else if (Program == 4) {
    configuredWS();
  }

  /*
  if (millis()-prevMillis > 3000){
    openOS();
  }
  if (millis()-prevMillis > 6000){
    closeOS();
    prevMillis = millis();
  }
  */

  measure();   // read loacells
}
   

// Communication with Web-Server
void recvOneChar() {
  char recievedChar;
  if (Serial.available() > 0) {
    recievedChar = Serial.read();
    Program = recievedChar;
  }
}

void ledDelayBlink(int delay_time, int blink_times){
  for (int i = 0; i < blink_times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delay_time/(blink_times*2));
    digitalWrite(LED_BUILTIN, LOW);
    delay(delay_time/(blink_times*2));
  }         
}

void measure() {
  static boolean newDataReady = 0;

  // check for new data/start next conversion:
  if (LOADCELL_RIGHT.update()) newDataReady = true;
  LOADCELL_LEFT.update();

  //get smoothed value from data set
  if ((newDataReady)) {
    float a = LOADCELL_RIGHT.getData();
    float b = LOADCELL_LEFT.getData();
    Serial.print(a);
    Serial.print("  ");
    Serial.println(b);
    newDataReady = 0;
  }
}

/*
 * **********************************************************************************************************
 * *************************************  STATE MACHINE FUNCTIONS  ******************************************
 * **********************************************************************************************************
 */
 
void openOS() {  
  /*
   * Tests necessary: What is the fastest speed the servos can follow?
   */

  if ( os_pos_right != OS_POS_OPEN_RIGHT || os_pos_left != OS_POS_OPEN_LEFT) { 
      OS_RIGHT.setEaseToD(OS_POS_OPEN_RIGHT, 1000); // (postion, time to get there) 
      OS_LEFT.setEaseToD(OS_POS_OPEN_LEFT, 1000); // (postion, time to get there) 
      synchronizeAllServosStartAndWaitForAllServosToStop(); // moves in blocking mode
      
      os_pos_right = OS_POS_OPEN_RIGHT; // set position, might unnecessary with new library
      os_pos_left = OS_POS_OPEN_LEFT;
  }
}   

void closeOS() {
  /*
   * Tests necessary: What is the fastest speed the servos can follow?
   */

  if ( os_pos_right != OS_POS_CLOSED_RIGHT || os_pos_left != OS_POS_CLOSED_LEFT) { 
      OS_RIGHT.setEaseToD(OS_POS_CLOSED_RIGHT, 1000); // (postion, time to get there) 
      OS_LEFT.setEaseToD(OS_POS_CLOSED_LEFT, 1000); // (postion, time to get there) 
      synchronizeAllServosStartAndWaitForAllServosToStop(); // moves in blocking mode
      
      os_pos_right = OS_POS_CLOSED_RIGHT; // set position, might unnecessary with new library
      os_pos_left = OS_POS_CLOSED_LEFT;
  }
}

void configuredWS() {
  
  // postioning of os servos
  closeOS();
  
  // positioning of ws servos following a sine accelaration curve
      WS_RIGHT.setEasingType(EASE_SINE_IN_OUT); // position curve type
      WS_LEFT.setEasingType(EASE_SINE_IN_OUT);  // position curve type
      
      WS_RIGHT.setEaseToD(WS_POS_PULL_RIGHT,1000);      
      WS_LEFT.setEaseToD(WS_POS_PULL_LEFT,1000);
      synchronizeAllServosStartAndWaitForAllServosToStop();
      WS_RIGHT.setEaseToD(WS_POS_EXT_RIGHT,1000);      
      WS_LEFT.setEaseToD(WS_POS_EXT_LEFT,1000);
      synchronizeAllServosStartAndWaitForAllServosToStop();   
}

 /*
void adaptiveWS() {
 
  // postioning of os servos
  OS_RIGHT.writeMicroseconds(OS_POS_CLOSED_RIGHT);
  OS_LEFT.writeMicroseconds(OS_POS_CLOSED_LEFT); 
  
  // read loadcell values
  loadcell_value_left = LOADCELL_LEFT.read(); 
  loadcell_value_right = LOADCELL_RIGHT.read();

  //Berechnen des gleitenden Durchnschnitts (Mit Offset)
  loadcell_value_avg = (loadcell_value_left/1000 + loadcell_value_right/100)/2;

  //Aktueller Durchnitt größer als der davor gemessene
  if (loadcell_value_avg > loadcell_prev_value_avg) { 
    //Positionierung der ÖS-Motoren WARUM?
    //OS_RIGHT.writeMicroseconds(OS_POS_CLOSED_RIGHT);
    //OS_LEFT.writeMicroseconds(OS_POS_CLOSED_LEFT);
    
    //Anpassen der WS-Position
    ws_pos_right = ws_pos_right + WS_STEP;
    ws_pos_left = ws_pos_left - WS_STEP;     
    //Positionierung der WS-Antriebe
    WS_RIGHT.writeMicroseconds(ws_pos_right);
    WS_LEFT.writeMicroseconds(ws_pos_left);

    //Überschreiben des gleitenden Durchschnitts
    loadcell_prev_value_avg = loadcell_value_avg;

    //Wartezeit, bis der nächste LOADCELL-Wert verfügbar wird    
    delay(100);
  }

  //Aktueller Durchnitt kleiner als der davor gemessene
  else if (loadcell_value_avg < loadcell_prev_value_avg) { 

    //Positionierung der ÖS-Motoren
    OS_RIGHT.writeMicroseconds(OS_POS_CLOSED_RIGHT);
    OS_LEFT.writeMicroseconds(OS_POS_CLOSED_LEFT);
      
    //Anpassen der WS-Position
    ws_pos_right = ws_pos_right - WS_STEP;
    ws_pos_left = ws_pos_left + WS_STEP;

    //Positionierung der WS-Antriebe
    WS_RIGHT.writeMicroseconds(ws_pos_right);
    WS_LEFT.writeMicroseconds(ws_pos_left);
     
    //Überschreiben des gleitenden Durchschnitts
    loadcell_prev_value_avg = loadcell_value_avg;

    //Wartezeit, bis der nächste LOADCELL-Wert verfügbar wird WARUM
    delay(100);
  }
  
  //Aktueller Durchnitt gleich dem davor gemessenen
  else if (loadcell_value_avg == loadcell_prev_value_avg) { 

    //Positionierung der ÖS-Motoren
    OS_RIGHT.writeMicroseconds(OS_POS_CLOSED_RIGHT);
    OS_LEFT.writeMicroseconds(OS_POS_CLOSED_LEFT);

    //Erhalten der WS-Position
    ws_pos_right = ws_pos_right;
    ws_pos_left = ws_pos_left;     

    //Positionierung der WS-Antriebe
    WS_RIGHT.writeMicroseconds(ws_pos_right);
    WS_LEFT.writeMicroseconds(ws_pos_left);

    //Überschreiben des gleitenden Durchschnitts
    loadcell_prev_value_avg = loadcell_value_avg;

    //Wartezeit, bis der nächste LOADCELL-Wert verfügbar wird    
    delay(100);
    
  }
}
*/


  /*
  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 't') {
      LOADCELL_RIGHT.tareNoDelay();
      LOADCELL_LEFT.tareNoDelay();
    }
  }
  //check if last tare operation is complete
  if (LOADCELL_RIGHT.getTareStatus() == true) {
    Serial.println("Tare loadcell 1 complete");
  }
  if (LOADCELL_LEFT.getTareStatus() == true) {
    Serial.println("Tare loadcell 2 complete");
  }

}
  */
