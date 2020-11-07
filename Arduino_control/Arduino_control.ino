/*******************************************************
 * This program is the main program for thebachelor thesis Kontibat mechatronic gripper.
 * Author: Florian Geissler
 */
 
// Libraries
#include <HX711_ADC.h>
#include <Servo.h>    

/*
 * **********************************************************************************************************
 * ********************************************  GLOBAL  ****************************************************
 * **********************************************************************************************************
 */

// Pi Value
#define PI 3.1415926535897932384626433832795        
              
// Timing Vars
int mister_t; //Iterator in Motorcontroll Loops KANN WEG ???
const int SERVO_TIMEOUT = 3; //Step in Motorcontroll Loops

// Program (Mode)
int Program = 1;
// Force-Sensor Pinout
const byte LOADCELL_RIGHT_DOUT_PIN = 3;
const byte LOADCELL_RIGHT_SCK_PIN = 4;
const byte LOADCELL_LEFT_DOUT_PIN = 12; 
const byte LOADCELL_LEFT_SCK_PIN = 11;

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
Servo OS_RIGHT;                      
Servo OS_LEFT;                      
Servo WS_RIGHT;                  
Servo WS_LEFT; 

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
 *  extended 0
 *  pull 1505
 *  
 *  Those are set as postions and a motion range is set (f.e. OS_MOTION_RANGE = 1700 - 1094). This way both servos have to move the same amount.
 */

// OS Servo Callibration
int OS_MOTION_RANGE = 605;                                                // same for both servos
int os_pos_right;                                                         // tracks is position
int OS_POS_OPEN_RIGHT = 1700;                                             // callibration postition
int OS_POS_CLOSED_RIGHT = OS_POS_OPEN_RIGHT-OS_MOTION_RANGE;              // callibration postition
int os_pos_left;                                                          // tracks is position
int OS_POS_OPEN_LEFT = 1020;                                              // callibration postition
int OS_POS_CLOSED_LEFT = OS_POS_OPEN_LEFT + OS_MOTION_RANGE;              // not in use, just as explaination


// WS Servo Callibration
int WS_MOTION_RANGE = 1060; //Werte des PWM Signal mit dem Sketch Servo_Callibration ermittelt
int ws_pos_right; 
int ws_pos_ext_right = 445;
int ws_pos_left; 
int ws_pos_ext_left = 2200;
int WS_STEP = 18; 

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
  // baudrate (communication speed) of serial port (:= 57,6 kbps)
  Serial.begin(57600);   
  delay(10);
  
  // attach buildin LED as status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // SERVOS:
  ledDelayBlink(1000, 1);
  // configuration of servos
  OS_RIGHT.attach(OS_RIGHT_PIN);
  OS_LEFT.attach(OS_LEFT_PIN);
  WS_RIGHT.attach(WS_RIGHT_PIN);
  WS_LEFT.attach(WS_LEFT_PIN);
  
  // move servos in starting position and save position
  OS_RIGHT.writeMicroseconds(OS_POS_OPEN_RIGHT);
  OS_LEFT.writeMicroseconds(OS_POS_OPEN_LEFT);
  WS_RIGHT.writeMicroseconds(ws_pos_ext_right);
  WS_LEFT.writeMicroseconds(ws_pos_ext_left);
  os_pos_right = OS_POS_OPEN_RIGHT;
  os_pos_left = OS_POS_OPEN_LEFT;
  ws_pos_right = ws_pos_ext_right;
  ws_pos_left = ws_pos_ext_left; 



  //LOADCELLS:
  ledDelayBlink(2000, 2);
  // calibration value for each loadcell - if loadcells delivers wrong measurements: recalibrate with program "Loadcell_calibration"
  float calibrationValue_right = 696.0;
  float calibrationValue_left = 733.0;
  
  // loadcell startup
  long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte loadcell_right_rdy = false;
  byte loadcell_left_rdy = false;
  LOADCELL_RIGHT.begin();
  LOADCELL_LEFT.begin();
  while (!(loadcell_left_rdy && loadcell_right_rdy)) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_right_rdy) loadcell_right_rdy = LOADCELL_RIGHT.startMultiple(stabilizingtime, _tare);
    if (!loadcell_left_rdy) loadcell_left_rdy = LOADCELL_LEFT.startMultiple(stabilizingtime, _tare);
  }
  if (LOADCELL_RIGHT.getTareTimeoutFlag()|| LOADCELL_RIGHT.getSignalTimeoutFlag()) {              // error log
    Serial.println("Timeout, check MCU>HX711 right wiring and pin designations");
    while(true);
  } 
  if (LOADCELL_LEFT.getTareTimeoutFlag()|| LOADCELL_LEFT.getSignalTimeoutFlag()) {                // error log
    Serial.println("Timeout, check MCU>HX711 left wiring and pin designations");
    while(true);
  }
  LOADCELL_RIGHT.setCalFactor(calibrationValue_right); // user set calibration value (float)
  LOADCELL_LEFT.setCalFactor(calibrationValue_left); // user set calibration value (float)

  // DONE
  Serial.println("Startup is complete");
  ledDelayBlink(5000, 10);
}
 
void loop() {
  /*
   * Main loop with communication with web-server, loadcell measurements and state machine. 
   */
  // communication with web-server
  recvOneChar();
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
  // read loacells
  measure();
}
   

// Communication with Web-Server
void recvOneChar() {
  char recievedChar;
  if (Serial.available() > 0) {
    recievedChar = Serial.read();
    Program = recievedChar;
  }
}

/*
 * **********************************************************************************************************
 * *************************************  STATE MACHINE FUNCTIONS  ******************************************
 * **********************************************************************************************************
 */
 
void openOS() {  
  // postioning of ws servos
  WS_RIGHT.writeMicroseconds(ws_pos_ext_right);
  WS_LEFT.writeMicroseconds(ws_pos_ext_left);
  
  // positioning of os servos following a sine accelaration curve
  if ( os_pos_right == OS_POS_CLOSED_RIGHT && os_pos_left == OS_POS_CLOSED_LEFT) {
    for (mister_t = 0; mister_t <= 81; mister_t = mister_t + SERVO_TIMEOUT){
        os_pos_right = OS_POS_CLOSED_RIGHT + (OS_MOTION_RANGE * sin((PI/162) * mister_t)); 
        os_pos_left = OS_POS_CLOSED_LEFT -(OS_MOTION_RANGE * sin((PI/162) * mister_t)); 
        OS_RIGHT.writeMicroseconds(os_pos_right);                                     
        OS_LEFT.writeMicroseconds(os_pos_left); 
        delay(SERVO_TIMEOUT);  
    }
  } 
}  

void closeOS() {
  // postioning of ws servos
  WS_RIGHT.writeMicroseconds(ws_pos_ext_right);
  WS_LEFT.writeMicroseconds(ws_pos_ext_left);

  // positioning of os servos following a sine accelaration curve
  if ( os_pos_right == OS_POS_OPEN_RIGHT && os_pos_left == OS_POS_OPEN_LEFT){
    for (mister_t = 0; mister_t <= 81; mister_t = mister_t + 3){
        os_pos_right = OS_POS_OPEN_RIGHT -(OS_MOTION_RANGE * sin((PI/162) * mister_t));
        os_pos_left = OS_POS_OPEN_LEFT + (OS_MOTION_RANGE * sin((PI/162) * mister_t));        
        OS_RIGHT.writeMicroseconds(os_pos_right);
        OS_LEFT.writeMicroseconds(os_pos_left);
        delay(SERVO_TIMEOUT);
    }
  }
}

void configuredWS() {
  // postioning of os servos
  OS_RIGHT.writeMicroseconds(OS_POS_CLOSED_RIGHT);
  OS_LEFT.writeMicroseconds(OS_POS_CLOSED_LEFT);

  // positioning of ws servos following a sine accelaration curve
  for (mister_t = 0; mister_t <= 144; mister_t = mister_t + 3){
      ws_pos_right = (int) (ws_pos_ext_right + (WS_MOTION_RANGE * sin((PI/144) * mister_t)));
      ws_pos_left = (int) (ws_pos_ext_left - (WS_MOTION_RANGE * sin((PI/144) * mister_t))); 
      WS_RIGHT.writeMicroseconds(ws_pos_right);      
      WS_LEFT.writeMicroseconds(ws_pos_left);
      delay(SERVO_TIMEOUT);
  }   
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

void measure() {
  long t;
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LOADCELL_RIGHT.update()) newDataReady = true;
  LOADCELL_LEFT.update();

  //get smoothed value from data set
  if ((newDataReady)) {
    if (millis() > t + serialPrintInterval) {
      float a = LOADCELL_RIGHT.getData();
      float b = LOADCELL_LEFT.getData();
      Serial.print(a);
      Serial.print("  ");
      Serial.println(b);
      newDataReady = 0;
      t = millis();
    }
  }

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
  */
}

void ledDelayBlink(int delay_time, int blink_times){
  for (int i = 0; i < blink_times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delay_time/(blink_times*2));
    digitalWrite(LED_BUILTIN, LOW);
    delay(delay_time/(blink_times*2));
  }         
}
