#include <HX711.h>    //Loadcell Verstärker Library
#include <Servo.h>    //Servo Library

#define PI 3.1415926535897932384626433832795

HX711 DMS_LEFT;       //Loadcell Links
HX711 DMS_RIGHT;      //Loadcell Rechts
Servo DS535_LEFT;     //OS-System Servo Links
Servo DS535_RIGHT;    //OS-System Servo Rechts
Servo AMX_0902_LEFT;  //WS-System Servo Links
Servo AMX_0902_RIGHT; //WS-System Servo Rechts

//Loadcell Pinout
const byte DMS_LEFT_DOUT_PIN = 3;
const byte DMS_LEFT_SCK_PIN = 4;
const byte DMS_RIGHT_DOUT_PIN = 12; 
const byte DMS_RIGHT_SCK_PIN = 11;

//Actuators Pinout
const byte DS535_LEFT_PIN = 6;
const byte DS535_RIGHT_PIN = 5;
const byte AMX_0902_LEFT_PIN = 9;
const byte AMX_0902_RIGHT_PIN = 10;

//Timing variables
int mister_t;
int period = 3;

//Variablen für gleitenden Durchschnitt der DMS Messwerte
long temp_dms_value_1_left = 0;
long temp_dms_value_2_left = 0;
long temp_dms_value_1_right = 0;
long temp_dms_value_2_right = 0;
long temp_dms_value_1;
long temp_dms_value_2;

//Programmauswahl
int Program = 1;
char recievedChar;

// ÖS-Antriebspositionierung
int Winkelbereich_OS = 630;
int pos_left;
int OS_Pos_offen_left = 1732;
int OS_Pos_geschlossen_left = OS_Pos_offen_left-Winkelbereich_OS;
int pos_right;
int OS_Pos_offen_right = 1550;
int OS_Pos_geschlossen_right = OS_Pos_offen_right + Winkelbereich_OS;


// WS-Antriebspositioinierung
int KSA_left; 
int KSA_left_null = 1300;
int KSA_right; 
int KSA_right_null = 1855;
int KSA_schritt = 18; 

//***VORBEREITENDES SETUP PROGRAMM*** 
void setup() {
  //Konfiguration der DMS Links und Rechts
  DMS_LEFT.begin(DMS_LEFT_DOUT_PIN, DMS_LEFT_SCK_PIN);
  DMS_LEFT.is_ready();
  DMS_RIGHT.begin(DMS_RIGHT_DOUT_PIN, DMS_RIGHT_SCK_PIN);
  DMS_RIGHT.is_ready();

  //Konfiguration der Antriebe
  DS535_LEFT.attach(DS535_LEFT_PIN);
  DS535_RIGHT.attach(DS535_RIGHT_PIN);
  AMX_0902_LEFT.attach(AMX_0902_LEFT_PIN);
  AMX_0902_RIGHT.attach(AMX_0902_RIGHT_PIN);

  //Positionieren der Antriebe
  DS535_LEFT.writeMicroseconds(OS_Pos_offen_left);
  DS535_RIGHT.writeMicroseconds(OS_Pos_offen_right);
  AMX_0902_LEFT.writeMicroseconds(KSA_left_null);
  AMX_0902_RIGHT.writeMicroseconds(KSA_right_null);

  //Speichern der ursprunglichen Positionierung
  KSA_left = KSA_left_null;
  KSA_right = KSA_right_null; 
  pos_left = OS_Pos_offen_left;
  pos_right = OS_Pos_offen_right;

  //Kalibrieirung von DMS ohne Last
  DMS_LEFT.set_scale();
  DMS_LEFT.tare();
  DMS_LEFT.get_units(10);
  DMS_LEFT.set_scale();
  DMS_RIGHT.set_scale();
  DMS_RIGHT.tare();
  DMS_RIGHT.get_units(10);
  DMS_RIGHT.set_scale();
  delay(500);

  //Messung der unbelasteten DMS-Werte und bilden ursprünglichen Wertes des gleitenden Durchsschnittes
  temp_dms_value_1_right = DMS_RIGHT.read_average(20);
  temp_dms_value_1_left = DMS_LEFT.read_average(20);
  temp_dms_value_1 = (temp_dms_value_1_right/1000 + temp_dms_value_1_left/100)/2;
  delay(500);

  //Baudrate des seriellen Ports (:= 57,6 kbps)
  Serial.begin(57600);      
}

//Hauptprogramm
void loop() {
    
  //Kommunikation mit dem Webserver
  recvOneChar();

  //Programm zum Öffnen
  if  (Program == 1) {
    openOS();
  } 
 
  //Programm zum Schließen
  if  (Program == 2) {
    closeOS();
  } 
  
  //Programm zur Kraftregelung der WS-Position
  if (Program == 3) {
    adaptiveWS();
  }
  
  //Programm zur WS-Positionierung nach Bahnkurve
  if (Program == 4) {
    configuredWS();
  }
}
   

//Funktionen zum Erhalten der Anweisungen vom Server
void recvOneChar() {
  if (Serial.available() > 0) {
    recievedChar = Serial.read();
    Program = recievedChar;
  }
}

//Programme

void openOS() {
  //Positionieren der WS-Antriebe
  AMX_0902_LEFT.writeMicroseconds(KSA_left_null);
  AMX_0902_RIGHT.writeMicroseconds(KSA_right_null);
  
  //Positionieren der ÖS-Antriebe nach Bahnkurve
  if ( pos_left == OS_Pos_geschlossen_left && pos_right == OS_Pos_geschlossen_right) {
    for (mister_t = 0; mister_t <= 81; mister_t = mister_t + period){
      pos_left = OS_Pos_geschlossen_left + (630 * sin((PI/162) * mister_t));
      pos_right = OS_Pos_geschlossen_right -(630 * sin((PI/162) * mister_t)); 
      DS535_LEFT.writeMicroseconds(pos_left);
      DS535_RIGHT.writeMicroseconds(pos_right);

      //Aktualisierungsrate der Antriebsposition
      delay(3);   
    } 
  }
}  

void closeOS() {
  //Positionieren der WS-Antriebe
  AMX_0902_LEFT.writeMicroseconds(KSA_left_null);
  AMX_0902_RIGHT.writeMicroseconds(KSA_right_null);

  //Positionieren der ÖS-Antriebe nach Bahnkurve
  if ( pos_left == OS_Pos_offen_left && pos_right == OS_Pos_offen_right){
    for (mister_t = 0; mister_t <= 81; mister_t = mister_t + 3){
      pos_left = OS_Pos_offen_left -(630 * sin((PI/162) * mister_t));
      pos_right = OS_Pos_offen_right + (630 * sin((PI/162) * mister_t));        
      DS535_LEFT.writeMicroseconds(pos_left);
      DS535_RIGHT.writeMicroseconds(pos_right);
      //Aktualisierungsrate der Antriebsposition
      delay(3);
    }
  }
}

void configuredWS() {
  //Positionieren der OS-Antriebe
  DS535_LEFT.writeMicroseconds(OS_Pos_geschlossen_left);
  DS535_RIGHT.writeMicroseconds(OS_Pos_geschlossen_right);

  //Positionieren der WS-Antriebe nach Bahnkurve
  for (mister_t = 0; mister_t <= 144; mister_t = mister_t + 3){
    KSA_left = (int) (KSA_left_null + (176.5 * sin((PI/144) * mister_t)));
    KSA_right = (int) (KSA_right_null - (176.5 * sin((PI/144) * mister_t))); 
    AMX_0902_LEFT.writeMicroseconds(KSA_left);      
    AMX_0902_RIGHT.writeMicroseconds(KSA_right);
    //Aktualisierungsrate der Antriebsposition
    delay(3);
  }   
}

void adaptiveWS() {
  //Positionierung der ÖS-Motoren
  DS535_LEFT.writeMicroseconds(OS_Pos_geschlossen_left);
  DS535_RIGHT.writeMicroseconds(OS_Pos_geschlossen_right); 
  
  //Einlesen der DMS-Werte
  temp_dms_value_2_right = DMS_RIGHT.read(); 
  temp_dms_value_2_left = DMS_LEFT.read();

  //Berechnen des gleitenden Durchnschnitts (Mit Offset)
  temp_dms_value_2 = (temp_dms_value_2_right/1000 + temp_dms_value_2_left/100)/2;

  //Aktueller Durchnitt größer als der davor gemessene
  if (temp_dms_value_2 > temp_dms_value_1) { 
    //Positionierung der ÖS-Motoren WARUM?
    //DS535_LEFT.writeMicroseconds(OS_Pos_geschlossen_left);
    //DS535_RIGHT.writeMicroseconds(OS_Pos_geschlossen_right);
    
    //Anpassen der WS-Position
    KSA_left = KSA_left + KSA_schritt;
    KSA_right = KSA_right - KSA_schritt;     
    //Positionierung der WS-Antriebe
    AMX_0902_LEFT.writeMicroseconds(KSA_left);
    AMX_0902_RIGHT.writeMicroseconds(KSA_right);

    //Überschreiben des gleitenden Durchschnitts
    temp_dms_value_1 = temp_dms_value_2;

    //Wartezeit, bis der nächste DMS-Wert verfügbar wird    
    delay(100);
  }

  //Aktueller Durchnitt kleiner als der davor gemessene
  if (temp_dms_value_2 < temp_dms_value_1) { 

    //Positionierung der ÖS-Motoren
    DS535_LEFT.writeMicroseconds(OS_Pos_geschlossen_left);
    DS535_RIGHT.writeMicroseconds(OS_Pos_geschlossen_right);
      
    //Anpassen der WS-Position
    KSA_left = KSA_left - KSA_schritt;
    KSA_right = KSA_right + KSA_schritt;

    //Positionierung der WS-Antriebe
    AMX_0902_LEFT.writeMicroseconds(KSA_left);
    AMX_0902_RIGHT.writeMicroseconds(KSA_right);
     
    //Überschreiben des gleitenden Durchschnitts
    temp_dms_value_1 = temp_dms_value_2;

    //Wartezeit, bis der nächste DMS-Wert verfügbar wird WARUM
    delay(100);
  }
  //Aktueller Durchnitt gleich dem davor gemessenen
  if (temp_dms_value_2 == temp_dms_value_1) { 

    //Positionierung der ÖS-Motoren
    DS535_LEFT.writeMicroseconds(OS_Pos_geschlossen_left);
    DS535_RIGHT.writeMicroseconds(OS_Pos_geschlossen_right);

    //Erhalten der WS-Position
    KSA_left = KSA_left;
    KSA_right = KSA_right;     

    //Positionierung der WS-Antriebe
    AMX_0902_LEFT.writeMicroseconds(KSA_left);
    AMX_0902_RIGHT.writeMicroseconds(KSA_right);

    //Überschreiben des gleitenden Durchschnitts
    temp_dms_value_1 = temp_dms_value_2;

    //Wartezeit, bis der nächste DMS-Wert verfügbar wird    
    delay(100);
  }
}
