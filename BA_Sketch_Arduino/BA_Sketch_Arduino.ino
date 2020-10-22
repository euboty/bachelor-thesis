#include <HX711.h>    //Loadcell Verstärker Library
#include <Servo.h>    //Servo Library

#define PI 3.1415926535897932384626433832795

HX711 DMS_left;       //Loadcell Links
HX711 DMS_right;      //Loadcell Rechts
Servo DS535_left;     //OS-System Servo Links
Servo DS535_right;    //OS-System Servo Rechts
Servo AMX_0902_left;  //WS-System Servo Links
Servo AMX_0902_right; //WS-System Servo Rechts

//DMS Pinout
const int DMS_left_DOUT_PIN = 3;
const int DMS_left_SCK_PIN = 4;
const int DMS_right_DOUT_PIN = 12; 
const int DMS_right_SCK_PIN = 11;

//Antriebe Pinout
const int DS535_left_PIN = 6;
const int DS535_right_PIN = 5;
const int AMX_0902_left_PIN = 9;
const int AMX_0902_right_PIN = 10;

//Variablen für Zeitüberwachung
int mister_t = 0;
unsigned long time_now = 0;
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
  DMS_left.begin(DMS_left_DOUT_PIN, DMS_left_SCK_PIN);
  DMS_left.is_ready();
  DMS_right.begin(DMS_right_DOUT_PIN, DMS_right_SCK_PIN);
  DMS_right.is_ready();

  //Konfiguration der Antriebe
  DS535_left.attach(DS535_left_PIN);
  DS535_right.attach(DS535_right_PIN);
  AMX_0902_left.attach(AMX_0902_left_PIN);
  AMX_0902_right.attach(AMX_0902_right_PIN);

  //Positionieren der Antriebe
  DS535_left.writeMicroseconds(OS_Pos_offen_left);
  DS535_right.writeMicroseconds(OS_Pos_offen_right);
  AMX_0902_left.writeMicroseconds(KSA_left_null);
  AMX_0902_right.writeMicroseconds(KSA_right_null);

  //Speichern der ursprunglichen Positionierung
  KSA_left = KSA_left_null;
  KSA_right = KSA_right_null; 
  pos_left = OS_Pos_offen_left;
  pos_right = OS_Pos_offen_right;

  //Kalibrieirung von DMS ohne Last
  DMS_left.set_scale();
  DMS_left.tare();
  DMS_left.get_units(10);
  DMS_left.set_scale();
  DMS_right.set_scale();
  DMS_right.tare();
  DMS_right.get_units(10);
  DMS_right.set_scale();
  delay(500);

  //Messung der unbelasteten DMS-Werte und bilden ursprünglichen Wertes des gleitenden Durchsschnittes
  temp_dms_value_1_right = DMS_right.read_average(20);
  temp_dms_value_1_left = DMS_left.read_average(20);
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
  
//Funktionen zum Warten
void wait3() {
  time_now = millis();
  while(millis() < time_now + period) {
  } 
}

void wait4() {
  time_now = millis();
  while(millis() < time_now + 4){
  }
}

void wait100() {
  time_now = millis();
  while(millis() < time_now + 100){
  }
}

//Programme

void openOS() {
  //Positionieren der WS-Antriebe
  AMX_0902_left.writeMicroseconds(KSA_left_null);
  AMX_0902_right.writeMicroseconds(KSA_right_null);
  
  //Positionieren der ÖS-Antriebe nach Bahnkurve
  if ( pos_left == OS_Pos_geschlossen_left && pos_right == OS_Pos_geschlossen_right) {
    for (mister_t = 0; mister_t <= 81; mister_t = mister_t + period){
      pos_left = OS_Pos_geschlossen_left + (630 * sin((PI/162) * mister_t));
      pos_right = OS_Pos_geschlossen_right -(630 * sin((PI/162) * mister_t)); 
      DS535_left.writeMicroseconds(pos_left);
      DS535_right.writeMicroseconds(pos_right);

      //Aktualisierungsrate der Antriebsposition
      wait3();   
    } 
  }
}  

void closeOS() {
  //Positionieren der WS-Antriebe
  AMX_0902_left.writeMicroseconds(KSA_left_null);
  AMX_0902_right.writeMicroseconds(KSA_right_null);

  //Positionieren der ÖS-Antriebe nach Bahnkurve
  if ( pos_left == OS_Pos_offen_left && pos_right == OS_Pos_offen_right){
    for (mister_t = 0; mister_t <= 81; mister_t = mister_t + 3){
      pos_left = OS_Pos_offen_left -(630 * sin((PI/162) * mister_t));
      pos_right = OS_Pos_offen_right + (630 * sin((PI/162) * mister_t));        
      DS535_left.writeMicroseconds(pos_left);
      DS535_right.writeMicroseconds(pos_right);
      //Aktualisierungsrate der Antriebsposition
      wait3();
    }
  }
}

void configuredWS() {
  //Positionieren der OS-Antriebe
  DS535_left.writeMicroseconds(OS_Pos_geschlossen_left);
  DS535_right.writeMicroseconds(OS_Pos_geschlossen_right);

  //Positionieren der WS-Antriebe nach Bahnkurve
  for (mister_t = 0; mister_t <= 144; mister_t = mister_t + 3){
    KSA_left = (int) (KSA_left_null + (176.5 * sin((PI/144) * mister_t)));
    KSA_right = (int) (KSA_right_null - (176.5 * sin((PI/144) * mister_t))); 
    AMX_0902_left.writeMicroseconds(KSA_left);      
    AMX_0902_right.writeMicroseconds(KSA_right);
    //Aktualisierungsrate der Antriebsposition
    wait3();
  }   
}

void adaptiveWS() {
  //Positionierung der ÖS-Motoren
  DS535_left.writeMicroseconds(OS_Pos_geschlossen_left);
  DS535_right.writeMicroseconds(OS_Pos_geschlossen_right); 
  
  //Einlesen der DMS-Werte
  temp_dms_value_2_right = DMS_right.read(); 
  temp_dms_value_2_left = DMS_left.read();

  //Berechnen des gleitenden Durchnschnitts (Mit Offset)
  temp_dms_value_2 = (temp_dms_value_2_right/1000 + temp_dms_value_2_left/100)/2;

  //Aktueller Durchnitt größer als der davor gemessene
  if (temp_dms_value_2 > temp_dms_value_1) { 
    //Positionierung der ÖS-Motoren WARUM?
    //DS535_left.writeMicroseconds(OS_Pos_geschlossen_left);
    //DS535_right.writeMicroseconds(OS_Pos_geschlossen_right);
    
    //Anpassen der WS-Position
    KSA_left = KSA_left + KSA_schritt;
    KSA_right = KSA_right - KSA_schritt;     
    //Positionierung der WS-Antriebe
    AMX_0902_left.writeMicroseconds(KSA_left);
    AMX_0902_right.writeMicroseconds(KSA_right);

    //Überschreiben des gleitenden Durchschnitts
    temp_dms_value_1 = temp_dms_value_2;

    //Wartezeit, bis der nächste DMS-Wert verfügbar wird    
    wait100();
  }

  //Aktueller Durchnitt kleiner als der davor gemessene
  if (temp_dms_value_2 < temp_dms_value_1) { 

    //Positionierung der ÖS-Motoren
    DS535_left.writeMicroseconds(OS_Pos_geschlossen_left);
    DS535_right.writeMicroseconds(OS_Pos_geschlossen_right);
      
    //Anpassen der WS-Position
    KSA_left = KSA_left - KSA_schritt;
    KSA_right = KSA_right + KSA_schritt;

    //Positionierung der WS-Antriebe
    AMX_0902_left.writeMicroseconds(KSA_left);
    AMX_0902_right.writeMicroseconds(KSA_right);
     
    //Überschreiben des gleitenden Durchschnitts
    temp_dms_value_1 = temp_dms_value_2;

    //Wartezeit, bis der nächste DMS-Wert verfügbar wird WARUM
    wait100();
  }
  //Aktueller Durchnitt gleich dem davor gemessenen
  if (temp_dms_value_2 == temp_dms_value_1) { 

    //Positionierung der ÖS-Motoren
    DS535_left.writeMicroseconds(OS_Pos_geschlossen_left);
    DS535_right.writeMicroseconds(OS_Pos_geschlossen_right);

    //Erhalten der WS-Position
    KSA_left = KSA_left;
    KSA_right = KSA_right;     

    //Positionierung der WS-Antriebe
    AMX_0902_left.writeMicroseconds(KSA_left);
    AMX_0902_right.writeMicroseconds(KSA_right);

    //Überschreiben des gleitenden Durchschnitts
    temp_dms_value_1 = temp_dms_value_2;

    //Wartezeit, bis der nächste DMS-Wert verfügbar wird    
    wait100();
  }
}
