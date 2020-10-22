#include <HX711.h>

#include <Servo.h>


#define PI 3.1415926535897932384626433832795

Servo DS535_left;
Servo DS535_right;
Servo AMX_0902_left;
Servo AMX_0902_right;
HX711 DMS_left;
HX711 DMS_right;

//Variablen für Zeitüberwachung
int mister_t = 0;
unsigned long time_now = 0;
int period = 3;


//DMS-Anschlüsse
const int DMS_left_DOUT_PIN = 3;
const int DMS_left_SCK_PIN = 4;
const int DMS_right_DOUT_PIN = 12; 
const int DMS_right_SCK_PIN = 11;

//Variablen für gleitenden Durchschnitt
long temp_value_1_right = 0;
long temp_value_1_left = 0;
long temp_value_2_right = 0;
long temp_value_2_left = 0;
long temp_value_1;
long temp_value_2;

//Programmauswahl
int Program = 1;
char recievedChar;
boolean newData = false;

// ÖS-Antriebspositionierung
int pos_left;
int pos_right;
int pos = 0;   
int Winkelbereich_OS = 630;
int OS_Pos_offen_right = 1550;
int OS_Pos_geschlossen_right = OS_Pos_offen_right + Winkelbereich_OS;
int OS_Pos_offen_left = 1732;
int OS_Pos_geschlossen_left = OS_Pos_offen_left-Winkelbereich_OS;
int schritt = Winkelbereich_OS/10;

// WS-Antriebspositioinierung
int KSA_left_null = 1300;
int KSA_right_null = 1855;
int KSA_right; 
int KSA_left; 
int KSA_schritt = 18; 


//Vorbereitungsprogramm
void setup() {

  //Anschließen der DMS
  DMS_left.begin(DMS_left_DOUT_PIN, DMS_left_SCK_PIN);
  DMS_left.is_ready();
  DMS_right.begin(DMS_right_DOUT_PIN, DMS_right_SCK_PIN);
  DMS_right.is_ready();

  //Anschließen der Antriebe
  DS535_left.attach(6);
  DS535_right.attach(5);
  AMX_0902_left.attach(9);
  AMX_0902_right.attach(10);

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
  DMS_right.set_scale();
  DMS_right.tare();
  DMS_right.get_units(10);
  DMS_right.set_scale();
  
  DMS_left.set_scale();
  DMS_left.tare();
  DMS_left.get_units(10);
  DMS_left.set_scale();
  delay(500);
  
  //Messung der unbelasteten DMS-Werte
  temp_value_1_right = DMS_right.read_average(20);
  temp_value_1_left = DMS_left.read_average(20);
  
  //Bilden des ursprünglichen Wertes des gleitenden Durchsschnittes
  temp_value_1 = (temp_value_1_right/1000 + temp_value_1_left/100)/2;
  delay(500);

  //Baudrate des seriellen Ports (:= 57,6 kbps)
  Serial.begin(57600);
}

//Hauptprogramm
void loop() {
  
//Kommunikation mit dem Webserver
recvOneChar();
showNewData();

/*__________________________________________________
Programm zum Schließen*/  
if  (Program == 2) {
  
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



/*__________________________________________________
Programm zur Kraftregelung der WS-Position*/
if (Program == 3) {

  //Positionierung der ÖS-Motoren
  DS535_left.writeMicroseconds(OS_Pos_geschlossen_left);
  DS535_right.writeMicroseconds(OS_Pos_geschlossen_right); 
  
  //Einlesen der DMS-Werte
  temp_value_2_right = DMS_right.read(); 
  temp_value_2_left = DMS_left.read();

  //Berechnen des gleitenden Durchnschnitts (Mit Offset)
  temp_value_2 = (temp_value_2_right/1000 + temp_value_2_left/100)/2;

  //Aktueller Durchnitt größer als der davor gemessene
  if (temp_value_2 > temp_value_1) { 

    //Positionierung der ÖS-Motoren
    DS535_left.writeMicroseconds(OS_Pos_geschlossen_left);
    DS535_right.writeMicroseconds(OS_Pos_geschlossen_right);

    //Anpassen der WS-Position
    KSA_left = KSA_left + KSA_schritt;
    KSA_right = KSA_right - KSA_schritt;     

    //Positionierung der WS-Antriebe
    AMX_0902_left.writeMicroseconds(KSA_left);
    AMX_0902_right.writeMicroseconds(KSA_right);

    //Überschreiben des gleitenden Durchschnitts
    temp_value_1 = temp_value_2;

    //Wartezeit, bis der nächste DMS-Wert verfügbar wird    
    wait100();
  }

  //Aktueller Durchnitt kleiner als der davor gemessene
  if (temp_value_2 < temp_value_1) { 

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
    temp_value_1 = temp_value_2;

    //Wartezeit, bis der nächste DMS-Wert verfügbar wird 
    wait100();
  }
  //Aktueller Durchnitt gleich dem davor gemessenen
  if (temp_value_2 == temp_value_1) { 

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
    temp_value_1 = temp_value_2;

    //Wartezeit, bis der nächste DMS-Wert verfügbar wird    
    wait100();
  }
}

/*__________________________________________________
Programm zur WS-Positionierung nach Bahnkurve*/
if (Program == 4) {

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

/*__________________________________________________
Programm zum Öffnen*/
if (Program == 1) {
  DS535_left.attach(6);
  DS535_right.attach(5);
  AMX_0902_left.writeMicroseconds(KSA_left_null);
  AMX_0902_right.writeMicroseconds(KSA_right_null);

  //Positionieren der ÖS-Antriebe nach Bahnkurve
  if ( pos_left == OS_Pos_geschlossen_left && pos_right == OS_Pos_geschlossen_right){
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
 }

 
/*__________________________________________________
Funktionen zum Erhalten der Anweisungen vom Server*/
void recvOneChar() {
    if (Serial.available() > 0) {
        recievedChar = Serial.read();
        Program = recievedChar;
        newData = true;
    }
}
void showNewData() {
    if (newData == true) {
        newData = false;
    }
}

/*__________________________________________________
Funktionen zum Warten*/
void wait3() {
  time_now = millis();
      while(millis() < time_now + period){
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
  