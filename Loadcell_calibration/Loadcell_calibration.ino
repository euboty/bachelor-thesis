/*
 * Slightly modified to fit Kontibat project ~ Florian Geissler in November 2020
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   This example file shows how to calibrate the load cell and optionally store the calibration
   value in EEPROM, and also how to change the value manually.
   The result value can then later be included in your project sketch or fetched from EEPROM.

   To implement calibration in your project sketch the simplified procedure is as follow:
       LoadCell.tare();
       //place known mass
       LoadCell.refreshDataSet();
       float newCalibrationValue = LoadCell.getNewCalibration(known_mass);

       Press t to tare, c to set manual parameter, n to calibrate other loadcell.
*/

#include <HX711_ADC.h>
#include <EEPROM.h>

// Force-Sensor Pinout set here according to application
const byte FORCE_RIGHT_DOUT_PIN = 3;
const byte FORCE_RIGHT_SCK_PIN = 4;
const byte FORCE_LEFT_DOUT_PIN = 11; 
const byte FORCE_LEFT_SCK_PIN = 12;

// pins:
byte HX711_dout;
byte HX711_sck;


HX711_ADC *LoadCell = nullptr;    // open issue on github on how to utilize lib in this way

const int calVal_eepromAdress = 0;
long t;

void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");
  delay(2000);
  chooseLoadcell();        //sets pin values of desired loadcell
  loadcellInitializing();    //HX711 constructor
  
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell->update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell->getData();
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 't') tareIt(); //tare
    else if (inByte == 'r') calibrate(); //calibrate
    else if (inByte == 'c') changeSavedCalFactor(); //edit calibration value manually
    else if (inByte == 'n') {
      chooseLoadcell();
      loadcellInitializing();
    }
  }
}

void tareIt(){
  LoadCell->tareNoDelay();
  // check if last tare operation is complete
  if (LoadCell->getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}

void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  Serial.read();
  while (_resume == false) {
    LoadCell->update();
    if (Serial.available() > 0) {
      if (Serial.available() > 0) {
        float i;
        char inByte = Serial.read();
        if (inByte == 't'){
          Serial.println("T pressed.");
          LoadCell->tareNoDelay();
          while (LoadCell->getTareStatus() == false) {
          }
        Serial.println("Tare complete");
        _resume = true;
        }
      }
    }
  }

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell->update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  LoadCell->refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = LoadCell->getNewCalibration(known_mass); //get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");
  //saveToEEPROM(newCalibrationValue);

  Serial.println("End calibration");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'r' from serial monitor.");
  Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
  Serial.println("For other loadcell send 'n' from serial monitor.");
  Serial.println("***");
}

void changeSavedCalFactor() {
  float oldCalibrationValue = LoadCell->getCalFactor();
  boolean _resume = false;
  Serial.println("***");
  Serial.print("Current value is: ");
  Serial.println(oldCalibrationValue);
  Serial.println("Now, send the new value from serial monitor, i.e. 696.0");
  float newCalibrationValue;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        Serial.print("New calibration value is: ");
        Serial.println(newCalibrationValue);
        LoadCell->setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  _resume = false;
  //saveToEEPROM(newCalibrationValue);
  Serial.println("End change calibration value");
  Serial.println("***");
}

void chooseLoadcell() {
  Serial.println("Which loadcell do you wish to calibrate?");
  Serial.println("Send number of desired loadcell over serial:");
  Serial.println("1. Loadcell Right");
  Serial.println("2. Loadcell Left");
  Serial.println();
  while (Serial.available() == 0) {};          // wait till user enters loadcell number
  int loadcell_pointer = Serial.read();
  if (loadcell_pointer == '1') {
    HX711_dout = FORCE_RIGHT_DOUT_PIN;
    HX711_sck = FORCE_RIGHT_SCK_PIN;
  }
  else if (loadcell_pointer == '2') {
    HX711_dout = FORCE_LEFT_DOUT_PIN;
    HX711_sck = FORCE_LEFT_SCK_PIN;
  }
  else {
    Serial.println("Something went wrong. Try again.");
    chooseLoadcell();
  }
}

void loadcellInitializing() {
  LoadCell = new HX711_ADC(HX711_dout, HX711_sck);
  LoadCell->begin();
  long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = false; //set this to false if you don't want tare to be performed in the next step
  LoadCell->start(stabilizingtime, _tare);
  //if (LoadCell->getTareTimeoutFlag() || LoadCell->getSignalTimeoutFlag()) {
    //Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    //while (1);
  //}
  //else {
  LoadCell->setCalFactor(1.0); // user set calibration value (float), initial value 1.0 may be used for this sketch
  Serial.println("Startup is complete");
  //}
  while (!LoadCell->update());
  calibrate(); //start calibration procedure
}

void saveToEEPROM(int newCalibrationValue) {
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");

  boolean _resume = false;
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;

      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }
}
