/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------

   Edited For Thrust Stand's HX711 Calibration Testing

   Mahdi Ibne Kamal nov2025
*/


#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

//pins:
const int HX711_DT = 4; 
const int HX711_SCK = 5; 

//HX711 constructor:
HX711_ADC LoadCell(HX711_DT, HX711_SCK);

const int calibration_value_eeprom_address = 0;
unsigned long time_in_ms = 0;

void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive

  LoadCell.start(2000, true); // ----> (stabilizing_time, tare)

  bool time_out = LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag();

  if (time_out) {
    Serial.println("Timeout!");
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate(); //start calibration procedure
}

void loop() {

  const int print_interval = 0; // increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update())
  // get smoothed value from the dataset:
    if (millis() > time_in_ms + print_interval) {
      float i = LoadCell.getData();
      Serial.print("Load_cell: ");
      Serial.println(i);
      time_in_ms = millis();
    }
  

  // receive command from serial terminal
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay(); //tare
    else if (inByte == 'r') calibrate(); //calibrate
    else if (inByte == 'c') change_calibration_factor(); //edit calibration value manually
  }

  // check if last tare operation is complete
  if (LoadCell.getTareStatus()) 
    Serial.println("Tare complete");
  

}

void calibrate() {
  Serial.println("_______________________________________________________________");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean resume = false;
  while (!resume) {

    LoadCell.update();

    if (Serial.available() > 0) 
        if (Serial.read() == 't') LoadCell.tareNoDelay();
      
    
    if (LoadCell.getTareStatus()) {
      Serial.println("Tare complete");
      resume = true;
    }

  }

  Serial.println("Place a known mass and enter its value (i.e. 100.0)");

  float known_mass = 0;
  resume = false;
  while (!resume) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        resume = true;
      }
    }
  }

  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  float new_calibration_value = LoadCell.getNewCalibration(known_mass); //get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(new_calibration_value);

  Serial.println();

  Serial.print("Save to EEPROM adress ");
  Serial.print(calibration_value_eeprom_address);
  Serial.println("? [y/n]");

  resume = false;
  while (!resume) {
    if (Serial.available() > 0) {

      char inByte = Serial.read();

      if (inByte == 'y') {

      #if defined(ESP8266)|| defined(ESP32)
              EEPROM.begin(512);
      #endif
              EEPROM.put(calibration_value_eeprom_address, new_calibration_value);
      #if defined(ESP8266)|| defined(ESP32)
              EEPROM.commit();
      #endif


        EEPROM.get(calibration_value_eeprom_address, new_calibration_value);
        Serial.print("Value ");
        Serial.print(new_calibration_value);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calibration_value_eeprom_address);

        resume = true;
      }

      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        resume = true;
      }
    }
  }

  Serial.println("End calibration");
  Serial.println("_______________________________________________________________");
  Serial.println("send 'r' -> re-calibrate and 'c' -> manual correction.");
  Serial.println("_______________________________________________________________");
}

void change_calibration_factor() {

  float old_calibration_value = LoadCell.getCalFactor();

  boolean resume = false;

  Serial.println("_______________________________________________________________");
  Serial.print("Current value is: ");
  Serial.println(old_calibration_value);
  Serial.println("Enter new factor, e.g. 696.0");

  float new_calibration_value;

  while (!resume) {
    if (Serial.available() > 0) {
      new_calibration_value = Serial.parseFloat();
      if (new_calibration_value != 0) {
        Serial.print("New calibration factor: ");
        Serial.println(new_calibration_value);
        LoadCell.setCalFactor(new_calibration_value);
        resume = true;
      }
    }
  }

  resume = false;
  Serial.print("Save to EEPROM adress ");
  Serial.print(calibration_value_eeprom_address);
  Serial.println("? [y/n]");

  while (!resume) {
    if (Serial.available() > 0) {

      char inByte = Serial.read();
      if (inByte == 'y') {


      #if defined(ESP8266)|| defined(ESP32)
              EEPROM.begin(512);
      #endif
              EEPROM.put(calibration_value_eeprom_address, new_calibration_value);
      #if defined(ESP8266)|| defined(ESP32)
              EEPROM.commit();
      #endif

        EEPROM.get(calibration_value_eeprom_address, new_calibration_value);
        Serial.print("Value ");
        Serial.print(new_calibration_value);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calibration_value_eeprom_address);
        resume = true;
      }

      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        resume = true;
      }
    }
  }
  Serial.println("End change calibration value");
  Serial.println("_______________________________________________________________");
}
