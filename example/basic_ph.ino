#include <Arduino.h>
#include "analogPH.h"
#include <Wire.h>

analogPH pHSensor(Wire, 0x49);
pH_calibration_buffer dataSet;

String inputString ;         // a String to hold incoming data
char bufftext[200];
bool stringComplete = false;
int str_idx=0;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  inputString.reserve(200);
  Serial.println(pHSensor.begin()?"sensor found":"not found");
  // put your setup code here, to run once:
}

void loop() {
  delay(500);
  pHSensor.singleReading();
  Serial.println("\t ph = "+String(pHSensor.getPH(),6) +"\t pH stable = "+(pHSensor.isStable())?"stable":"unstable" );
  if(stringComplete)
  {
    
    Serial.println(inputString);
    inputString.trim();
    inputString.toCharArray(bufftext,200);
    if(strcmp(bufftext,"mid")==0)
    {
      pHSensor.calibration(MID_PH);
    }
    else if(strcmp(bufftext,"low")==0)
    {
      pHSensor.calibration(LOW_PH);
    }
    else if(strcmp(bufftext,"hi")==0)
    {
      pHSensor.calibration(HIGH_PH);
    }
    inputString = "";
  };
  
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      return;
    }
    

  }
}