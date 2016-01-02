/**
  @file termocontrol.ino
  Thermo regulator
  @author David Dorchies
  @date 27/12/2015
  @version 1.0.0
 
  The circuit:
  Connections to LCD 2x16 display :
  - LCD VSS to ground
  - LCD VDD to +5V
  - LCD VO to adjustable resistor
  - LCD RS pin to digital pin 8
  - LCD R/W pin to ground
  - LCD Enable pin to digital pin 7
  - LCD D4 pin to digital pin 6
  - LCD D5 pin to digital pin 5
  - LCD D6 pin to digital pin 4
  - LCD D7 pin to digital pin 3
  - LCD A pin to digital pin 2 with 1K Ohm resistance
  - LCD K pin to ground
  Connections to 2xLM35
  - LM35 inside to analog 0
  - LM35 ouitside to analog 1
  Connections to pushbuttons
  - Push button Set to digital 12
  - Push button Plus to digital 11
  - Push button Minus to digital 10
  Connection to 12V Fan supplied by Vin pin
  - BC547 Base to digital pin 13 through LED + 220 Ohm
 */

// include the libraries 
#include <LiquidCrystal.h>
#include <EEPROM.h>

// DEBUG : uncomment the next line for getting debug print on serial
//#define _DEBUG_

#ifdef _DEBUG_
#define _DBG_INIT_ Serial.begin(9600)
#define _DBG_PRINT_(s) Serial.print(String(ulTime)+": ");Serial.println(s)
#else
#define _DBG_INIT_
#define _DBG_PRINT_(s) 
#endif

// Connections parameters
int aiAPTemp[2] = {0,1}; // Analogic Pin for LM35  inside[0] and outside[1]
int iDPFan = 13; // Digital Pin of "Set" button
int iDPBtnMode = 12; // Digital Pin of "Set" button
int iDPBtnMinus = 11; // Digital Pin of "Minus" button
int iDPBtnPlus = 10; // Digital Pin of "Plus" Button
int iDPLcdLight = 2; // Digital Pin of LCD A pin for LCD light

// Program parameters
unsigned long iTempoMode = 10000; // Set tempo for returning to mode zero in milliseconds
unsigned long iButtonDelay = 500; // delay between push actions in milliseconds
unsigned long iMeasureDelay = 10; // delay between measurements in milliseconds 
unsigned long iDisplayDelay = 2000; // Delay between measurements displays in milliseconds 
const int NB_STORAGE = 50; // Number of time steps stores in memory
unsigned long iAutoRollDisp = 2000; // Delay for autoroll display of stored temperatures in milliseconds

// Program control variables
boolean bButtonMode = false; // true if the button is pushed
boolean bButtonPlus = false; // true if the button is pushed
boolean bButtonMinus = false; // true if the button is pushed
boolean bButtonReady = true; // true if the push button can be handled
boolean bUpdateDisp = true; // true if the temperature display should be updated
boolean bFanOn = false; // Control of fan
boolean bLcdOn = true; // Control of LCD
unsigned long ulTime; // Current Millis
unsigned long ulLightOff; // Millis when light will switch off
unsigned long ulFanReady; // Millis when fan will be able to start/stop (10 sec. from starting arduino)
unsigned long ulModeEnd = 0; // Millis when the function mode will return to zero
unsigned long ulButtonReady = 0; // Millis when a new action could be handle by pushing a button
unsigned long ulMeasure = 0; // Millis when next reading of temperatures
unsigned long ulDisplayTemp = 0; // Millis when next display of temperatures
unsigned long ulStore = 0; // Millis when next data calculation and store
unsigned long ulAutoRollDisp = 0; // Millis when next update of autoroll display
unsigned long ulFanLastStart = 0; // Millis when the fan started last time

/** 
 * Function mode
 * - 0 : Display temperature (Current / Mini / Maxi)
 * - 1 : Display stored mean temperatures
 * - 2 : Fan run stats
 * - 3 : Set target temperature (°C)
 * - 4 : Set minimum delta temperature to fan (°C)
 * - 5 : Set temporisation for fan (sec.)
 * - 6 : Set temporisation for light (sec.)
 * - 7 : Set storage time step
 */
int iMode=0;
// Display Mode
// - 0 : Current temperatures + Target
// - 1 : Minimum temperatures
// - 2 : Maximum temperatures
int iDisp=0;

// Measures
unsigned long aMeasures[2] = {0, 0}; // Sum of temperatures for mean calculation of display temp
word nMeasures = 0; // Count of read measurements for mean calculation of display temp
word aTemp[2]; // Temperature 1/10 °C inside[0] and outside[1]
word aTempMin[2] = {1000, 1000}; // Minimum temperature inside[0] and outside[1]
word aTempMax[2] = {0, 0}; // Maximum temperature inside[0] and outside[1]
unsigned long nTempMean = 0; // count the number of records for mean calculation
unsigned long ulTempSum[2] = {0, 0}; // Sum the temperatures for mean calculation of stored temp

// EEPROM stored variables
const byte byteMagicKey = 181; // MagicKey that should be in first byte of EEPROM to considere reading it
struct eeprom_t {
  word tempTarget = 160; // Target temperature in 1/10 °C
  byte tempDelta = 5; // Minimum delta temp between in & out to fan in 1/10 °C
  unsigned long iTempoFan = 60000; // Fan temporisation for start/stop in milliseconds
  unsigned long iTempoLCD = 120000; // LCD Light temporisation  in milliseconds
  unsigned long iStoreTimeStep = 3600000; // Temperature storage time step in milliseconds
  word aTempMean[2*NB_STORAGE]; // Storage of mean temperature during 50 last storage time step [2n] inside and [2n+1] outside
  word anFanStarts[NB_STORAGE]; // count the number of fan starts by storage time step
  word aiFanRunTime[NB_STORAGE]; // Time of fan run time for each storage time step
} e;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 7, 6, 5, 4, 3); // LiquidCrystal(rs, enable, d4, d5, d6, d7)


void setup() {
  _DBG_INIT_;      // when _DEBUG is defined, open the serial port at 9600 bps:
  digitalWrite(iDPLcdLight,HIGH);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  //lcd.noCursor();
  // Print a message to the LCD.
  lcd.print("Initialisation");
  // Set Digital Pins for push button in pullup mode. See https://www.arduino.cc/en/Tutorial/InputPullupSerial
  pinMode(iDPFan,OUTPUT);
  pinMode(iDPBtnMode, INPUT_PULLUP);    
  pinMode(iDPBtnMinus, INPUT_PULLUP);   
  pinMode(iDPBtnPlus, INPUT_PULLUP);
  pinMode(iDPLcdLight, OUTPUT);
  analogReference(INTERNAL); // Set Analog Voltage to 1.1V. See http://playground.arduino.cc/Main/LM35HigherResolution
  lcd.setCursor(0,1);
  lcd.print("Sensors");
  stabiliseMeasures(); // For avoiding artefic measurement due to voltage change
  delay(iDisplayDelay);
  lcd.setCursor(0,1);
  if(digitalRead(iDPBtnMode)!=LOW) {
    // Reading saved data in EEPROM
    byte byteKey;
    EEPROM.get(0,byteKey);
    if(byteKey == byteMagicKey) {
      // The first byte match the magic key, we can read data !
      lcd.print("Reading EEPROM");
      EEPROM.get(1,e);
    } else {
      lcd.print("Default settings");
    }
  } else {
    lcd.print("Reset settings");
  }
  delay(2000);
  ulTime = millis();
  ulStore = ulTime + e.iStoreTimeStep;
  ulLightOff = ulTime + e.iTempoLCD;
  ulFanReady = ulTime + e.iTempoFan;
}


void loop() {

  // Set current time
  ulTime = millis();

  // Read buttons values 
  unsigned long ulButtonPressed = ulTime + 50; // Button press accepted after 5/100 sec.
  do {
    bButtonMode = (digitalRead(iDPBtnMode)==LOW);
    bButtonPlus = (digitalRead(iDPBtnPlus)==LOW);
    bButtonMinus = (digitalRead(iDPBtnMinus)==LOW);
  } while((bButtonMode || bButtonPlus || bButtonMinus) && (ulButtonPressed > millis()));
  
  // Control function time out
  if(ulModeEnd < ulTime && iMode > 0) {
    iMode = 0;
    iDisp = 0;   
    storeEEPROM();
    bUpdateDisp=true;
  }
  
  // Measurement
  if(ulMeasure < ulTime) {
    for(int i=0; i<2; i++) {
      aMeasures[i] += analogRead(aiAPTemp[i]);
    }
    nMeasures += 1;
    ulMeasure = ulTime + iMeasureDelay;
  }
  
  // Control temperature and fan start/stop
  if(ulDisplayTemp < ulTime) {
    // Temperature measurement. See http://playground.arduino.cc/Main/LM35HigherResolution
    if(nMeasures > 100) {
      float conv = 1 / (nMeasures * 0.931);
      for(int i=0; i<2; i++) {        
        aTemp[i] = aMeasures[i] * conv;
        // Update Min/Max
        if(aTemp[i] < aTempMin[i]) aTempMin[i] = aTemp[i];
        if(aTemp[i] > aTempMax[i]) aTempMax[i] = aTemp[i];
        // Mean calculation
        ulTempSum[i] += aTemp[i];
        aMeasures[i] = 0;
      }
      nMeasures = 0;
      nTempMean++;
      ulDisplayTemp = ulTime + iDisplayDelay;
      if(iMode==0) bUpdateDisp = true;
      
      // Control fan
      if(ulFanReady < ulTime) {
        // Correction of temp out respect to minimum delta temp
        word tempOutCor; // Corrected temp out with delta
        if(aTemp[0]>e.tempTarget) {
          tempOutCor = aTemp[1] + e.tempDelta;
        }else {
          tempOutCor = aTemp[1] - e.tempDelta;
        }
        _DBG_PRINT_("In "+String(aTemp[0])+" OutCor "+String(tempOutCor));
        if((aTemp[0] > tempOutCor)^(aTemp[0] < e.tempTarget)) {
          if(!bFanOn) switchFan();
        }
        else if (bFanOn) {
          switchFan();
        }
      }
    }
  }
  
  // Calculate and store mean temperature and fan stats
  if(ulStore < ulTime && nTempMean > 0) {
    // Temperatures
    for(int i=0; i<2; i++) {
      for(int j=NB_STORAGE-1; j>0; j--) {
        e.aTempMean[2*j+i] = e.aTempMean[2*(j-1)+i];
      }
      e.aTempMean[i] = ulTempSum[i] / nTempMean;
      ulTempSum[i] = 0;
    }
    nTempMean = 0;
    // Fan stats
    if(bFanOn) {
      e.aiFanRunTime[0] += (ulTime - ulFanLastStart)/1000;
      ulFanLastStart = ulTime;
    }
    for(int j=NB_STORAGE-1; j>0; j--) {
      e.anFanStarts[j] = e.anFanStarts[j-1];
      e.aiFanRunTime[j] = e.aiFanRunTime[j-1];
    }
    e.anFanStarts[0] = 0;
    e.aiFanRunTime[0] = 0;
    storeEEPROM();
    ulStore = ulTime + e.iStoreTimeStep;
  }
  
  // Autoroll display for stared mean temperatures and fan stats
  if((iMode == 1 || iMode == 2) && ulAutoRollDisp < ulTime) {
    iDisp += 1;
    if(iDisp > NB_STORAGE-1) iDisp = 1;
    bUpdateDisp = true;
  }

  // Control LCD display and function to display
  if((bButtonMode || bButtonPlus || bButtonMinus) && ulButtonReady < ulTime) {

    // Manage times out
    ulButtonReady = ulTime + iButtonDelay; // A button was pressed and can be handled => Set the next idle time
    ulModeEnd = ulTime + iTempoMode; // Postpone the tempo for returning to display mode
    ulLightOff = ulTime + e.iTempoLCD; // Update time for switching off

    // Control LCD light
    if(!bLcdOn) {
      switchLCD(true);
      return; // the key press only switch on the LCD
    }    
    
    if(bButtonMode) {
      // Set button pushed => Change fonction
      iMode += 1;
      iDisp = 0; // Reset Disp Mode
      if(iMode > 7) iMode=0;
    }
    if(iMode == 1 || iMode == 2) {
      // Postpone the tempo for returning to display mode : special time for autoroll
      ulModeEnd = ulTime + (NB_STORAGE)*iAutoRollDisp;
    }
   
  } else if(!bUpdateDisp) {
    // Control light switch off
    if(ulLightOff < ulTime && bLcdOn) {
      // Switch the display off
      switchLCD(false);
    }
    // No order to be handled => Stop here
    return;
  } else {
    // Button press is not valid (ulButtonReady > ulTime)
    bButtonMode = false;
    bButtonPlus = false;
    bButtonMinus = false;
  }

  // Run functions 
  if((bButtonMode || bButtonPlus || bButtonMinus) || bUpdateDisp) {

    bUpdateDisp = false;
    switch(iMode) {
      case 0:
        // Display Mode
        if(bButtonPlus) {
          iDisp +=1;
          if(iDisp > 2) iDisp = 0;
        }
        if(bButtonMinus) {
          iDisp -=1;
          if(iDisp < 0) iDisp = 2;
        }
        if(bButtonPlus && bButtonMinus) {
          // Reset min & max
          for(int i=0; i<2; i++) {
            aTempMin[i] = aTemp[i];
            aTempMax[i] = aTemp[i];
          }
          displayTemp("RESET MINI MAXI", aTemp);
        } else {
          switch(iDisp) {
            case 0:
              // Display In/Out/Target Temperature
              displayTemp("THERMOSTAT "+tempToString(e.tempTarget), aTemp);
              break;
            case 1:
              // Display Minimum Temperatures
              displayTemp("TEMPERATURE MINI", aTempMin);
              break;
            case 2:
              // Display Maximum Temperatures
              displayTemp("TEMPERATURE MAXI", aTempMax);
              break;
          }
        }
        break;
        
      case 1:
      case 2:
        // Display stored mean temperatures
        if(iDisp < 0) iDisp = 0;
        if(bButtonPlus) {
          iDisp +=1;
          if(iDisp > NB_STORAGE-1) iDisp = 0;
        }
        if(bButtonMinus) {
          iDisp -=1;
          if(iDisp < 0) iDisp = NB_STORAGE-1;
        }
        if(iMode==1) {
          if(e.aTempMean[2*iDisp] == 0) iDisp = 0;
          displayTemp("Mean Temp "+timeToString((iDisp+1)*e.iStoreTimeStep), &e.aTempMean[2*iDisp]);
        } else {
          lcd.clear();
          lcd.print("Fan runs "+timeToString(iDisp*e.iStoreTimeStep));
          lcd.setCursor(0,1);
          lcd.print("Nb.: "+String(e.anFanStarts[iDisp]));
          lcd.print(" - "+timeToString((unsigned long)e.aiFanRunTime[iDisp]*1000));
        }
        ulAutoRollDisp = ulTime + iAutoRollDisp;
        break;
      case 3:
        // Set Target temperature
        e.tempTarget = setTemperature("SET THERMOSTAT", e.tempTarget);
        break;
      case 4:
        // Set MinDelta temperature for running fan
        e.tempDelta = setTemperature("DELTA MIN TEMP.", e.tempDelta);
        break;
      case 5:
        // Set fan temporisation 
        if(bButtonPlus && bButtonMinus) {
          // Manually switch the fan
          switchFan();
        } else {
          e.iTempoFan = setTemporisation("SET TEMPO FAN", e.iTempoFan);
        }
        break;
      case 6:
        // Set LCD temporisation 
        e.iTempoLCD = setTemporisation("SET TEMPO LCD", e.iTempoLCD);
        break;
      case 7:
        // Set Storage time step
        e.iStoreTimeStep = setTemporisation("STORE TIME STEP", e.iStoreTimeStep);
        ulStore = ulTime + e.iStoreTimeStep;
    }
  } 
}


/** 
 * Convert temperature in 1/10 °C to string in °C
 * @param temp in 1/10 °C
 * @return string of formated temperature with 1 decimals
 * @author David Dorchies
 * @date 28/12/2015
 */
String tempToString(word temp){
  byte intPart = temp/10;
  return String(intPart) + "." + String(temp - intPart * 10);
}


/** 
 * Convert time to string
 * @param time time in millisecondes
 * @return string in format 00d00h00m00s with only time units different to zero
 * @author David Dorchies
 * @date 29/12/2015
 */
String timeToString(unsigned long time) {
  unsigned long aulTS[4] = {86400, 3600, 60, 1};
  char acTS[5] = "dhms";
  String sT = "";
  int iT = 0;
  time = time / 1000;
  for(int i=0; i<4; i++) {
    if(time >= aulTS[i]) {
      iT = time / aulTS[i];
      if(iT<10) sT = sT + "0";
      sT = sT + iT;
      sT = sT + acTS[i];
      time = time - iT * aulTS[i];
    }
  }
  return sT;
}


/**
 * Display temperatures
 * @param sMode
 * @param aT[2] Temperatures to display  inside[0] and outside[1]
 * @param tempOut
 * @author David Dorchies
 * @date 28/12/2015
 */
void displayTemp(String sMode, word aT[]) {
  lcd.clear();
  lcd.print(sMode);
  lcd.setCursor(0, 1);
  lcd.print("IN "+tempToString(aT[0])+" OUT "+tempToString(aT[1]));
}


/**
 * Read measurement multiple times for stabilisation
 */
void stabiliseMeasures() {
  int temp;
  // First measures after changing analog ref are not accurate
  for(int i=0; i<100; i++) {
    for(int j=0; i<2; i++) {
      temp = analogRead(aiAPTemp[j]);
    }
  }
  ulMeasure = ulTime + iDisplayDelay;
}


/*
 * Display and set temperature with Minus and Plus buttons
 * @param sName Label to display in first row
 * @param temp Temperature to change
 * @author David Dorchies
 * @date 29/12/2015
 */
word setTemperature(String sName, word temp) {
  if(bButtonPlus) temp += 5;
  if(bButtonMinus) temp -= 5;
  if(temp < 0) temp = 0 ;
  // Display Maximum Temperatures
  lcd.clear();
  lcd.print(sName);
  lcd.setCursor(0, 1);
  lcd.print(tempToString(temp));
  return temp;
}


/**
 * Display and set temporisation with Minus and Plus buttons
 * using adaptable time steps
 * @param sName Label to display in first row
 * @param temp Temperature to change
 * @author David Dorchies
 * @date 29/12/2015
 */
unsigned long setTemporisation(String sName, unsigned long ulTempo) {
  unsigned long aulStep[] = {30,60,300,900,3600};
  int iStep;
  for(iStep=1; iStep<6; iStep++) {
    if(bButtonPlus && ulTempo < aulStep[iStep]*1000) break;
    if(bButtonMinus && ulTempo <= aulStep[iStep]*1000) break;
  }
  if(bButtonPlus) ulTempo += aulStep[iStep-1]*1000;
  if(bButtonMinus) ulTempo -= aulStep[iStep-1]*1000;
  if(ulTempo < 30000) ulTempo = 30000 ;
  // Display Maximum Temperatures
  lcd.clear();
  lcd.print(sName);
  lcd.setCursor(0, 1);
  lcd.print(timeToString(ulTempo));
  return ulTempo;
}


/**
 * Switch the fan on and off
 * @author David Dorchies
 * @date 31/12/2015
 */
void switchFan() {
  if(!bFanOn) {
    // Switch on the fan
    _DBG_PRINT_("Fan ON");
    digitalWrite(iDPFan,HIGH);
    ulFanLastStart = ulTime;
    e.anFanStarts[0] += 1;
  } else {
    // Switch off the fan
    _DBG_PRINT_("Fan OFF");
    digitalWrite(iDPFan,LOW);
    e.aiFanRunTime[0] += (ulTime - ulFanLastStart)/1000;
  }
  bFanOn = !bFanOn;
  ulFanReady = ulTime + e.iTempoFan;
  if(!bLcdOn) switchLCD(true);
  stabiliseMeasures(); // For avoiding artifac measurement due to voltage change
}


/**
 * Switch the LCD on and off
 * @param bLcd switch the LCD screen (true=on, false=off)
 * @author David Dorchies
 * @date 01/01/2016
 */
void switchLCD(boolean bLcd) {
  if(bLcd) {
    digitalWrite(iDPLcdLight,HIGH);
    lcd.display();
    ulLightOff = ulTime + e.iTempoLCD; // Update time for switching off
  } else {
    digitalWrite(iDPLcdLight,LOW);
    lcd.noDisplay();
    // Reset interface
    iMode = 0;
    iDisp = 0;
  }
  bLcdOn = bLcd;
  stabiliseMeasures(); // For avoiding artefic measurement due to voltage change
}

/** 
 *  Store configuration and statistics in EEPROM using EEPROM library (need Arduino IDE >= 1.6.2)
 */
void storeEEPROM() {
  lcd.clear();
  lcd.print("Writing EEPROM");
  EEPROM.put(0, byteMagicKey);
  EEPROM.put(1,e);
}
