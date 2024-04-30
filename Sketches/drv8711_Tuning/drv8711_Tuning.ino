//##########################################################################
/*   Example sketch demonstrating the DRV8711 library for use with BOOST-DRV8711 Module 
     Gives a Serial interface to make runtme adjustments to key paramaters;
     
     Available commands over the serial:
     X=x where x is selected Axis (0:X, 1:Y, 2:Z)
     I=x.xx where x.xx is current in amps
     S=xx where xx is microstep value (1,2,4,8,16,32,64,126,256)
     M=x where x is decay mode (0:SLOW, 1:SLOW/MIXED, 2:FAST, 3:MIXED , 4:SLOW/AUTO, 5:AUTOMIXED) 
     B=x.xx where x.xx is TBLANK time in uS 
     D=x.x where x.x is TDECAY time in uS
     O=x.x where x.x is TOFF time in uS
     A=0/1 ABT enable/disable
     E=0/1 Motor enable/disable
     R=Reset to Defaults
*/
//##########################################################################

#include <drv8711.h>
#include <SPI.h>
#include "src\button_input.h"

// Pin definitions
const byte ResetPin = 49;
const byte SleepPin = 53;
const byte SM1_ChipSelectPin = 47;
const byte SM2_ChipSelectPin = 46;
const byte ACT_ChipSelectPin = 40;

// Initialise an array of drv8711 objects for the drivers
drv8711 Axis[3] = { drv8711(SM1_ChipSelectPin), drv8711(SM2_ChipSelectPin), drv8711(ACT_ChipSelectPin) };  //Parameter is Serial Chip Select (SCS) pin for Driver

// Constants for current calc / setting
const float ISENSE = 0.05;                                    // Value of current sense resistors in ohms
const float CurrentLimit = 4.5;                               // Current Limit ( do not set above 4.5 for BOOST-DRV8711 )
const float ISGAIN40_max = 2.75 * 255 / (256 * 40 * ISENSE);  // Max Current for each ISGAIN Value.
const float ISGAIN20_max = 2.75 * 255 / (256 * 20 * ISENSE);
const float ISGAIN10_max = 2.75 * 255 / (256 * 10 * ISENSE);
const float ISGAIN5_max = 2.75 * 255 / (256 * 5 * ISENSE);

// variables for decoded parameters
int ISGAIN = 0;      // Currently Selected ISGAIN Value
float Amps = 0.00;   // For Calculated Peak Current
float TBLANK = 0.0;  // For Blanking Time in uS
String ABT = "";     // For ABT State
float TDECAY = 0.0;  // For Decay Time in uS
float TOFF = 0.0;    // For PWM Off time in uS

// variables for reading status register
int readDelay = 1000;
long int LastRead = 0;

// variables for serial input
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

int currentAxis = 0;  // Selected Axis

//##########################################################################
void setup()
//##########################################################################
{
  // Setup the buttons
  setupButtons();
  // Setup the frequency at which each step will be taken.
  

  // Wake Modules
  pinMode(SleepPin, OUTPUT);
  digitalWrite(SleepPin, HIGH);
  delay(1);

  // Reset the driver before initialisation
  pinMode(ResetPin, OUTPUT);
  digitalWrite(ResetPin, HIGH);
  delay(10);
  digitalWrite(ResetPin, LOW);
  delay(1);

  digitalWrite(50, HIGH); // CIPO is initially set HIGH during setup. Default pin is D50 on the Mega 2560.

  // Start Serial
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  // Write library defaults to Drivers
  // Setup Stepper Motor 1
  Axis[0].stepper_motor_init();
  Axis[0].disable();
  Axis[0].WriteAllRegisters();
  Axis[0].ReadAllRegisters();
  // Setup Stepper Motor 2
  Axis[1].stepper_motor_init();
  Axis[1].disable();
  Axis[1].WriteAllRegisters();
  Axis[1].ReadAllRegisters();
  // Setup Actuator
  Axis[2].actuator_init();
  Axis[2].disable();
  Axis[2].WriteAllRegisters();
  Axis[2].ReadAllRegisters();


  // Print a summary of key settings
  displaySettings();
}

//##########################################################################
void loop()
//##########################################################################
{
  //
  actionForButton();

  // Periodically check status register, and print any flagged errors
  if (millis() > LastRead + readDelay) {
    Axis[currentAxis].get_status();
    if (Axis[currentAxis].G_STATUS_REG.UVLO) {
      Serial.println("ERROR: UnderVoltage Lockout");
    } else {
      if (Axis[currentAxis].G_STATUS_REG.STDLAT) Serial.println("ERROR: Latched Stall Detect");
      if (Axis[currentAxis].G_STATUS_REG.BPDF) Serial.println("ERROR: Channel B Predriver Fault");
      if (Axis[currentAxis].G_STATUS_REG.APDF) Serial.println("ERROR: Channel A Predriver Fault");
      if (Axis[currentAxis].G_STATUS_REG.BOCP) Serial.println("ERROR: Channel B Over Current");
      if (Axis[currentAxis].G_STATUS_REG.AOCP) Serial.println("ERROR: Channel A Over Current");
      if (Axis[currentAxis].G_STATUS_REG.OTS) Serial.println("ERROR: Over Temperature");
    }
    LastRead = millis();
    Axis[currentAxis].clear_status();
  }

  //Serial Interface
  serialEvent();  //read any serial input.

  if (stringComplete) {  // if input line complete
    parseInput();

    //clear input
    inputString = "";
    stringComplete = false;

    //refresh Settings Summary
    Axis[currentAxis].ReadAllRegisters();
    displaySettings();
  }
}

//##########################################################################
void serialEvent()
//##########################################################################
{
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

//##########################################################################
void displaySettings()
//##########################################################################
{
  // Get ISGAIN as a Number
  switch (Axis[currentAxis].G_CTRL_REG.ISGAIN) {
    case ISGAIN_5:
      ISGAIN = 5;
      break;
    case ISGAIN_10:
      ISGAIN = 10;
      break;
    case ISGAIN_20:
      ISGAIN = 20;
      break;
    case ISGAIN_40:
      ISGAIN = 40;
      break;
  }
  //calculate human friendly values
  Amps = 2.75 * Axis[currentAxis].G_TORQUE_REG.TORQUE / (256 * ISENSE * ISGAIN);
  TBLANK = (Axis[currentAxis].G_BLANK_REG.TBLANK + 1) * 0.02;
  if (Axis[currentAxis].G_BLANK_REG.ABT) {
    ABT = "ON";
  } else {
    ABT = "OFF";
  }
  TDECAY = (Axis[currentAxis].G_DECAY_REG.TDECAY + 1) * 0.5;
  TOFF = (Axis[currentAxis].G_OFF_REG.TOFF + 1) * 0.5;

  //output summary
  Serial.println("############################################################################################");
  Serial.println(" DRV8711 Settings Summary");
  Serial.println(" (X) Selected Axis: " + String(currentAxis) + " (0:X, 1:Y, 2:Z)");
  Serial.println(" (I) Peak Current: " + String(Amps) + "A    ( TORQUE: " + String(Axis[currentAxis].G_TORQUE_REG.TORQUE) + " , ISGAIN: " + String(ISGAIN) + " )");
  Serial.println(" (S) MicroStep Mode: 1/" + String(pow(2, Axis[currentAxis].G_CTRL_REG.MODE), 0));
  Serial.println(" (M) Decay Mode: " + String(Axis[currentAxis].G_DECAY_REG.DECMOD) + " (0:SLOW, 1:SLOW/MIXED, 2:FAST, 3:MIXED , 4:SLOW/AUTO, 5:AUTOMIXED)");
  Serial.println(" (B) TBLANK: " + String(TBLANK, 2) + "uS (" + String(Axis[currentAxis].G_BLANK_REG.TBLANK) + ")");
  Serial.println(" (A) ABT: " + ABT);
  Serial.println(" (D) TDECAY: " + String(TDECAY, 1) + "uS");
  Serial.println(" (O) TOFF: " + String(TOFF, 1) + "uS");
  Serial.print(" (E) Motor ");
  if (Axis[currentAxis].G_CTRL_REG.ENBL) {
    Serial.println("Enabled");
  } else {
    Serial.println("Disabled");
  }
  Serial.println("############################################################################################");
}

//##########################################################################
void parseInput()
//##########################################################################
{
  // parse serial input
  int eqPos = inputString.indexOf('=');
  if (eqPos > 0) {  // if input contains '='
    // split input into command and value
    String command = inputString.substring(0, eqPos);
    String value = inputString.substring(eqPos + 1);
    // call function based on command, and pass value
    if (command == "I") setCurrent(value.toFloat());
    if (command == "S") setMicroSteps(value.toInt());
    if (command == "M") setDecayMode(value.toInt());
    if (command == "B") setTBLANK(value.toFloat());
    if (command == "D") setTDECAY(value.toFloat());
    if (command == "O") setTOFF(value.toFloat());
    if (command == "A") setABT(value.toInt());
    if (command == "E") setMotor(value.toInt());
    if (command == "X") selectAxis(value.toInt());

  } else {  //parse any simple commands
    String command = inputString.substring(0, 1);
    if (command == "R") resetDefaults();
    //    if (command == "?" ) displayHelp();
  }
}

//##########################################################################
void setCurrent(float newAmps)
//##########################################################################
{
  unsigned int newISGAIN;
  unsigned int newTORQUE;
  if (newAmps >= 0 and newAmps <= CurrentLimit) {
    if (newAmps <= ISGAIN40_max) {
      newISGAIN = ISGAIN_40;
      newTORQUE = int(newAmps / (ISGAIN40_max / 255));
    } else {
      if (newAmps <= ISGAIN20_max) {
        newISGAIN = ISGAIN_20;
        newTORQUE = int(newAmps / (ISGAIN20_max / 255));
      } else {
        if (newAmps <= ISGAIN10_max) {
          newISGAIN = ISGAIN_10;
          newTORQUE = int(newAmps / (ISGAIN10_max / 255));
        } else {
          newISGAIN = ISGAIN_5;
          newTORQUE = int(newAmps / (ISGAIN5_max / 255));
        }
      }
    }
    Axis[currentAxis].G_TORQUE_REG.TORQUE = newTORQUE;
    Axis[currentAxis].G_CTRL_REG.ISGAIN = newISGAIN;
    Axis[currentAxis].WriteAllRegisters();
  }
}

//##########################################################################
void setMicroSteps(int steps)
//##########################################################################
{
  int smode = Axis[currentAxis].G_CTRL_REG.MODE;

  switch (steps) {
    case (1):
      smode = STEPS_1;
      break;
    case (2):
      smode = STEPS_2;
      break;
    case (4):
      smode = STEPS_4;
      break;
    case (8):
      smode = STEPS_8;
      break;
    case (16):
      smode = STEPS_16;
      break;
    case (32):
      smode = STEPS_32;
      break;
    case (64):
      smode = STEPS_64;
      break;
    case (128):
      smode = STEPS_128;
      break;
    case (256):
      smode = STEPS_256;
      break;
  }
  Axis[currentAxis].G_CTRL_REG.MODE = smode;
  Axis[currentAxis].WriteAllRegisters();
}

//##########################################################################
void setDecayMode(int dmode)
//##########################################################################
{
  if (dmode >= 0 and dmode < 6) {
    Axis[currentAxis].G_DECAY_REG.DECMOD = dmode;
    Axis[currentAxis].WriteAllRegisters();
  }
}

//##########################################################################
void setTBLANK(float newTime)
//##########################################################################
{
  if (newTime >= 1 and newTime <= 5.1) {
    Axis[currentAxis].G_BLANK_REG.TBLANK = int((newTime) / 0.02) - 1;
    Axis[currentAxis].WriteAllRegisters();
  }
}

//##########################################################################
void setTDECAY(float newTime)
//##########################################################################
{
  if (newTime >= 0.5 and newTime < 128) {
    Axis[currentAxis].G_DECAY_REG.TDECAY = int(newTime * 2) - 1;
    Axis[currentAxis].WriteAllRegisters();
  }
}

//##########################################################################
void setTOFF(float newTime)
//##########################################################################
{
  if (newTime >= 0.5 and newTime < 128) {
    Axis[currentAxis].G_OFF_REG.TOFF = int(newTime * 2) - 1;
    Axis[currentAxis].WriteAllRegisters();
  }
}

//##########################################################################
void setMotor(int stat)
//##########################################################################
{
  if (stat == 1) {
    Axis[currentAxis].enable();
  } else {
    Axis[currentAxis].disable();
  }
}

//##########################################################################
void setABT(int stat)
//##########################################################################
{
  if (stat == 1) {
    Axis[currentAxis].G_BLANK_REG.ABT = ON;
  } else {
    Axis[currentAxis].G_BLANK_REG.ABT = OFF;
  }
  Axis[currentAxis].WriteAllRegisters();
}

//##########################################################################
void resetDefaults()
//##########################################################################
{
  Axis[0].stepper_motor_init();
  Axis[1].stepper_motor_init();
  Axis[2].actuator_init();

}

//##########################################################################
void selectAxis(int axis)
//##########################################################################
{
  if (axis >= 0 and axis < 3) {
    currentAxis = axis;
  }
}

//##########################################################################
void enableMotor()
//##########################################################################
{
  Axis[currentAxis].enable();
}

//##########################################################################
void disableMotor()
//##########################################################################
{
  Axis[currentAxis].disable();
}

