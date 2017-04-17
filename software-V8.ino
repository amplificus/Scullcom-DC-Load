//SCULLCOM HOBBY ELECTRONICS
//ELECTRONIC DC LOAD PROJECT
//Software Version 8
//5th February 2017

#include <SPI.h>                              //include SPI library (Serial Peripheral Interface)
#include <Wire.h>                             //include I2C library
#include <LCD.h>                              //
#include <LiquidCrystal_I2C.h>                // F Malpartida's NewLiquidCrystal library
                                              // https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/NewliquidCrystal_1.3.4.zip
#include <math.h>                             //
#include <Adafruit_MCP4725.h>                 //Adafruit DAC library  https://github.com/adafruit/Adafruit_MCP4725
#include <MCP342x.h>                          //Steve Marple library avaiable from    https://github.com/stevemarple/MCP342x
#include <MCP79410_Timer.h>                   //Scullcom Hobby Electronics library  http://www.scullcom.com/MCP79410Timer-master.zip

#include <Keypad.h>                           //http://playground.arduino.cc/Code/Keypad

const byte ROWS = 4;                          //four rows
const byte COLS = 4;                          //four columns

//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {5, 6, 7, 8}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {9, 10, 11, 12}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
char customKey;

Adafruit_MCP4725 dac;                         //constructor

uint8_t address = 0x68;                       //0x68 is the default address for the MCP3426 device
MCP342x adc = MCP342x(address);

const byte MCP79410_ADDRESS = 0x6f;           //0x6f is the default address for the MCP79410 Real Time Clock IC
MCP79410_Timer timer = MCP79410_Timer(MCP79410_ADDRESS);

//Set the pins on the I2C chip used for LCD connections
//ADDR,EN,R/W,RS,D4,D5,D6,D7
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);    //0x27 is the default address of the LCD with I2C bus module

const byte pinA = 3;                          //digital pin (also interrupt pin) for the A pin of the Rotary Encoder
const byte pinB = 4;                          //digital pin for the B pin of the Rotary Encoder

const byte CursorPos = 17;                    //analog pin A3 used as a digital pin to set cursor position (rotary encoder push button)

const byte fan = 2;                           //digital pin 2 for fan control output
const byte temperature = A0;                  //analog pin used for temperature output from LM35
int temp;                                     //
int tempMin = 26;                             //temperature at which to start the fan
int tempMax = 50;                             //maximum temperature when fan speed at 100%
int fanSpeed;

float BatteryLife = 0;                        //
float BatteryLifePrevious = 0;                //
float Seconds = 0;                            //
float BatteryCutoffVolts;                     //used to set battery discharge cut-off voltage (was 3.0)
float MaxBatteryCurrent = 1.0;                //maximum battery current allowed for Battery Capacity Testing

int stopSeconds;                              //store for seconds when timer stopped

int CP = 8;                                   //cursor start position

boolean toggle = false;                       //used for toggle of Load On/Off button

unsigned long controlVoltage = 0;             //used for DAC to control MOSFET

long current = 0;                             //variable used by ADC for measuring the current
long voltage = 0;                             //variable used by ADC for measuring the voltage

float reading = 0;                            //variable for Rotary Encoder value divided by 1000

float setCurrent = 0;                         //variable used for the set current of the load
float setPower = 0;                           //variable used for the set power of the load
float setResistance = 0;                      //variable used for the set resistance of the load
float setCurrentCalibrationFactor = 1.000;    //May use this for calibration adjustment later
float dacCalibrationFactor = 1.010;           //calibration adjustment for the DAC reference voltage of 4.096V

float voltageOffset = 0;                      //variable to store voltage reading zero offset adjustment at switch on
float currentOffset = 0;                      //variable to store current reading zero offset adjustment at switch on

float setControlCurrent = 0;                  //variable used to set the temporary store for control current required

int VoltsDecimalPlaces = 3;                   //number of decimal places used for Voltage display on LCD

float ActualVoltage = 0;                      //variable used for Actual Voltage reading of Load
float ActualCurrent = 0;                      //variable used for Actual Current reading of Load
float ActualPower = 0;                        //variable used for Actual Power reading of Load

float PowerCutOff = 50;                       //maximum Power allowed in Watts - then limited to this level CAN BE CHANGED AS REQUIRED
float CurrentCutOff = 5;                      //maximum Current setting allowed in Amps - then limited to this level
float BatteryCurrent;                         //
float LoadCurrent;                            //

int setReading = 0;                           //

int ControlVolts = 0;                         //used to set output current
float OutputVoltage = 0;                      //

String Mode ="  ";                            //used to identify which mode

int modeSelected = 0;                         //Mode status flag

int lastCount = 50;                           //
volatile float encoderPosition = 0;           //
volatile unsigned long factor= 0;             //number of steps to jump
volatile unsigned long encoderMax = 50000;    //sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED

float LiPoCutOffVoltage = 3.0;
float LiFeCutOffVoltage = 2.8;
float NiCdCutOffVoltage = 1.0;
float ZiZnCutOffVoltage = 1.0;
float PbAcCutOffVoltage = 1.75;
String BatteryType ="    ";

byte exitMode = 0;      //used to exit battery selection menu and return to CC Mode

//--------------------------------Interrupt Routine for Rotary Encoder------------------------
void isr()
{
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  if (interruptTime - lastInterruptTime > 5) {  //
    if (digitalRead(pinB) == LOW)
    {
      encoderPosition = encoderPosition - factor;
    }else{
      encoderPosition = encoderPosition + factor;
    }
    encoderPosition = min(encoderMax, max(0, encoderPosition));  // sets maximum range of rotary encoder
    lastInterruptTime = interruptTime;
  }
    }
//---------------------------------Initial Set up---------------------------------------
void setup() {
 Serial.begin(9600);                                      //used for testing only

 Wire.begin();                                            //join i2c bus (address optional for master)

 MCP342x::generalCallReset();                             // Reset devices
 delay(1);                                                //MC342x needs 300us to settle, wait 1ms - (may not be required)

 pinMode (pinA, INPUT);
 pinMode (pinB, INPUT);

 pinMode (CursorPos, INPUT_PULLUP);

 pinMode (fan, OUTPUT);
 pinMode (temperature, INPUT);
 
 analogReference(INTERNAL);                               //use Arduino internal reference for tempurature monitoring

 attachInterrupt(digitalPinToInterrupt(pinA), isr, LOW);

 dac.begin(0x61);                                         //the DAC I2C address with MCP4725 pin A0 set high
 dac.setVoltage(0,false);                                 //reset DAC to zero for no output current set at Switch On

 lcd.begin(20, 4);                                        //set up the LCD's number of columns and rows 
 lcd.setBacklightPin(3,POSITIVE);                         // BL, BL_POL
 lcd.setBacklight(HIGH);                                  //set LCD backlight on
 
 lcd.clear();                                             //clear LCD display
 lcd.setCursor(6,0);                                      //set LCD cursor to column 0, row 4
 lcd.print("SCULLCOM");                                   //print SCULLCOM to display with 5 leading spaces (you can change to your own)
 lcd.setCursor(1,1);                                      //set LCD cursor to column 0, row 1 (start of second line)
 lcd.print("Hobby Electronics");                          //print Hobby Electronics to display (you can change to your own)
 lcd.setCursor(1,2);
 lcd.print("DC Electronic Load"); //
 lcd.setCursor(0,3);
 lcd.print("Software Version 8.0"); //
 delay(3000);                                             //1500 mSec delay for intro display
 lcd.clear();                                             //clear dislay

 lcd.setCursor(8,0);
 lcd.print("OFF");                                        //indicate that LOAD is off at start up
 Current();                                               //sets current mode to CC (Constant Current) at Power Up
  
 customKey = customKeypad.getKey();
}

//------------------------------------------Main Program Loop---------------------------------
void loop() {
  readKeypadInput();                                     //read Keypad entry

  lcd.setCursor(18,3);                                   //sets display of Mode indicator at bottom right of LCD
  lcd.print(Mode);                                       //display mode selected on LCD (CC, CP, CR or BC)

  reading = encoderPosition/1000;                        //read input from rotary encoder
  
  maxConstantCurrentSetting();                           //set maxiumum Current allowed in Constant Current Mode (CC)
  powerLevelCutOff();                                    //Check if Power Limit has been exceed
  batteryCurrentLimitValue();                            //Battery Discharge Constant Current Limit Value in BC Mode
  displayEncoderReading();                               //display rotary encoder input reading on LCD
  delay(10);                                             //used to test - may not be required
  lastCount = encoderPosition;                           //store rotary encoder current position
  CursorPosition();                                      //check and change the cursor position if cursor button pressed
  readVoltageCurrent();                                  //routine for ADC's to read actual Voltage and Current
  ActualReading();                                       //Display actual Voltage, Current readings and Actual Wattage
  
  dacControl();
  dacControlVoltage();                                   //sets the drive voltage to control the MOSFET
  batteryCapacity();                                     //test if Battery Capacity (BC) mode is selected - if so action
  fanControl();                                          //call heatsink fan control
}

//----------------------Read Keypad Input-----------------------------------------------------
void readKeypadInput (void) {
  customKey = customKeypad.getKey();
  
 // if (customKey != NO_KEY){                             //only used for testing keypad
 // Serial.print("customKey = ");                         //only used for testing keypad
 // Serial.println(customKey);                            //only used for testing keypad
 // }                                                     //only used for testing keypad
               
  if(customKey == 'A'){                                   //check if Constant Current button pressed
  toggle = false;                                         //switch Load OFF
  lcd.setCursor(8,0);
  lcd.print("OFF");
  Current();                                              //if selected go to Constant Current Selected routine
  encoderPosition = 0;                                    //reset encoder reading to zero
  }
         
  if(customKey == 'B'){                                   //check if Constant Power button pressed
  toggle = false;                                         //switch Load OFF
  lcd.setCursor(8,0);
  lcd.print("OFF"); 
  Power();                                                //if selected go to Constant Power Selected routine
  encoderPosition = 0;                                    //reset encoder reading to zero
  }
          
  if(customKey == 'C'){                                   //check if Constant Resistance button pressed  
  toggle = false;                                         //switch Load OFF
  lcd.setCursor(8,0);
  lcd.print("OFF");  
  Resistance();                                           //if selected go to Constant Resistance Selected routine
  encoderPosition = 0;                                    //reset encoder reading to zero
  }

  if(customKey == 'D'){                                   //check if Battery Capacity button pressed
  dac.setVoltage(0,false);                                //Ensures Load is OFF - sets DAC output voltage to 0
  toggle = false;                                         //switch Load OFF
  batteryType();                                          //select battery type
 

    if (exitMode == 1){                                   //if NO battery type selected revert to CC Mode
    lcd.setCursor(8,0);
    lcd.print("OFF");
    Current();                                            //if selected go to Constant Current Selected routine
    encoderPosition = 0;                                  //reset encoder reading to zero
    }
    else
    {
    lcd.setCursor(16,2);
    lcd.print(BatteryType);                               //print battery type on LCD 
    lcd.setCursor(8,0);
    lcd.print("OFF");
    timer.reset();                                        //reset timer
    BatteryLifePrevious = 0;
    BatteryCapacity();                                    //go to Battery Capacity Routine
    }
  }
  
  if(customKey == '*'){                                   //check if ZERO READING key pressed
  zeroOffset();                                           //perform zeroing of voltage and current reading with no load connected  
  }

  if(customKey == '#') {                                //check if Load ON/OFF button pressed
  LoadSwitch();
  }
}

//----------------------Limit Maximum Current Setting-----------------------------------------
void maxConstantCurrentSetting (void) {
  if (Mode == "CC" && reading > CurrentCutOff){           //Limit maximum Current Setting
  reading = CurrentCutOff;
  encoderPosition = (CurrentCutOff * 1000);               //keep encoder position value at maximum Current Limit
  lcd.setCursor(0,3);
  lcd.print("                    ");                      //20 spaces to clear last line of LCD 
  }
}

//----------------------Power Level Cutoff Routine-------------------------------------------
void powerLevelCutOff (void) {
  if (ActualPower  > PowerCutOff){                        //Check if Power Limit has been exceed
  reading = 0;
  encoderPosition = 0; 
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("Exceed Power");
  lcd.setCursor(8,0);
  lcd.print("OFF");
  toggle = false;                                         //switch Load Off
  }
}

//----------------------Constant Current Limit Value------------------------------------------
void batteryCurrentLimitValue (void) {
  if (Mode == "BC" && reading > MaxBatteryCurrent){
  reading = MaxBatteryCurrent;
  encoderPosition = (MaxBatteryCurrent*1000);            //keep encoder position value at 1000mA
  }
}

//----------------------Display Rotary Encoder Input Reading on LCD---------------------------
void displayEncoderReading (void) {
  lcd.setCursor(8,2);                                    //start position of setting cusor position (indicates which digit will change)
  if (reading < 10) {                                    //add a leading zero to display if reading less than 10
  lcd.print("0"); 
  }
  
  lcd.print (reading,3);                                 //show input reading from Rotary Encoder on LCD
  
  lcd.setCursor (CP, 2);                                 //sets cursor position
  lcd.cursor();                                          //show cursor on LCD
}

//--------------------------Cursor Position-------------------------------------------------------
//Change the position routine
void CursorPosition(void) {
  if (digitalRead(CursorPos) == LOW) {
    
  delay(200);                                              //simple key bounce delay  
    CP = CP + 1;
    if (CP==10){
      CP=CP+1;
    } 
  }
  if (CP>13){
    CP=8;                                                 //
  }
  if (CP == 13){
  factor = 1;
  }
  if (CP == 12) {
  factor = 10;
  }
  if (CP == 11){
  factor = 100;
  }
  if (CP == 9){
  factor = 1000;
  }

  if (CP == 8) {                                           //
  factor = 10000;                                          //
  }
}

//-----------------------Read Voltage and Current---------------------------------------------
void readVoltageCurrent (void) {
  
  MCP342x::Config status;
// Initiate a conversion; convertAndRead() will wait until it can be read
  adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain1,                         //"gain1" means we have select the input amp of the ADC to x1
           1000000, voltage, status);
  
// Initiate a conversion; convertAndRead() will wait until it can be read
  adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain4,                         //"gain4" means we have select the input amp of the ADC to x4
           1000000, current, status);
  
}

//----------------------Calculate Actual Voltage and Current and display on LCD---------------------
void ActualReading(void) {
  ActualCurrent = ((((current - currentOffset)*2.048)/32767) * 2.5);        //calculate load current
  ActualVoltage = ((((voltage - voltageOffset)*2.048)/32767) * 50);         //calculate load voltage upto 100v
  ActualPower = ActualVoltage*ActualCurrent;

  if (ActualVoltage <10) {
    VoltsDecimalPlaces = 3;
    }else{
    VoltsDecimalPlaces = 2;
    }
 
  lcd.setCursor(0,1);
    lcd.print(ActualCurrent,3);
    lcd.print("A");
    lcd.print(" ");                                        //two spaces between actual current and voltage readings
    lcd.print(ActualVoltage,VoltsDecimalPlaces);
    lcd.print("V");
    lcd.print(" "); 
    lcd.print(ActualPower,2);
    lcd.print("W");
    lcd.print(" ");
}

//-----------------------DAC Control Voltage for Mosfet---------------------------------------
void dacControlVoltage (void) {
  if (Mode == "CC"){
  setCurrent = reading*1000;                                //set current is equal to input value in Amps
  setReading = setCurrent;                                  //show the set current reading being used
  setControlCurrent = (setCurrent * setCurrentCalibrationFactor)* dacCalibrationFactor;
  controlVoltage = setControlCurrent; 
  }

  if (Mode == "CP"){
  setPower = reading*1000;                                  //in Watts
  setReading = setPower;
  setCurrent = setPower/ActualVoltage;
  setControlCurrent = (setCurrent * setCurrentCalibrationFactor)* dacCalibrationFactor;
  controlVoltage = setControlCurrent;                       //
  }

  if (Mode == "CR"){
  setResistance = reading;                                  //in ohms
  setReading = setResistance;
  setCurrent = (ActualVoltage)/setResistance*1000;
  setControlCurrent = (setCurrent * setCurrentCalibrationFactor )* dacCalibrationFactor;
  controlVoltage = setControlCurrent; 
  }
}

//-----------------------Battery Capacity Discharge Routine-----------------------------------
void batteryCapacity (void) {
  if (Mode == "BC"){
    setCurrent = reading*1000;                             //set current is equal to input value in Amps
    setReading = setCurrent;                               //show the set current reading being used
    setControlCurrent = (setCurrent * setCurrentCalibrationFactor)* dacCalibrationFactor;
    controlVoltage = setControlCurrent;

    lcd.setCursor(0,3);
    lcd.print (timer.getTime());                           //start clock and print clock time

    Seconds = timer.getTotalSeconds();                     //get totals seconds
  
    LoadCurrent = ActualCurrent;                           //if timer still running use present Actual Current reading
    if (timer.status() == 2){                              //if timer is halted then use last Actual Current reading before timer stopped
      LoadCurrent = BatteryCurrent;
      }

  
    BatteryLife = (LoadCurrent*1000)*(Seconds/3600);       //calculate battery capacity in mAh
    lcd.setCursor(9,3);
    BatteryLife = round(BatteryLife);

    if(BatteryLife >= BatteryLifePrevious){                //only update LCD (mAh) if BatteryLife has increased
  
      if (BatteryLife < 10) {                              //add a 3 leading zero to display if reading less than 10
      lcd.print("000");
      }

      if (BatteryLife >= 10 && BatteryLife <100){          //add a 2 leading zero to display
      lcd.print("00");  
      }

      if (BatteryLife >= 100 && BatteryLife <1000){        //add a 1 leading zero to display
      lcd.print("0"); 
      }
  
    lcd.print(BatteryLife,0);
    lcd.setCursor(13,3);
    lcd.print("mAh");
    BatteryLifePrevious = BatteryLife;                      //update displayed battery capacity on LCD
    } 
  }


  if (Mode == "BC" && ActualVoltage <= BatteryCutoffVolts){ //stops clock if battery reached cutoff level and switch load off

  BatteryCurrent = ActualCurrent;
  dac.setVoltage(0,false);                                  //reset DAC to zero for no output current set at switch on
  //Load = 0;                                               //Load is toggled OFF
  toggle = false;
  lcd.setCursor(8,0);
  lcd.print("OFF");                                         //indicate that LOAD is off at start up
  timer.stop();
  }
}

//-----------------------Fan Control----------------------------------------------------------
void fanControl (void) {
  temp = analogRead(temperature);
  temp = temp * 0.107421875; // convert to Celsius
  
  if (temp < tempMin) {                                   //is temperature lower than minimum setting
    fanSpeed = 0;                                         //fan turned off
    digitalWrite(fan, LOW);
    lcd.setCursor(16,0);
    lcd.print(temp);
    lcd.print((char)0xDF);
    lcd.print("C");   
  }
  
  if ((temp >= tempMin) && (temp <= tempMax)){
    fanSpeed = map(temp, tempMin, tempMax, 131, 255);
    Serial.print("Fan Speed");                          //used for testing only
    Serial.println(fanSpeed);                           //used for testing only
    analogWrite(fan, fanSpeed);
    lcd.setCursor(16,0);
    lcd.print(temp);
    lcd.print((char)0xDF);
    lcd.print("C");
  }
}

//-----------------------Toggle Current Load ON or OFF------------------------------
void LoadSwitch(void) {
 
    if(toggle)
    {
      lcd.setCursor(8,0);
      lcd.print("OFF");
      //Load = 0;
      toggle = !toggle;        
    }
    else
    {
      lcd.setCursor(8,0);
      lcd.print("ON ");
      lcd.setCursor(0,3);
      lcd.print("                    ");                 //clear bottom line of LCD
      //Load = 1;
      toggle = !toggle;
    }
  //delay(100);                                          //simple delay for key debounce (commented out if not required)
}

//-----------------------Select Constant Current LCD set up--------------------------------
void Current(void) {
  Mode = ("CC");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");  
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set I = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  CP = 9;                                               //sets cursor starting position to units.
}

//----------------------Select Constant Power LCD set up------------------------------------
void Power(void) {
  Mode = ("CP");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set W = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print("W");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  CP = 9;                                               //sets cursor starting position to units.
}

//----------------------- Select Constant Resistance LCD set up---------------------------------------
void Resistance(void) {
  Mode = ("CR");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");  
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set R = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print((char)0xF4);
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  CP = 9;                                               //sets cursor starting position to units.
}

//----------------------- Select Battery Capacity Testing LCD set up---------------------------------------
void BatteryCapacity(void) {
  Mode = ("BC");
  lcd.setCursor(0,0);
  lcd.print("BATTERY");
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set I = ");
  lcd.setCursor(14,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
}

//----------------------Battery Type Selection Routine------------------------------------------------
void batteryType (void) {
  exitMode = 0;                                         //reset EXIT mode
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Select Battery Type");  
  lcd.setCursor(0,1);
  lcd.print("1=LiPo/Li-Ion 2=LiFe");
  lcd.setCursor(0,2);
  lcd.print("3=NiCd/NiMH   4=ZiZn");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("5=PbAc        6=Exit");                    //20 spaces so as to allow for Load ON/OFF to still show

  customKey = customKeypad.waitForKey();                //stop everything till the user press a key.

  if (customKey == '1'){
  BatteryCutoffVolts = LiPoCutOffVoltage;
  BatteryType = ("LiPo");
    }

  if (customKey == '2'){
  BatteryCutoffVolts = LiFeCutOffVoltage;
  BatteryType = ("LiFe");
    }

  if (customKey == '3'){
  BatteryCutoffVolts = NiCdCutOffVoltage;
  BatteryType = ("NiCd");  
    }

  if (customKey == '4'){
  BatteryCutoffVolts = ZiZnCutOffVoltage;
  BatteryType = ("ZiZn"); 
    }

  if (customKey == '5'){
  BatteryCutoffVolts = PbAcCutOffVoltage;
  BatteryType = ("PbAc"); 
    }

  if (customKey == '6'){                                  //Exit selection screen
  exitMode = 1;
    }
  

  if (customKey == '7' || customKey == '8' || customKey == '9' || customKey == '0' || customKey == 'A' || customKey == 'B' || customKey == 'C' || customKey == 'D' || customKey == '*' || customKey == '#'){
  batteryType();                                                      //ignore other keys
    }
batteryTypeSelected();                                    //briefly display battery type selected and discharge cut off voltage
lcd.clear();

}

//--------------------------Zero Setting Offset Routine--------------------------------------------
void zeroOffset (void) {
  readVoltageCurrent();                                  //routine for ADC to read actual Voltage and Current
  voltageOffset = voltage;
  currentOffset = current;  
}
//--------------------------Set DAC Voltage--------------------------------------------
void dacControl (void) {
  if (!toggle){
  dac.setVoltage(0,false);                                 //set DAC output voltage to 0 if Load Off selected
    if(Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer.status() == 1){
    timer.stop();
    }
  
  }else{
  dac.setVoltage(controlVoltage,false);                   //set DAC output voltage for Range selected
    if(Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer.status() != 1){
    timer.start();
    }
  }
}

//--------------------------Battery Selected Information--------------------------------------------
void batteryTypeSelected (void) {
  if (exitMode !=1){                                      //if battery selection was EXIT then skip this routine
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Battery Selected");
  lcd.setCursor(8,1);
  lcd.print(BatteryType);                                 //display battery type selected
  lcd.setCursor(2,2);
  lcd.print("Discharge Cutoff");
  lcd.setCursor(6,3);
  lcd.print(BatteryCutoffVolts);                          //display battery discharge cut off voltage
  lcd.print(" volts");                                    
  delay(3000);
  }
}

//--------------------------------------------------------------------------------------------------
