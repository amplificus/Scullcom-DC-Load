//SCULLCOM HOBBY ELECTRONICS
//ELECTRONIC DC LOAD PROJECT
//Software Version 2
//19th January 2017

#include <SPI.h>                              //include SPI library (Serial Peripheral Interface)
#include <Wire.h>                             //include I2C library
#include <LCD.h>                              //
#include <LiquidCrystal_I2C.h>                // F Malpartida's NewLiquidCrystal library
                                              // https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/NewliquidCrystal_1.3.4.zip
#include <math.h>                             //
#include <Adafruit_MCP4725.h>                 //Adafruit DAC library  https://github.com/adafruit/Adafruit_MCP4725
#include <MCP342x.h>                          //Steve Marple library avaiable from    https://github.com/stevemarple/MCP342x
#include <MCP79410_Timer.h>                   //Scullcom Hobby Electronics library  http://www.scullcom.com/MCP79410Timer-master.zip

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

const byte CursorPos = 5;                     //digital pin 5 used to set cursor position
const byte LoadOnOff = 6;                     //digital pin 6 used to toggle load on or off
const byte constantCurrent = 7;               //digital pin 7 used to select constant current mode
const byte constantResistance = 8;            //digital pin 8 to select constant resistance mode
const byte constantPower = 9;                 //digital pin 9 to select constant power mode
const byte batteryCapacity = 10;              //digital pin 10 used to select current constant mode setting

const byte fan = 2;                           //digital pin 2 for fan control output
const byte temperature = A0;                  //analog pin used for temperature output from LM35
int temp;                                     //
int tempMin = 26;                             //temperature at which to start the fan
int tempMax = 50;                             //maximum temperature when fan speed at 100%
int fanSpeed;

float BatteryLife = 0;                        //
float BatteryLifePrevious = 0;                //
float Seconds = 0;                            //
float BatteryCutoffVolts = 3.0;               //used to set battery discharge cut-off voltage
float MaxBatteryCurrent = 1.0;                //maximum battery current allowed for Battery Capacity Testing

int stopSeconds;                              //store for seconds when timer stopped

int CP = 8;                                   //cursor start position

int OnOff = 0;                                //
boolean toggle = true;                        //used for toggle of Load On/Off button
int Load = 0;                                 //sets output of DAC on or off (0 = off and 1 = on)

unsigned long controlVoltage = 0;             //used for DAC to control MOSFET

long current = 0;                             //variable used by ADC for measuring the current
long voltage = 0;                             //variable used by ADC for measuring the voltage

float reading = 0;                            //variable for Rotary Encoder value divided by 1000

float setCurrent = 0;                         //variable used for the set current of the load
float setPower = 0;                           //variable used for the set power of the load
float setResistance = 0;                      //variable used for the set resistance of the load
float setCurrentCalibrationFactor = 1.117;    //May use this for calibration adjustment later (was 1.14)
float dacCalibrationFactor = 1.220703125;     //calibration adjustment for the DAC (5000mV / 4096 steps)

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
 Serial.begin(9600);                                     //used for testing only

 Wire.begin();                                            //join i2c bus (address optional for master)

 MCP342x::generalCallReset();                             // Reset devices
 delay(1);                                                //MC342x needs 300us to settle, wait 1ms - (may not be required)

 pinMode (pinA, INPUT);
 pinMode (pinB, INPUT);

 pinMode (LoadOnOff, INPUT_PULLUP);
 pinMode (CursorPos, INPUT_PULLUP);
 pinMode (constantCurrent, INPUT_PULLUP);
 pinMode (constantPower, INPUT_PULLUP);
 pinMode (constantResistance, INPUT_PULLUP);
 pinMode (batteryCapacity, INPUT_PULLUP);
 pinMode (fan, OUTPUT);
 pinMode (temperature, INPUT);
 
 analogReference(INTERNAL);                             //use Arduino internal reference for tempurature monitoring

 attachInterrupt(digitalPinToInterrupt(pinA), isr, LOW);

 dac.begin(0x61);                                       //the DAC I2C address with MCP4725 pin A0 set high
 dac.setVoltage(0,false);                               //reset DAC to zero for no output current set at switch on

 lcd.begin(20, 4);                                      //set up the LCD's number of columns and rows 
 lcd.setBacklightPin(3,POSITIVE);                       // BL, BL_POL
 lcd.setBacklight(HIGH);                                //set LCD backlight on
 
 lcd.clear();                                           //clear LCD display
 lcd.setCursor(6,0);                                    //set LCD cursor to column 0, row 4
 lcd.print("SCULLCOM");                                 //print SCULLCOM to display with 5 leading spaces (you can change to your own)
 lcd.setCursor(1,1);                                    //set LCD cursor to column 0, row 1 (start of second line)
 lcd.print("Hobby Electronics");                        //print Hobby Electronics to display (you can change to your own)
 lcd.setCursor(1,2);
 lcd.print("DC Electronic Load"); //
 lcd.setCursor(0,3);
 lcd.print("Software Version 2.0"); //
 delay(3000);                                           //1500 mSec delay for intro display
 lcd.clear();                                           //clear dislay

 lcd.setCursor(8,0);
 lcd.print("OFF");                                      //indicate that LOAD is off at start up
 Current();                                             //sets current mode to CC (Constant Current) at Power Up
}
//------------------------------------------Main Program Loop---------------------------------
void loop() {
if(digitalRead(constantCurrent)== LOW) {                //check if Constant Current button pressed
  Load = 0;
  lcd.setCursor(8,0);
  lcd.print("OFF");  
  Current();                                            //if selected go to Constant Current Selected routine
}
if(digitalRead(constantPower)== LOW) {                  //check if Constant Power button pressed
  Load = 0;
  lcd.setCursor(8,0);
  lcd.print("OFF");  
  Power();                                              //if selected go to Constant Power Selected routine
}
if(digitalRead(constantResistance)== LOW) {             //check if Constant Resistance button pressed
  Load = 0;
  lcd.setCursor(8,0);
  lcd.print("OFF");  
  Resistance();                                         //if selected go to Constant Resistance Selected routine
}

if(digitalRead(batteryCapacity) == LOW) {
  Load = 0;
  lcd.setCursor(8,0);
  lcd.print("OFF");
  
  timer.reset();
  
  BatteryLifePrevious = 0;
  BatteryCapacity();                                    //go to Battery Capacity Routine
}

  lcd.setCursor(18,2);
  lcd.print(Mode);                                      //display mode selected on LCD (CC, CP or CR)

  reading = encoderPosition/1000;                       //read input from rotary encoder

if (Mode == "CC" && reading > CurrentCutOff){           //Limit maximum Current Setting
  reading = CurrentCutOff;
  encoderPosition = (CurrentCutOff * 1000);             //keep encoder position value at maximum Current Limit
  lcd.setCursor(0,3);
  lcd.print("                    ");                    //20 spaces to clear last line of LCD
  
}

if (ActualPower  > PowerCutOff){                        //Check if Power Limit has been exceed
  reading = 0;
  encoderPosition = 0; 
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("Exceed Power Limit");
  lcd.setCursor(8,0);
  lcd.print("OFF");
  Load = 0;                                              //Load is toggled OFF  
}

if (Mode == "BC" && reading > MaxBatteryCurrent){
  reading = MaxBatteryCurrent;
  encoderPosition = (MaxBatteryCurrent*1000);            //keep encoder position value at 1000mA
}
 
  lcd.setCursor(8,2);                                    //start position of setting cusor position (indicates which digit will change)
  if (reading < 10) {                                    //add a leading zero to display if reading less than 10
  lcd.print("0"); 
}
  
  lcd.print (reading,3);                                 //show input reading from Rotary Encoder on LCD
  
  lcd.setCursor (CP, 2);                                 //sets cursor position
  lcd.cursor();                                          //show cursor on LCD
  delay(10);                                             //used to test - may not be required
  lastCount = encoderPosition;                           //store encoder current position


  CursorPosition();                                      //check and change the cursor position if cursor button pressed

 
  readVoltageCurrent();                                  //routine for ADC to read actual Voltage and Current
  ActualReading();                                       //Display actual Voltage and Current readings
  CursorPosition();                                      //Change the cusor position and reading adjusment factor (for 10's, 100's and 1000's)

if (Mode == "CC"){
  setCurrent = reading*1000;                             //set current is equal to input value in Amps
  setReading = setCurrent;                               //show the set current reading being used
  setControlCurrent = (setCurrent / dacCalibrationFactor)* setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent; 
}

if (Mode == "CP"){
  setPower = reading*1000;                               //in Watts
  setReading = setPower;
  setCurrent = setPower/ActualVoltage;
  setControlCurrent = (setCurrent/ dacCalibrationFactor)* setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent;                    //
}

if (Mode == "CR"){
  setResistance = reading;                               //in ohms
  setReading = setResistance;
  setCurrent = (ActualVoltage)/setResistance*1000;
  setControlCurrent = (setCurrent/ dacCalibrationFactor )* setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent; 
}

if (Mode == "BC"){
  setCurrent = reading*1000;                             //set current is equal to input value in Amps
  setReading = setCurrent;                               //show the set current reading being used
  setControlCurrent = (setCurrent / dacCalibrationFactor)* setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent;

  lcd.setCursor(0,3);
  lcd.print (timer.getTime());                           //start clock and print clock time

  Seconds = timer.getTotalSeconds();                     //get totals seconds
  
LoadCurrent = ActualCurrent;                             //if timer still running use present Actual Current reading
  if (timer.status() == 2){                                   //if timer is halted then use last Actual Current reading before timer stopped
    LoadCurrent = BatteryCurrent;
 }

  
  BatteryLife = (LoadCurrent*1000)*(Seconds/3600); //calculate mAh
  //BatteryLife = (10*1000)*(Seconds/3600);  //for testing only
  lcd.setCursor(9,3);
  BatteryLife = round(BatteryLife);

if(BatteryLife >= BatteryLifePrevious){                   //only update LCD (mAh) if BatteryLife has increased
  
  if (BatteryLife < 10) {                                //add a 3 leading zero to display if reading less than 10
  lcd.print("000");
  Serial.print(BatteryLife);
  Serial.println("A");
  
}
 if (BatteryLife >= 10 && BatteryLife <100){             //add a 2 leading zero to display
  lcd.print("00"); 
Serial.print(BatteryLife);
  Serial.println("B");

  
}
 if (BatteryLife >= 100 && BatteryLife <1000){           //add a 1 leading zero to display
  lcd.print("0"); 
}
  lcd.print(BatteryLife,0);
  lcd.setCursor(13,3);
  lcd.print("mAh");

  BatteryLifePrevious = BatteryLife;
}

  
}


if (Mode == "BC" && ActualVoltage <= BatteryCutoffVolts){             //stops clock if battery reached cutoff level and switch load off

BatteryCurrent = ActualCurrent;

//delay(10);
  
   dac.setVoltage(0,false);                              //reset DAC to zero for no output current set at switch on
   Load = 0;                                             //Load is toggled OFF
   lcd.setCursor(8,0);
   lcd.print("OFF");                                     //indicate that LOAD is off at start up

   //stopTimer();
   timer.stop();

   
//TimerMode = 1;
}


LoadSwitch();                                            //Toggle load on/off routine
if (Load == 0){
  dac.setVoltage(0,false);                               //set DAC output voltage to 0 if Load Off selected

  if(Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer.status() == 1){
    timer.stop();
  }
  
}else{
  dac.setVoltage(controlVoltage,false);                    //set DAC output voltage for Range selected
  
  if(Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer.status() != 1){
    timer.start();
  }
}

fanControl();                                            //call heatsink fan control
}
//--------------------------Cursor Position-------------------------------------------------------
//Change the position
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

if (CP == 8) {                                             //
  factor = 10000;                                          //
}
}
//----------------------Calculate Actual Voltage and Current and display on LCD---------------------
void ActualReading(void) {
  ActualCurrent = (((current*2.048)/32767)*4);            //calculate load current
  ActualVoltage = (((voltage*2.048)/32767) * 50);         //calculate load voltage upto 100v
  ActualPower = ActualVoltage*ActualCurrent;

  if (ActualVoltage <10) {
    VoltsDecimalPlaces = 3;
  }else{
    VoltsDecimalPlaces = 2;
  }
  Serial.print("CURRENT = ");         //for testing only
  Serial.println(ActualCurrent);      //for testing only
  Serial.print("VOLTAGE = ");         //for testing only
  Serial.println(ActualVoltage);      //for testing only
 
  lcd.setCursor(0,1);
  lcd.print(ActualCurrent,3);
  lcd.print("A");
  lcd.print(" ");                                        //two spaces between actual current and voltage readings
  lcd.print(ActualVoltage,VoltsDecimalPlaces);
  lcd.print("V");
  //lcd.setCursor(0,1);
  lcd.print(" "); 
  lcd.print(ActualPower,2);
  lcd.print("W");
  lcd.print(" ");
}
//-----------------------Switch Current Load On or Off------------------------------
void LoadSwitch(void) {
 
  OnOff = digitalRead(LoadOnOff);
  if (OnOff == LOW)
  {
    if(toggle)
    {
      //digitalWrite(13, HIGH);   // set the LED on
      lcd.setCursor(8,0);
      lcd.print("ON ");
      lcd.setCursor(0,3);
      lcd.print("                    ");                 //clear bottom line of LCD
      Load = 1;
      toggle = !toggle;
    }
    else
    {
      //digitalWrite(13, LOW);    // set the LED off
      lcd.setCursor(8,0);
      lcd.print("OFF");
      Load = 0;
      toggle = !toggle;
    }
  }
  //delay(200);  //simple delay for key debounce
}
//-----------------------Select Constant Current--------------------------------
void Current(void) {
  Mode = ("CC");
lcd.setCursor(0,0);
lcd.print("DC LOAD");  
lcd.setCursor(0,2);
lcd.print("                ");
lcd.setCursor(0,2);
lcd.print("Set I = ");
//lcd.setCursor(12,2);
//lcd.print("  ");
lcd.setCursor(14,2);
lcd.print("A");
lcd.setCursor(0,3);                               //clear last line of time info
lcd.print("                    ");                //20 spaces so as to allow for Load ON/OFF to still show
}
//----------------------Select Constant Power------------------------------------
void Power(void) {
Mode = ("CP");
lcd.setCursor(0,0);
lcd.print("DC LOAD");
lcd.setCursor(0,2);
lcd.print("                ");
lcd.setCursor(0,2);
lcd.print("Set W = ");
//lcd.setCursor(12,2);
//lcd.print("  ");
lcd.setCursor(14,2);
lcd.print("W");
lcd.setCursor(0,3);                               //clear last line of time info
lcd.print("                    ");                //20 spaces so as to allow for Load ON/OFF to still show
}
//----------------------- Select Constant Resistance ---------------------------------------
void Resistance(void) {
  Mode = ("CR");
lcd.setCursor(0,0);
lcd.print("DC LOAD");  
lcd.setCursor(0,2);
lcd.print("                ");
lcd.setCursor(0,2);
lcd.print("Set R = ");
//lcd.setCursor(12,2);
//lcd.print("  ");
lcd.setCursor(14,2);
lcd.print((char)0xF4);
lcd.setCursor(0,3);                               //clear last line of time info
lcd.print("                    ");                //20 spaces so as to allow for Load ON/OFF to still show
}

//----------------------- Select Battery Capacity Testing ---------------------------------------
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
lcd.setCursor(0,3);                               //clear last line of time info
lcd.print("                    ");                //20 spaces so as to allow for Load ON/OFF to still show
}

//-----------------------Read Voltage and Current---------------------------------------------
void readVoltageCurrent (void) {
  
  MCP342x::Config status;
// Initiate a conversion; convertAndRead() will wait until it can be read
  adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain1,
           1000000, voltage, status);
  
// Initiate a conversion; convertAndRead() will wait until it can be read
  adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain1,
           1000000, current, status);
  
}
//-----------------------Fan Control-----------------------------------------------------
void fanControl (void) {
  temp = analogRead(temperature);
  Serial.print("Raw Temp = ");
Serial.println(temp);
  temp = temp * 0.107421875; // convert to Celsius
Serial.print("Temp C: ");
  Serial.println(temp);
  
  if (temp < tempMin) {                           //is temperature lower than minimum setting
    fanSpeed = 0;                                 //fan turned off
    digitalWrite(fan, LOW);
    lcd.setCursor(16,0);
    //lcd.print("T = ");
    lcd.print(temp);
    lcd.print((char)0xDF);
    lcd.print("C");
    //lcd.setCursor(5,3);
    //lcd.print("Fan OFF");
    
  }
  if ((temp >= tempMin) && (temp <= tempMax)){
    fanSpeed = map(temp, tempMin, tempMax, 131, 255);
    Serial.print("Fan Speed");                    //for testing only
    Serial.println(fanSpeed);                     //for testing only
    analogWrite(fan, fanSpeed);
    lcd.setCursor(16,0);
    //lcd.print("T = ");
    lcd.print(temp);
    lcd.print((char)0xDF);
    lcd.print("C");
    //lcd.setCursor(5,3);
    //lcd.print("Fan ON ");   
  }
}
