# Senior-Project
//Senior Project RISC Team AutoSampler Code (Version 6-8-21)
/*
//NEW:(4-12-21)
//Code includes stage 3 servo program to test in step through mode but is not put in the auto mode!
//Library must be included for the servo!
//NEW: (4-20-21)
-Mode 4 (setting mode)basic setup with no imediate code to run
-NOTE: need to add LED functions for RGB (for now there is a phoney LedMode4 coded)
-Add place cap clockwise and counterclockwise in manual mode (buttons 7 and 8)
//NEW: (4-27-21)
-RGB_color method added
-Select Mode (RED), Manual Mode (GREEN), Step Through Mode (YELLOW), Autonomous Mode (blink BLUE), Settings Mode (PURPLE), Busy Signal (WHITE)
//NEW: (4-30-21)
-Add Communication section, send data of time/date/location/temperature/humidity to Blynk app when collected sample complete
-Add Setting mode
New: (5-9-21)
-Fix sequence for new layout (rotating disk sensors and stage 3 and 4 position) stage 3 uses DiskSen4 and stage 4 uses DiskSen1
-Adjust sequence in autonomous mode to run sensor changes
-Adjust rotate command in step through mode to run sensor changes
-Error code in SealCap() to limit time to 8.5 sec
-Select mode uses (Blue) LED now
-Adgust LED to be (RED) on busy signal and blink (RED) on Autonomous mode
-New TEST Autonomous mode for speed test of sequence (mode 5) (Hold button 5 and 7)
-Autonomous Mode has time delay and hour to start worked in using the clock module (in while loops)
NEW: (5-12-21)
-Adjust some error timing and delays to run correctly
NEW: (5-13-21)
-Add values for sample data usage
-Add EEPROM funtions (GetData and StoreData) for saving and retrieving sample data
-Change Auto modes to use StoreData functions and run Blink app durring while loop waits. Note: Auto Test mode only has Blink app at end of 3 samples
-Change Auto modes to skip rotateStage1() after the first sequence because stage 4 = stage 1
-Edit delay on conveyor stop (in conveyor funtions) to let the magnet go past the disk fully
-Aux Power is added on RelayAux. This is for the servo functions which now have the second power supply programed (500 millisecond delay to startup)
-Reverse option is avalible for rotating disk in manual mode using relay pin 29
-Add error codes if there is an error (Each code # represents a funtion problem) DiskRotake also has an error delay of 10sec now (ErrorCode 0 = OK)
NEW: (5-20-21)
-Add Blynk code to display sample data for Mode 3 $ 5
NEW: (5-26-21)
-Fixed all relays turn on when starting
-Added functions to display error code to Blynk app
NEW: (5-28-21)
-Added new variables for settings mode: rx_byte, rx_str, stringIntResult, stringFloatResult
-Added two functions to read serial monitor input: readCharToInt, readCharToFloat
-Added settings mode (still needs testing and revisions)
NEW: (6-1-21)
-Fixed and tested Setting Mode
-Checked setting data in EEPROM
-Tested Mode 5 with SampleTime
NEW: (6-7-21)
-Added condition with SampleTime for Mode 5
-Added collected sample data to print in Serial monitor
NEW: (6-8-21)****************************************************************************************************************
-Reduced error time delays for rotating disk functions and increased time delay for sensor input on CapSeal function Good for safty and reliability but check later
*/


//Library ***********
#include <Servo.h>//for servo in stage 3

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#define BLYNK_PRINT Serial
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <EEPROM.h>//for saving data on device for later use (byte storage)

/*******************************************************************/
//DEFINE DHT11 TEMPERATURE & HUMIDITY MODULE
#include <DHT.h>
#define DHTPIN 12 //Digital pin out connected to 12
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

/*******************************************************************/
// DEFINE DS1307 REAL TIME CLOCK
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <WidgetRTC.h>
// Use for real time clock
WidgetRTC rtc;
tmElements_t tm;

/*******************************************************************/
// Auth Token in the Blynk App
// Go to the Project Settings (nut icon).
char auth[] = "V7KmxxGMWWDqU7i0_5KpHvU2jjvPXiSt";
// Your WiFi credentials.
// Set password to "" for open networks.
/*
char ssid[] = "907";
char pass[] = "";
*/
//University of Washington
/*
char ssid[] = "University of Washington";
char pass[] = "";
*/

// RAIN WIFI
char ssid[] = "";
char pass[] = "";

#define EspSerial Serial1//Hardware Serial on Mega 2560
#define ESP8266_BAUD 115200//ESP8266 baud rate
ESP8266 wifi(&EspSerial);
BlynkTimer timer;


/***************************************************************/
//Set pin names and #'s
// INPUT PINS
//SENSORS
const int ConveyorSen1 = 22;//Conveyor Hall effect sensor
const int DiskSen1 = 24;//Disk position 1 Hall effect sensor
const int DiskSen2 = 26;//Disk position 2 Hall effect sensor
const int DiskSen4 = 28;//Disk position 4 Hall effect sensor
const int CapSen1 = 30;//Limit switch for stage 4
//BUTTONS
const int Button1 = 32;
const int Button2 = 34;
const int Button3 = 36;
const int Button4 = 38;
const int Button5 = 40;
const int Button6 = 42;
const int Button7 = 44;
const int Button8 = 46;
const int Button9 = 48;
const int Button10 = 2;//interrupt button clanged to 2 

//OUTPUT PINS
//RELAYS
const int RelayAux = 7;//Auxiliary power supply relay #1 (for servo)
const int RelayConveyorF = 23;//forward conveyor relay#2
const int RelayConveyorR = 25;//reverse conveyor relay#3
const int RelayLinUp = 31;//Linear actuator up relay#4
const int RelayLinDown = 33;//Linear actuator down relay#5
const int RelayPump = 27;//Pump relay#7
const int RelayRotate = 35;//Rotating disk relay#8
const int RelayRotateR = 29;//Rotating disk reverse relay#6
//LEDs
const int LedMode0 = 39;//Select mode LED, RED
const int LedMode1 = 41;//Manual mode LED, GREEN
const int LedMode2 = 43;//Step through/test mode LED, BLUE
//LedMode3 (autonomous) does not have a light to save power
const int LedMode4 = 47;//Place holder for mode 4, YELLOW ******************NEW****(May be deleted check other lines)
const int Led = 45;//Operating LED (runs when busy with a task), RED *****(May be deleted check other lines)
//SERVO
const int ServoPin = 6;//Stage 3 servo in pin 6***********************
Servo Servo1;//Servo is now this name 



//OTHER VARIABLES
volatile int Mode = 0;//Trigger for mode types (also an interrupt value)
unsigned long TimeStart = 0;//used for a starting time reference
int ServoAngle = 0;//start servo position at 0 degrees******

//INTERNAL MEMORY***************************************
int SampleTime = 0;
//Get data values These will contain the current save data from EEPROM after the GetData function is run
float Latitude = 0;
float Longitude = 0;
int ErrorCode = 0;//Marks the type of error incurred (0= no error)
float Temp = 0;
float Humidity = 0;
int Hour = 0;
int Minute = 0;
int Second = 0;
int Day = 0;
int Month = 0;
int Year = 0;
String T0, T1, T2, D0, D1, D2, Time0, Date0, Time1, Date1, Time2, Date2, ErrorTime, ErrorDate;
char rx_byte = 0;
String rx_str = "";
int stringIntResult;
float stringFloatResult;

//SET TIME
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  

void setup() {
  // initialization of pins and starting values
  Serial.begin(9600);
  //INPUTS
  //SENSORS
  pinMode(ConveyorSen1, INPUT_PULLUP);//triggered On at LOW
  pinMode(DiskSen1, INPUT_PULLUP);//internal pull up resistor used
  pinMode(DiskSen2, INPUT_PULLUP);
  pinMode(DiskSen4, INPUT_PULLUP);
  pinMode(CapSen1, INPUT_PULLUP);
  //BUTTONS
  pinMode(Button1, INPUT_PULLUP);//button active on LOW
  pinMode(Button2, INPUT_PULLUP);
  pinMode(Button3, INPUT_PULLUP);
  pinMode(Button4, INPUT_PULLUP);
  pinMode(Button5, INPUT_PULLUP);
  pinMode(Button6, INPUT_PULLUP);
  pinMode(Button7, INPUT_PULLUP);
  pinMode(Button8, INPUT_PULLUP);
  pinMode(Button9, INPUT_PULLUP);
  pinMode(Button10, INPUT_PULLUP);

  //OUTPUTS
  //RELAYS
  pinMode(RelayAux, OUTPUT);
  pinMode(RelayConveyorF, OUTPUT);
  pinMode(RelayConveyorR, OUTPUT);
  pinMode(RelayLinUp, OUTPUT);
  pinMode(RelayLinDown, OUTPUT);
  pinMode(RelayPump, OUTPUT);
  pinMode(RelayRotate, OUTPUT);
  pinMode(RelayRotateR, OUTPUT);

  //LEDS
  pinMode(LedMode0, OUTPUT);
  pinMode(LedMode1, OUTPUT);
  pinMode(LedMode2, OUTPUT);
  //*********************************add LedMode4 output
  pinMode(Led, OUTPUT);

 AllOff();
  //SERVO
  //NOTE******relay sequence if hooked to aux. power*******
  digitalWrite(RelayAux, LOW);//turn servo power on
  delay(5000);//wait 5 seconds to get power
  //Start servo...
  Servo1.attach(ServoPin);//initialize servo to pin
  Servo1.write(ServoAngle);//start servo at 0
  delay(3000);//wait 3 seconds to move
  digitalWrite(RelayAux, HIGH);//turn servo power off

  //Set interrupt as Button10
  attachInterrupt(0, Reset, LOW);//interupt when LOW
  //attachInterrupt(digitalPinToInterrupt(Button10), Reset, LOW);//interupt when LOW

 AllOff();//Turn all values off to start

  //GET EEPROM DATA started************
  //This below line will need to be enabled later and the second one deleted***************************
  GetData(10);//gets SampleTime value
  //EEPROM.put(8, SampleTime);//Put sample time of 8:00am in for now (fixed time value)

  //Error code is reset in Auto modes only! (Below lines are for testing only)
  //ErrorCode = 0;//set error value to OK = 0
  //EEPROM.put(10, ErrorCode);//Store error value in memory


  //COMMUNICATION
  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  delay(10);
  Blynk.begin(auth, wifi, ssid, pass);
  dht.begin();//begin for temperature and humidity sensor
  
}

/**************************************************************************/
void loop() {
  

  //Enter Select Mode (Mode 0)****************
  if(Mode == 0){
    Serial.println("Enter Select Mode");//*****Test line****
    delay(2000);//wait 2 seconds
    //digitalWrite(LedMode0, HIGH);
    RGB_color(0,0,255); //turn on select mode LED - Blue

    while(Mode == 0){//wait for a mode selection in loop
      if(digitalRead(Button1) == LOW){
        //Go to Manual mode
        Mode = 1;
      }else if(digitalRead(Button2) == LOW){
        //Go to Step through/test mode
        Mode = 2;
      }else if(digitalRead(Button3) == LOW){
        //Go to autonomous mode
        Mode = 3;
      }else if(digitalRead(Button4) == LOW){
        //Go to Setting mode
        Mode = 4;
      }else if((digitalRead(Button5) == LOW) && (digitalRead(Button7) == LOW)){
        Mode = 5;//autonomous TEST mode
      }else{
        Mode = 0;//otherwise don't chage
      }
    }
    //digitalWrite(LedMode0, LOW);//turn off select mode LED before exiting
    RGB_color(0,0,0); //turn off select mode LED before exiting
    delay(1000);//give some time before switching
  }




  //Manual Mode (Mode 1)***********************
  if(Mode == 1){
    //Serial.println("Enter Manual Mode");//*****Test line****
    LedBlink(2);//give time to notice a valid press and let user know to wait
    //delay(1000);//give time to remove finger (maybe)
    //digitalWrite(LedMode1, HIGH);//turn on manual LED
    digitalWrite(RelayAux, LOW);//turn servo power on
    delay(5000);//wait 5 seconds to get power
    RGB_color(0,255,0); //turn on manual mode LED - green
    

    while(Mode == 1){
      //check for a manual button to be pressed and run desired device...
      
      //Conveyor forward
      while(digitalRead(Button1) == LOW){
        digitalWrite(RelayConveyorF, LOW);
        //Serial.println("Conveyor Forward");//*****Test line****
      }
      digitalWrite(RelayConveyorF, HIGH);

      //Conveyor reverse
      while(digitalRead(Button2) == LOW){
        digitalWrite(RelayConveyorR, LOW);
        //Serial.println("Conveyor Reverse");//*****Test line****
      }
      digitalWrite(RelayConveyorR, HIGH);

      //Pump
      while(digitalRead(Button3) == LOW){
        digitalWrite(RelayPump, LOW);
        //Serial.println("Pump on");//*****Test line****
      }
      digitalWrite(RelayPump, HIGH);

      //Linear motor extend
      while(digitalRead(Button4) == LOW){
        digitalWrite(RelayLinDown, LOW);
        //Serial.println("Linear motor extend");//*****Test line****
      }
      digitalWrite(RelayLinDown, HIGH);

      //Linear motor retract
      while(digitalRead(Button5) == LOW){
        digitalWrite(RelayLinUp, LOW);
        //Serial.println("Linear motor retract");//*****Test line****
      }
      digitalWrite(RelayLinUp, HIGH);

      //Rotating disk forward
      while(digitalRead(Button6) == LOW){
        digitalWrite(RelayRotate, LOW);
        //Serial.println("Rotating disk on");//*****Test line****
      }
      digitalWrite(RelayRotate, HIGH);

      //Rotating disk Reverse
      while(digitalRead(Button7) == LOW){
        digitalWrite(RelayRotateR, LOW);
        //Serial.println("Rotating disk on");//*****Test line****
      }
      digitalWrite(RelayRotateR, HIGH);

      //Place cap loading ***************
      //Run place cap in clockwise direction
      for(ServoAngle = ServoAngle; (digitalRead(Button8) == LOW)&&(ServoAngle < 180); ServoAngle++){
        Servo1.write(ServoAngle);
        delay(15);//speed of movement
      }

      //Place cap on tube direction************
      //Run place cap in counterclockwise direction
      for(ServoAngle = ServoAngle; (digitalRead(Button9) == LOW)&&(ServoAngle > 0); ServoAngle--){
        Servo1.write(ServoAngle);
        delay(15);//speed of movement
      }
      
      
    }
    //digitalWrite(LedMode1, LOW);//turn off manual mode LED before exiting
    digitalWrite(RelayAux, HIGH);//turn servo power off
    RGB_color(0,0,0); //turn off manual mode LED before exiting
    delay(1000);
  }




  //Step through/test mode (Mode 2)***************
  if(Mode == 2){
    //Serial.println("Enter Step through/test Mode");//*****Test line****
    delay(1000);
    //digitalWrite(LedMode2, HIGH);//Turn on Step through LED
    RGB_color(255,255,0); //turn on step through mode LED - yellow

    while(Mode == 2){
      //check for step Buttons to be pressed and run desired function...

      if(digitalRead(Button1) == LOW){
        //Rotate disk to stage 1
        //digitalWrite(LedMode2, LOW);//Turn off mode light
        RGB_color(0,0,0); //turn off mode light
        delay(500);//give some time
        RotateStage1();
        //digitalWrite(LedMode2, HIGH);//Turn mode light back on showing ready for more input
        RGB_color(255,255,0); //turn mode light back on showing ready for more input
      }else if(digitalRead(Button2) == LOW){
        //Move conveyor forward until in the sewer
        //digitalWrite(LedMode2, LOW);//Turn off mode light
        RGB_color(0,0,0); //turn off mode light
        delay(500);//give some time
        ConveyorF();
        //digitalWrite(LedMode2, HIGH);//Turn mode light back on showing ready for more input
        RGB_color(255,255,0); //turn mode light back on showing ready for more input
      }else if(digitalRead(Button3) == LOW){
        //Move conveyor in reverse until up
        //digitalWrite(LedMode2, LOW);//Turn off mode light
        RGB_color(0,0,0); //turn off mode light
        delay(500);//give some time
        ConveyorR();
        //digitalWrite(LedMode2, HIGH);//Turn mode light back on showing ready for more input
        RGB_color(255,255,0); //turn mode light back on showing ready for more input
      }else if(digitalRead(Button4) == LOW){
        //Rotate disk to stage 2
        //digitalWrite(LedMode2, LOW);//Turn off mode light
        RGB_color(0,0,0); //turn off mode light
        delay(500);//give some time
        RotateStage2();
        //digitalWrite(LedMode2, HIGH);//Turn mode light back on showing ready for more input
        RGB_color(255,255,0); //turn mode light back on showing ready for more input
      }else if(digitalRead(Button5) == LOW){
        //Run pump sequence
        //digitalWrite(LedMode2, LOW);//Turn off mode light
        RGB_color(0,0,0); //turn off mode light
        delay(500);//give some time
        RunPump();
        //digitalWrite(LedMode2, HIGH);//Turn mode light back on showing ready for more input
        RGB_color(255,255,0); //turn mode light back on showing ready for more input
      }else if(digitalRead(Button6) == LOW){
        //Rotate disk to stage 3 (Optiolal)
        //digitalWrite(LedMode2, LOW);//Turn off mode light
        RGB_color(0,0,0); //turn off mode light
        delay(500);//give some time
        RotateStage3();
        //digitalWrite(LedMode2, HIGH);//Turn mode light back on showing ready for more input
        RGB_color(255,255,0); //turn mode light back on showing ready for more input
      }else if(digitalRead(Button7) ==LOW){
        //Run cap placing sequence (optional)
        //digitalWrite(LedMode2, LOW);//Turn off mode light
        RGB_color(0,0,0); //turn off mode light
        delay(500);//give some time
        PlaceCap();
        //digitalWrite(LedMode2, HIGH);//Turn mode light back on showing ready for more input
        RGB_color(255,255,0); //turn mode light back on showing ready for more input
      }else if(digitalRead(Button8) == LOW){
        //Rotate disk to stage 4
        //digitalWrite(LedMode2, LOW);//Turn off mode light
        RGB_color(0,0,0); //turn off mode light
        delay(500);//give some time
        RotateStage4();
        //digitalWrite(LedMode2, HIGH);//Turn mode light back on showing ready for more input
        RGB_color(255,255,0); //turn mode light back on showing ready for more input
      }else if(digitalRead(Button9) == LOW){
        //Run seal cap sequence
        //digitalWrite(LedMode2, LOW);//Turn off mode light
        RGB_color(0,0,0); //turn off mode light
        delay(500);//give some time
        SealCap();
        //digitalWrite(LedMode2, HIGH);//Turn mode light back on showing ready for more input
        RGB_color(255,255,0); //turn mode light back on showing ready for more input
      }else{
        //digitalWrite(LedMode2, HIGH);//otherwise, keep the mode light on
        RGB_color(255,255,0); //otherwise, keep the mode light on
      }
      //...if()
      //...if()
    }
    //digitalWrite(LedMode2, LOW);//Turn off step through LED before exiting
    RGB_color(0,0,0); //turn off step through LED before exiting
    delay(1000);
  }




  //Autonomous Mode (Mode 3)******************************************************
  if(Mode == 3){
    //Blink Led 10 times to enter mode - RED
    LedBlink(10); 
   ErrorCode = 0;//set error value to OK = 0
  EEPROM.put(10, ErrorCode);//Store error value in memory


    for(int i = 0; i < 3; i++){//can take 3 samples
      //TimeStart = millis();//start time stamp

    //EEPROM.get(8, SampleTime); // or
    GetData(10);// Get SampleTime from EEPROM
    timedate();// Run timedate function
      
      while((tm.Hour) != SampleTime){//Set time start here**** (hour logged in memory)
        //Wait here until sample needs to be taken at specified time
        //User can do things while in autonomous mode here like check WiFi
        Serial.println("Autonomous Mode wait 1 (Wait until sample time)");//*****Test line****
        Blynk.run();
        timer.run(); // Initiates BlynkTimer

        if(Mode != 3){
          //Exit autonomous mode if reset button is pressed or Mode changed
          //NOTE (this method can also be used for exiting if errors are detected
          goto ExitMode3;//go to the end of autonomous mode
        }
      }

      //Start sequence...(Take a sample)
      if(i ==0){
        RotateStage1();//Rotate to stage 1 onlt in the first sequence because stage 4 on disk matches stage 1
      }
      if(Mode != 3){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      ConveyorF();
      if(Mode != 3){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);

      //Wait to collect sample
      TimeStart = millis();//set time stamp
      while((millis() - TimeStart) <= 10800000){//Set time delay here***** (3 hours)
        //Wait here until sample is collected 
        //User or device can do things in this loop while waiting
        Serial.println("Autonomous Mode wait 2 (Takeing sample)");//*****Test line****
        //*****************
        Blynk.run();
        timer.run(); // Initiates BlynkTimer

        if(Mode != 3){
          //Exit autonomous mode if reset button is pressed or Mode changed
          goto ExitMode3;//go to the end of autonomous mode
        }
      }

      //Preserve the sample... (finish sequence)
      ConveyorR();
      if(Mode != 3){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      RotateStage2();
      if(Mode != 3){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      RunPump();
      if(Mode != 3){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      RotateStage3();
      if(Mode != 3){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      PlaceCap();
      if(Mode != 3){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      RotateStage4();
      if(Mode != 3){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      SealCap();
      //Sample complete!

      StoreData(i);//Store the sample data to EEPROM memory
    }
    
    //No more samples to take so enter a wait mode if the reset was not pressed yet
    ExitMode3: //Exit of autonomous mode sequence
    while(Mode == 3){
     Blynk.run();
     timer.run(); // Initiates BlynkTimer
     Serial.println("Samples done");
     if (digitalRead(52) == HIGH){//Button in Blynk app connect to PIN 52, ON >> HIGH
        terminal_0();//1st sample data
        terminal_1();//2nd sample data
        terminal_2();//3rd sample data
        Serial.println("Post samples data");
     }
    }

    delay(1000);
    
  }

//**************************************************************************
//Autonomous Mode TEST (Mode 5)(Fast speed for testing)******************************************************
  if(Mode == 5){
    //Blink Led 2 times, then white, then blink... to enter Test auto mode
    LedBlink(2); 
    RGB_color(255,0,0);//red
    delay(800);
    RGB_color(255,255,0);//yellow
    delay(800);
    RGB_color(0,255,0);//green
    delay(800);
    RGB_color(255,0,255);//purple
    delay(800);
    RGB_color(0,0,255);//blue
    delay(800);
    RGB_color(255,255,255);//white
    delay(800);
    RGB_color(0,0,0);

    ErrorCode = 0;//set error value to OK = 0
    EEPROM.put(10, ErrorCode);//Store error value in memory

   //EEPROM.get(8, SampleTime); // or
   GetData(10);// Get SampleTime from EEPROM
   timedate();// Run timedate function


    while((tm.Hour) != SampleTime){//Set time start here**** (hour logged in memory)
        //Wait here until sample needs to be taken at specified time
        //User can do things while in autonomous mode here like check WiFi
        Serial.println("Wait SampleTime");//*****Test line****
        Blynk.run();
        timer.run(); // Initiates BlynkTimer
    }

    for(int i = 0; i < 1; i++){//can take 3 samples (test with i < "sample #" here)     
      TimeStart = millis();//start time stamp
      while((millis() - TimeStart) <= 10000){//Set time delay here**** (10 seconds)
        //Wait here until sample needs to be taken at specified time
        //User can do things while in autonomous mode here like check WiFi
        Serial.println("Autonomous Mode wait 1 (Wait until sample time)");//*****Test line****

        if(Mode != 5){
          //Exit autonomous mode if reset button is pressed or Mode changed
          //NOTE (this method can also be used for exiting if errors are detected
          goto ExitMode5;//go to the end of autonomous mode
        }
      }

      //Start sequence...(Take a sample)
      if(i == 0){
        RotateStage1();//Rotate to stage 1 onlt in the first sequence because stage 4 on disk matches stage 1
      }
      if(Mode != 5){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      ConveyorF();
      if(Mode != 5){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);

      //Wait to collect sample
      TimeStart = millis();//set time stamp
      while((millis() - TimeStart) <= 5000){//Set time delay here***** (5 seconds)
        //Wait here until sample is collected 
        //User or device can do things in this loop while waiting
        Serial.println("Autonomous Mode wait 2 (Takeing sample)");//*****Test line****

        if(Mode != 5){
          //Exit autonomous mode if reset button is pressed or Mode changed
          goto ExitMode5;//go to the end of autonomous mode
        }
      }

      //Preserve the sample... (finish sequence)
      ConveyorR();
      if(Mode != 5){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      RotateStage2();
      if(Mode != 5){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      RunPump();
      if(Mode != 5){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      RotateStage3();
      if(Mode != 5){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      PlaceCap();
      if(Mode != 5){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      RotateStage4();
      if(Mode != 5){
        //Exit autonomous mode if reset button is pressed or Mode changed
        break;
      }
      delay(1000);
      SealCap();
      //Sample complete!

      StoreData(i);//Store the sample data to EEPROM memory
      delay(2000);

      //Print data to serial monitor
      GetData(i);
      Serial.println("*********************************************");
      Serial.println("");
      Serial.println("Samples done");
      Serial.print("Time: ");
      Serial.println(T0);
      //Serial.println(T1);
      //Serial.println(T2);
      Serial.print("Date: ");
      Serial.println(D0);
      //Serial.println(D1);
      //Serial.println(D2);
      Serial.print("Temperature: ");
      Serial.println(Temp);
      Serial.print("Humidity: ");
      Serial.println(Humidity);
      Serial.println("");
      Serial.println("*********************************************");
      delay(5000);
    }//for
    
    //No more samples to take so enter a wait mode if the reset was not pressed yet
    ExitMode5: //Exit of autonomous mode sequence
    
    while(Mode == 5){
     Blynk.run();
     timer.run(); // Initiates BlynkTimer
     //Serial.println("Samples done");
     if (digitalRead(52) == HIGH){//Button in Blynk app connect to PIN 52, ON >> HIGH
        terminal_0();//1st sample data
        terminal_1();//2nd sample data
        terminal_2();//3rd sample data
        Serial.println("Post samples data");
     }
    }
    delay(1000);
  }

//*****************************************************************************
  //Setting Mode (mode 4)*****************
  if(Mode == 4){
    int Button1on = 0, Button2on = 0, Button3on = 0;
    int TimeSet = 0, SampleTimeSet = 0;
    float LatitudeSet = 0, LongitudeSet = 0;
    //digitalWrite(LedMode4, HIGH);//turn setting mode LED on
    RGB_color(255,0,255); //turn setting mode LED on -  purple
    //Serial.println("Setting Mode");
      if(digitalRead(Button1) == LOW){
        Button1on = 1;
        Button2on = 0;
        Button3on = 0;
      }
      if(digitalRead(Button2) == LOW){
        Button1on = 0;
        Button2on = 1;
        Button3on = 0;
      } 
      if(digitalRead(Button3) == LOW){
        Button1on = 0;
        Button2on = 0;
        Button3on = 1;
      }
      
      while (TimeSet == 0 && Button1on == 1 && digitalRead(Button2) == HIGH && digitalRead(Button3) == HIGH){
        //**Set clock(#1) function
        SetTime();
        TimeSet = 1;
        delay(2000);
        }

      while (SampleTimeSet == 0 && Button2on == 1 && digitalRead(Button1) == HIGH && digitalRead(Button3) == HIGH){
        //**Set sample time(#2) function
        Serial.println("*********************************************");
        Serial.println("");
        //Serial.println("Enter desired hour for sample time in 24hr time format:");
        readCharToInt();
        SampleTimeSet = stringIntResult;
        Serial.print("SampleTimeSet:");
        Serial.println(SampleTimeSet);
        EEPROM.put(8, SampleTimeSet);  // put SampleTime value in EEPROM
        delay(2000);
      }
      stringIntResult = 0;
      
      while (LatitudeSet == 0 && Button3on == 1 && digitalRead(Button1) == HIGH && digitalRead(Button2) == HIGH){
        //**Set location(#3)function
        // Set LatitudeSet first done, LongtitudeSet next (set one by one)
        Serial.println("*********************************************");
        Serial.println("");
        //Serial.println("Enter Latitude:");
        readCharToFloat();
        LatitudeSet = stringFloatResult;
        Serial.print("LatitudeSet:");
        Serial.println(LatitudeSet);
        EEPROM.put(0, LatitudeSet);  // put Latitude value in EEPROM
        delay(2000);
      }
      stringFloatResult = 0;// Reset stringFloatResult
      
      while (LongitudeSet == 0 && LatitudeSet != 0 && Button3on == 1 && digitalRead(Button1) == HIGH && digitalRead(Button2) == HIGH){
        Serial.println("*********************************************");
        Serial.println("");
        //Serial.println("Enter longitude:");
        readCharToFloat();
        LongitudeSet = stringFloatResult;
        Serial.print("LongitudeSet:");
        Serial.println(LongitudeSet);
        EEPROM.put(4, LongitudeSet);  // put Longitude value in EEPROM
        delay(2000);
      }
      stringFloatResult = 0;
      
      // Check setting data in EEPROM
      
      EEPROM.get(0, Latitude);//Get Latitude
      EEPROM.get(4, Longitude);//Get Longitude
      EEPROM.get(8, SampleTime);
      EEPROM.get(10, ErrorCode);
      Serial.println("***************************************************");
      Serial.println("");
      Serial.println("Check setting data in EEPROM");
      Serial.print("SampleTime: ");
      Serial.println(SampleTime);
      Serial.print("Latitude: ");
      Serial.println(Latitude);
      Serial.print("Longitude: ");
      Serial.println(Longitude);
      Serial.print("Errorcode: ");
      Serial.print(ErrorCode); 
      delay(5000);
      
      
        //digitalWrite(LedMode4, HIGH);//keep setting mode LED on
        RGB_color(255,0,255); //keep setting mode LED on
      
      //*Display status(#5) function
    //digitalWrite(LedMode4, LOW);//turn setting mode LED off
    RGB_color(0,0,0); //turn setting mode LED off
    
    delay(1000);
  }


  //Error Mode (Mode 9)******************
  if(Mode == 9){
    while(Mode == 9){
      //Serial.println("Error Mode!!!! Oh NO!");//*****Test line****
      //relay error information as needed but hault physical sampling untill reset
      LedBlink(1);//blink Led forever to signal problem until you switch modes (reset)
     Blynk.run();
     timer.run(); // Initiates BlynkTimer
     Serial.println("Error code");
     if (digitalRead(53) == HIGH){//Button in Blynk app connect to PIN 52, ON >> HIGH
        terminal_error();
        Serial.print("Errorcode: ");
        Serial.print(ErrorCode);     
     }
    }
  }
}

//**SUBFUNCTIONS********************


          
//NEW CODE****************

void RGB_color(int redValue, int greenValue, int blueValue) {
  //Change LED color depending on mode
  analogWrite(LedMode0, redValue);
  analogWrite(LedMode1, greenValue);
  analogWrite(LedMode2, blueValue);
}

void AllOff(){
  //Turn all outputs off
  digitalWrite(RelayAux, HIGH);
  digitalWrite(RelayConveyorF, HIGH);
  digitalWrite(RelayConveyorR, HIGH);
  digitalWrite(RelayLinUp, HIGH);
  digitalWrite(RelayLinDown, HIGH);
  digitalWrite(RelayPump, HIGH);
  digitalWrite(RelayRotate, HIGH);
  digitalWrite(RelayRotateR, HIGH);
  digitalWrite(LedMode0, LOW);
  digitalWrite(LedMode1, LOW);
  digitalWrite(LedMode2, LOW);
  digitalWrite(Led, LOW);
  
}

void Reset(){
  //Change Mode to 0 (select mode) for interrupt
  Mode = 0;
}

void ConveyorF(){
  //Function moves conveyor forward to the bottom of the sewer
  //digitalWrite(Led, HIGH);//Light on to say function is busy
  RGB_color(255,0,0); //light on to say function is busy
  digitalWrite(RelayConveyorF, LOW);//Run conveyor forward
  delay(5000);//allow 5 seconds to pass the first sensor flag
  TimeStart = millis();//set time stamp
  while(digitalRead(ConveyorSen1) == HIGH){
    //wait until sensor is triggered at LOW
    
    if((millis() - TimeStart) >= 40000){//set expected conveyor time (40 seconds)***
      //If the conveyor runs too long, there is a problem
      Mode = 9;//Error mode
      ErrorCode = 1;//set error value #1
      EEPROM.put(10, ErrorCode);//Store error value in memory
      break;//force end the code
    }
  }
  delay(1000);//delay 1 second to line up with the bottom
  digitalWrite(RelayConveyorF, HIGH);//Stop the conveyor
  //digitalWrite(Led, LOW);//Light off to say not busy
  RGB_color(0,0,0); //light off to say function is not busy
            
}

void ConveyorR(){
  //Function moves the conveyor with payload in reverse to the top of the disk
  //digitalWrite(Led, HIGH);//Busy light on
  RGB_color(255,0,0); //busy light on
  digitalWrite(RelayConveyorR, LOW);//Run conveyor in reverse
  delay(5000);//allow 5 seconds to pass second sensor flag
  TimeStart = millis();//set time stamp
  while(digitalRead(ConveyorSen1) == HIGH){
    //wait until sensor is triggered at LOW
    
    if((millis() - TimeStart) >= 40000){//set expected conveyor time (40 seconds)***
      //If the conveyor runs too long, there is a problem
      Mode = 9;//Error mode
      ErrorCode = 2;//set error value #2
      EEPROM.put(10, ErrorCode);//Store error value in memory
      break;//force end the code
    }
  }
  delay(2000);//delay 2 seconds to pass the disk fully
  digitalWrite(RelayConveyorR, HIGH);//Stop the conveyor
  //digitalWrite(Led, LOW);//Busy light off
  RGB_color(0,0,0); //busy light off
}


//FUNCTIONS DONE!******

void RotateStage1(){
  //Rotate the disk to stage 1
  //digitalWrite(Led, HIGH);//Light on to say function is busy
  RGB_color(255,0,0); //light on to say function is busy
  digitalWrite(RelayRotate, LOW);//run disk
  TimeStart = millis();//set time stamp
  while (digitalRead(DiskSen1) == HIGH){
    //Wait here until sensor
    //Serial.println("Rotate the disk to stage 1");//*****Test line****
    if((millis() - TimeStart) >= 3500){//set expected conveyor time (3.5 seconds)***
      //If the Disk runs too long, there is a problem
      digitalWrite(RelayRotate, HIGH);//disk off
      delay(500);
      digitalWrite(RelayRotateR, LOW);//Reverse disk!
      delay(1000);
      digitalWrite(RelayRotateR, HIGH);
      Mode = 9;//Error mode
      ErrorCode = 3;//set error value #3
      EEPROM.put(10, ErrorCode);//Store error value in memory
      break;//force end the code
    }
  }
  digitalWrite(RelayRotate, HIGH);//disk off
  //digitalWrite(Led, LOW);//Light off to say not busy
  RGB_color(0,0,0); //busy light off
}

void RotateStage2(){
  //Rotate the disk to stage 2
  //digitalWrite(Led, HIGH);//Light on to say function is busy
  RGB_color(255,0,0); //light on to say function is busy
  digitalWrite(RelayRotate, LOW);//disk on
  TimeStart = millis();//set time stamp
  while (digitalRead(DiskSen2) == HIGH){
    //wait here intil sensor 2
    //Serial.println("Rotate the disk to stage 2");//*****Test line****
    if((millis() - TimeStart) >= 3500){//set expected conveyor time (3.5 seconds)***
      //If the Disk runs too long, there is a problem
      digitalWrite(RelayRotate, HIGH);//disk off
      delay(500);
      digitalWrite(RelayRotateR, LOW);//Reverse disk!
      delay(1000);
      digitalWrite(RelayRotateR, HIGH);
      Mode = 9;//Error mode
      ErrorCode = 4;//set error value #4
      EEPROM.put(10, ErrorCode);//Store error value in memory
      break;//force end the code
    }
  }
  digitalWrite(RelayRotate, HIGH);//disk off
  //digitalWrite(Led, LOW);//Light off to say not busy
  RGB_color(0,0,0); //busy light off
}

void RotateStage3(){
  //Rotate the disk to stage 3 (Optional)****
  //Serial.println("Rotate the disk to stage 3");//*****Test line****
  //digitalWrite(Led, HIGH);//Light on to say function is busy
  RGB_color(255,0,0); //light on to say function is busy
  digitalWrite(RelayRotate, LOW);
  TimeStart = millis();//set time stamp
  while (digitalRead(DiskSen4) == HIGH){
    //Wait until sensor 4
    if((millis() - TimeStart) >= 3500){//set expected conveyor time (3.5 seconds)***
      //If the Disk runs too long, there is a problem
      digitalWrite(RelayRotate, HIGH);//disk off
      delay(500);
      digitalWrite(RelayRotateR, LOW);//Reverse disk!
      delay(1000);
      digitalWrite(RelayRotateR, HIGH);
      Mode = 9;//Error mode
      ErrorCode = 5;//set error value #5
      EEPROM.put(10, ErrorCode);//Store error value in memory
      break;//force end the code
    }
  }
  digitalWrite(RelayRotate, HIGH);
  //digitalWrite(Led, LOW);//Light off to say not busy
  RGB_color(0,0,0); //busy light off
}

void RotateStage4(){
  //Rotate the disk to stage 4
  //NOTE: same position as stage 1 so use the same sensor!
  //digitalWrite(Led, HIGH);//Light on to say function is busy
  RGB_color(255,0,0); //light on to say function is busy
  digitalWrite(RelayRotate, LOW);
  TimeStart = millis();//set time stamp
  while (digitalRead(DiskSen1) == HIGH){
    //wait until sensor 1
    if((millis() - TimeStart) >= 3500){//set expected conveyor time (3.5 seconds)***
      //If the Disk runs too long, there is a problem
      digitalWrite(RelayRotate, HIGH);//disk off
      delay(500);
      digitalWrite(RelayRotateR, LOW);//Reverse disk!
      delay(1000);
      digitalWrite(RelayRotateR, HIGH);
      Mode = 9;//Error mode
      ErrorCode = 6;//set error value #6
      EEPROM.put(10, ErrorCode);//Store error value in memory
      break;//force end the code
    }
  }
  digitalWrite(RelayRotate, HIGH);
  //digitalWrite(Led, LOW);//Light off to say not busy
  RGB_color(0,0,0); //busy light off
}

void RunPump(){
  //Run the pump to add preservative
  //digitalWrite(Led, HIGH);//Light on to say function is busy
  RGB_color(255,0,0); //light on to say function is busy
  digitalWrite(RelayPump, LOW);
  delay(8500);// 8.5 sec to add preservative
  //Serial.println("Add preservative");//*****Test line****
  digitalWrite(RelayPump, HIGH);
  //digitalWrite(Led, LOW);//Light off to say not busy
  RGB_color(0,0,0); //busy light off
}

void PlaceCap(){
  //Place a cap on the test tube at stage 3
  //Serial.println("Place cap");//*****Test line****
  //digitalWrite(Led, HIGH);//Light on to say function is busy
  RGB_color(255,0,0); //light on to say function is busy
  digitalWrite(RelayAux, LOW);//Turn relay on for servo power
  delay(500);//***************Change delay later if not enough time to start
  Servo1.write(ServoAngle);//start at 0
  //Slide open 180 degrees to get cap from the storage stack
  for(ServoAngle = 0; ServoAngle < 180; ServoAngle++){
    Servo1.write(ServoAngle);
    delay(20);//speed of movement
  }
  delay(2000);//wait for cap to drop
  //Slide closed 180 degrees to push cap to drop over tube
  for(ServoAngle = 180; ServoAngle > 0; ServoAngle--){
    Servo1.write(ServoAngle);
    delay(8);//speed of movement
  }
  delay(500);
  digitalWrite(RelayAux, HIGH);//Aux power to servo off
  delay(1000);
  //digitalWrite(Led, LOW);//Light off to say not busy
  RGB_color(0,0,0); //busy light off
}

void SealCap(){
  //Seal cap onto test tube with linear actuator (stage 4)
  //digitalWrite(Led, HIGH);//Light on to say function is busy
  RGB_color(255,0,0); //light on to say function is busy
  
  //Serial.println("Linear motor extend");//*****Test line****
  // linnear motor extend
  digitalWrite(RelayLinDown, LOW);//Turn motor on down
  TimeStart = millis();//set time stamp
  delay(5000);//do not detect sensor input for 5 sec to help with triger problems(Fix this later in wiring and power supply)************
  while(digitalRead(CapSen1) == HIGH){
    //Wait for the limit switch to trigger signaling cap is sealed
    if((millis() - TimeStart) >= 7500){//set expected linear actuator time (7 seconds)***
      //If the motor runs too long, there is a problem Stop and retract before the device breaks!
      Mode = 9;//Error mode
      ErrorCode = 7;//set error value #7
      EEPROM.put(10, ErrorCode);//Store error value in memory
      break;//force end the code
    }
  }
  digitalWrite(RelayLinDown, HIGH);
  delay(1000);//allow time to settle
  // linnear motor retract
  //Serial.println("Linear motor retract");//*****Test line****
  digitalWrite(RelayLinUp, LOW);
  delay(8500);// 8.5 sec to ensure return at initial position*********
  //Serial.println("Linear motor retracted (Done)");//*****Test line**** 
  digitalWrite(RelayLinUp, HIGH);

  //digitalWrite(Led, LOW);//Light off to say not busy
  RGB_color(0,0,0); //busy light off
}

void LedBlink(int n){
  //Blink Led 'n' times
  for (int i = 0; i < n; i++){
    //digitalWrite(Led, HIGH);//turn led on
    RGB_color(255,0,0); //busy light on
    delay(500);//wait 3 seconds
    //digitalWrite(Led, LOW);//turn LED off
    RGB_color(0,0,0); //busy light off
    delay(500);
  }
}

 void readCharToInt() {//Version 5/28/2021 used int
  // read characters from serial and change from string to int

  if (Serial.available() > 0) {   // is a character available?
    rx_byte = Serial.read();      // get the character

    if (rx_byte != '\n') {  // a character was received
      rx_str += rx_byte;    // add character to string
    }

    else {  // end of string
      stringIntResult = rx_str.toInt();  // convert string to int
      rx_str = "";  // clear string for reuse
    }
  }
  return stringIntResult;
}
void readCharToFloat() {
  // read characters from serial and change from string to float

  if (Serial.available() > 0) {   // is a character available?
    rx_byte = Serial.read();      // get the character

    if (rx_byte != '\n') {  // a character was received
      rx_str += rx_byte;    // add character to string
    }

    else {  // end of string
      stringFloatResult = rx_str.toFloat();  // convert string to float
      rx_str = "";  // clear string for reuse
    }
  }
  return stringFloatResult;
}


//EEPROM MEMORY FUNTIONS**********************************
void StoreData(int n){
    //Store Sample data using this funtion 
  //Takes in an int value for the sample number (sample 1,2,3 = n value of 0,1,2)
  if(n == 0){//First sample storage
    timedate();
    Time0 = String(tm.Hour)+':'+ String(tm.Minute)+':'+ String((tm.Second));
    Date0 = String(tm.Month)+'/'+ String(tm.Day)+'/'+ String(tmYearToCalendar(tm.Year));
    //Store temp data
    //Note: can use this for each address change->  eeAddress += sizeof(float); //Move address to the next byte after float 'f'.
    //But, here it is done manually
    //EEPROM.put(eeAddress, customVar);
    
    EEPROM.put(12, dht.readTemperature(true));//Save Temp data in F
    EEPROM.put(16, dht.readHumidity());//Save humidity data
    EEPROM.put(20, tm.Hour);//Hour
    EEPROM.put(22, tm.Minute);//minute
    EEPROM.put(24, tm.Second);//second
    EEPROM.put(26, tm.Day);//day
    EEPROM.put(28, tm.Month);//month
    EEPROM.put(30, tm.Year);//year
    
    EEPROM.put(72, Time0);//Time
    EEPROM.put(92, Date0);//Date
  }

  if(n == 1){//Second sample storage
    timedate();
    Time1 = String(tm.Hour)+':'+ String(tm.Minute)+':'+ String((tm.Second));
    Date1 = String(tm.Month)+'/'+ String(tm.Day)+'/'+ String(tmYearToCalendar(tm.Year));
    EEPROM.put(32, dht.readTemperature(true));//Save Temp data in F
    EEPROM.put(36, dht.readHumidity());//Save humidity data
    EEPROM.put(40, tm.Hour);//Hour
    EEPROM.put(42, tm.Minute);//minute
    EEPROM.put(44, tm.Second);//second
    EEPROM.put(46, tm.Day);//day
    EEPROM.put(48, tm.Month);//month
    EEPROM.put(50, tm.Year);//year

    EEPROM.put(112, Time1);//Time
    EEPROM.put(132, Date1);//Date
  }

  if(n == 2){//Third sample storage
    timedate();
    Time2 = String(tm.Hour)+':'+ String(tm.Minute)+':'+ String((tm.Second));
    Date2 = String(tm.Month)+'/'+ String(tm.Day)+'/'+ String(tmYearToCalendar(tm.Year));
    EEPROM.put(52, dht.readTemperature(true));//Save Temp data in F
    EEPROM.put(56, dht.readHumidity());//Save humidity data
    EEPROM.put(60, tm.Hour);//Hour
    EEPROM.put(62, tm.Minute);//minute
    EEPROM.put(64, tm.Second);//second
    EEPROM.put(66, tm.Day);//day
    EEPROM.put(68, tm.Month);//month
    EEPROM.put(70, tm.Year);//year

    EEPROM.put(152, Time2);//Time
    EEPROM.put(172, Date2);//Date
  }
  
}

//GetData
void GetData(int n){
  //Gets all data stored for sample and puts it in system variables for use as needed in other functions
  //Takes in the sample number (0,1,2)->(1,2,3) and store EEPROM data to int/float values (recombined for show)
  //Note: will get SampleTime data on n = 10
  if(n == 0){// Get First sample data
    EEPROM.get(0, Latitude);//Get Latitude
    EEPROM.get(4, Longitude);//Get Longitude
    EEPROM.get(10, ErrorCode);
    EEPROM.get(12, Temp);
    EEPROM.get(16, Humidity);
    EEPROM.get(20, Hour);
    EEPROM.get(22, Minute);
    EEPROM.get(24, Second);
    EEPROM.get(26, Day);
    EEPROM.get(28, Month);
    EEPROM.get(30, Year);
    
    EEPROM.get(72, T0);//Time
    EEPROM.get(92, D0);//Date
  }

  if(n == 1){//Get Second sample data
    EEPROM.get(0, Latitude);//Get Latitude
    EEPROM.get(4, Longitude);//Get Longitude
    EEPROM.get(10, ErrorCode);
    EEPROM.get(32, Temp);
    EEPROM.get(36, Humidity);
    EEPROM.get(40, Hour);
    EEPROM.get(42, Minute);
    EEPROM.get(44, Second);
    EEPROM.get(46, Day);
    EEPROM.get(48, Month);
    EEPROM.get(50, Year);

    EEPROM.get(112, T1);//Time
    EEPROM.get(132, D1);//Date
  }

  if(n == 2){//Get Third sample data
    EEPROM.get(0, Latitude);//Get Latitude
    EEPROM.get(4, Longitude);//Get Longitude
    EEPROM.get(10, ErrorCode);
    EEPROM.get(52, Temp);
    EEPROM.get(56, Humidity);
    EEPROM.get(60, Hour);
    EEPROM.get(62, Minute);
    EEPROM.get(64, Second);
    EEPROM.get(66, Day);
    EEPROM.get(68, Month);
    EEPROM.get(70, Year);

    EEPROM.get(152, T2);//Time
    EEPROM.get(172, D2);//Date
  }

  if(n == 10){//Get SampleTime for starting values
    EEPROM.get(8, SampleTime);
  }
  
}
//
void ErrorCodeData(){
  EEPROM.get(10, ErrorCode);
  timedate();
  ErrorTime = String(tm.Hour)+':'+ String(tm.Minute)+':'+ String((tm.Second));
  ErrorDate = String(tm.Month)+'/'+ String(tm.Day)+'/'+ String(tmYearToCalendar(tm.Year));
}

//
bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}

void SetTime(){
  bool parse=false;
  bool config=false;

  // get the date and time the compiler was run
  if (getDate(__DATE__) && getTime(__TIME__)) {
    parse = true;
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  }

  Serial.begin(9600);
  while (!Serial) ; // wait for Arduino Serial Monitor
  delay(200);
  if (parse && config) {
    Serial.print("DS1307 configured Time=");
    Serial.print(__TIME__);
    Serial.print(", Date=");
    Serial.println(__DATE__);
  } else if (parse) {
    Serial.println("DS1307 Communication Error :-{");
    Serial.println("Please check your circuitry");
  } else {
    Serial.print("Could not parse info from the compiler, Time=\"");
    Serial.print(__TIME__);
    Serial.print("\", Date=\"");
    Serial.print(__DATE__);
    Serial.println("\"");
  }
}



//*********************************************************

//COMMUNICATION SECTION***************************************************
//DS1307 REAL TIME CLOCK MODULE
//String Time, Date;
void timedate() {
  if (RTC.read(tm)) {
    Serial.print("Time = ");
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.println();
  }
  delay(1000);
}

int print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

//TERMINAL WIDGET IN BLYNK APP TO SAVE COLLECTED SAMPLE DATA
void terminal_0(){
    GetData(0);
    WidgetTerminal terminal(V10);
    // Clear the terminal content
    terminal.clear();
    // Information for 1st sample
    terminal.println(F("DONE!"));
    terminal.print("Time:");
    terminal.println(T0);
  
    terminal.print("Date:");
    terminal.println(D0);
    
    terminal.print("Location:" );
    terminal.print(Latitude);
    terminal.print(":");
    terminal.println(Longitude);
    
    terminal.print("Temperature:");
    terminal.print(Temp);
    terminal.println("F");
    
    terminal.print("Humidity:");
    terminal.println(Humidity);
    
    //upload data to Blynk cloud
    terminal.flush();
}
//
void terminal_1(){
    GetData(1);
    WidgetTerminal terminal(V11);
    // Clear the terminal content
    terminal.clear();
    // Information for 2nd sample
    terminal.println(F("DONE!"));
    terminal.print("Time:");
    terminal.println(T1);
  
    terminal.print("Date:");
    terminal.println(D1);
    
    terminal.print("Location:" );
    terminal.print(Latitude);
    terminal.print(":");
    terminal.println(Longitude);
    
    terminal.print("Temperature:");
    terminal.print(Temp);
    terminal.println("F");
    
    terminal.print("Humidity:");
    terminal.println(Humidity);
    
    //upload data to Blynk cloud
    terminal.flush();
}
//
void terminal_2(){
    GetData(2);
    WidgetTerminal terminal(V12);
    // Clear the terminal content
    terminal.clear();
    // Information for 3rd sample
    terminal.println(F("DONE!"));
    terminal.print("Time:");
    terminal.println(T2);
  
    terminal.print("Date:");
    terminal.println(D2);
    
    terminal.print("Location:" );
    terminal.print(Latitude);
    terminal.print(":");
    terminal.println(Longitude);
    
    terminal.print("Temperature:");
    terminal.print(Temp);
    terminal.println("F");
    
    terminal.print("Humidity:");
    terminal.println(Humidity);
    
    //upload data to Blynk cloud
    terminal.flush();
}
//Errorcode
void terminal_error(){
  WidgetTerminal terminal(V13);
  ErrorCodeData();
  terminal.clear();
  terminal.print("Error Code:");
  terminal.println(ErrorCode);
  
  terminal.print("Time:");
  terminal.println(ErrorTime);
  
  terminal.print("Date:");
  terminal.println(ErrorDate);
  terminal.flush();
}
