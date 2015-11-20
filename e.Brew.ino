// e.Brew
// A PID controller for single-kettle brewing. 
// by Richard Oostindie
//
// heavilly based on:
// Sous Vide Controller
// Bill Earl - for Adafruit Industries
//
// Based on the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregard
//------------------------------------------------------------------

// PID & PID Autotune
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
double Input;
double Output;
double Kp;
double Ki;
double Kd;
double PIDsetpoint;
PID myPID(&Input, &Output, &PIDsetpoint, Kp, Ki, Kd, DIRECT); //Specify the links and initial tuning parameters
int WindowSize = 10000; // 10 second Time Proportional Output window
unsigned long windowStartTime;
byte ATuneModeRemember=2;
double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;
boolean tuning = false;
PID_ATune aTune(&Input, &Output);


// Adafruit RGB/LCD Shield
#include <Wire.h>
#include <utility/Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
#define BUTTON_SHIFT BUTTON_SELECT
unsigned long lastInput = 0; // last button press
byte degreeSMALL[8] = {B01000, B10100, B01000, B00111, B01000, B01000, B01000, B00111};         //degree c symbol
byte SP_Symbol[8] = {B11100, B10000, B11100, B00111, B11101, B00111, B00100, B00100};           //SP Symbol
byte degree[8] = {B00110,B01001, B01001, B00110, B00000,B00000, B00000, B00000 };               //degree c symbol 2
byte barchar[8] = {B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111};                    //full bar symbol
byte hourglasschar[8] = {0b11111,0b10001,0b01110,0b00100,0b01010,0b10001,0b11111,0b00000};      //hourglass symbol
byte HeatONOFF[8] = {B00000, B01010, B01010, B01110, B01110, B01010, B01010, B00000};           //Heat symbol
byte RevHeatONOFF[8] = {B11111, B10101, B10101, B10001, B10001, B10101, B10101, B11111};        //reverse heat symbol
byte thermometer[8] = {B00100,B01010,B01010,B01110,B01110,B11111,B11111,B01110};                //thermometer symbol


//DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2
double sensor2;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempSensor1, tempSensor2;
String StrKettleSensor;
int IntKettleSensor;


//EEPROM
#include <EEPROM.h>
const int AlarmSpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;
const int TMarginAddress = 32;


// Timer stuff
#include <elapsedMillis.h>
elapsedMillis timeElapsed; //declare global if you don't want it reset every time loop runs
unsigned int interval = 60000; // 1 minute interval
int ElapsedMin = 0;


// Buzzer
#define buzzerPin 3
#define NOTE_C7  2093
#define NOTE_E7  2637
#define NOTE_G7  3136



// Pin definitions
#define RelayPin 7



// Variables

double TimerMargin;
int stage = 0;
double Setpoint[6] = {0,0,0,0,0,0};
int SpPos = 1;
double timer[6] = {0,0,0,0,0,0};
int TmPos = 1;
double hop[6] = {0,0,0,0,0,0};
int HopPos = 1;

int AlarmRaised = 0;
int BoilTimer = 0;
volatile long onTime = 0;

double AlarmSetpoint;



// ************************************************
// DiSplay Variables and constants
// ************************************************


const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO, SET_TIMER, RESET, TIMERMARGIN, SellSensor, SETHOP, SETALARM};
operatingState opState = OFF;

// ************************************************
// Sensor Variables and constants





/*************************************************
 * Public Constants for notes
 *************************************************/
 



// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
   Serial.begin(115200);
   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

/*
  String StrKettleSensor;
  for (int i = 41; i < 42; ++i)
  {
    if (EEPROM.read(i) != 0) { StrKettleSensor += char(EEPROM.read(i));  }
  }
  if(StrKettleSensor == "")
  {
    IntKettleSensor = 1;
  }
  else
  {
    IntKettleSensor = StrKettleSensor.toInt();
  }
*/


  pinMode(3, OUTPUT);//buzzer
   // Initialize Relay Control:

   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start
   pinMode (buzzerPin, OUTPUT) ;   // Output mode to drive buzzer

   // Initialize LCD DiSplay 

   lcd.begin(16, 2);
   lcd.createChar(1, degree); // create degree symbol from the binary
   lcd.createChar(2, hourglasschar); // create hourglass symbol from the binary
   lcd.createChar(3, RevHeatONOFF); // create heat symbol from the binary
   lcd.createChar(4, barchar); // create full bar symbol from the binary
   lcd.createChar(5, SP_Symbol); // create full bar symbol from the binary   
   lcd.createChar(6, thermometer); // create full bar symbol from the binary   
    //lcd.setBacklight(HIGH);
   
   lcd.setBacklight(TEAL);
   lcd.print(F("     e.Brew"));
   lcd.setCursor(0, 1);
   lcd.print(F("  Starting . . ."));

   // Start up the DS18B20 One Wire Temperature Sensor

   sensors.begin();
   if (!sensors.getAddress(tempSensor1, 0)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("Sensor Error"));
      buzz(buzzerPin, NOTE_C7, 300,3);
   }
   else
   {
      // Starting without errors, 2 short beeps
      buzz(buzzerPin, NOTE_C7, 100,2);
   }
    sensors.getAddress(tempSensor2, 1);
    sensors.setResolution(tempSensor1, 12);
   sensors.setWaitForConversion(false);


  // Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;

   delay(2000);  // Splash screen  
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   // wait for button release before changing state
   while(ReadButtons() != 0) {}

   lcd.clear();

   switch (opState)
   {
   case OFF:
      Off();
      break;
   case SETP:
      Tune_Sp();
      break;
    case RUN:
      Run();
      break;
   case TUNE_P:
      TuneP();
      break;
   case TUNE_I:
      TuneI();
      break;
   case TUNE_D:
      TuneD();
      break;
   case SET_TIMER:
      Tune_Time();
      break;
   case RESET:
      Reset();
      break;
   case TIMERMARGIN:
      Tune_TimerMargin();
      break;
   case SellSensor:
      SelectKettleSensor();
      break;
   case SETHOP:
      Hop_Alarm();
      break;
   case SETALARM:
      SetAlarm();
      break;
   }
}

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
  myPID.SetMode(MANUAL);
  digitalWrite(RelayPin, LOW);  // make sure it is off
  lcd.print(F("     e.Brew"));
  lcd.setCursor(0, 1);
  lcd.print(F("     READY"));
  uint8_t buttons = 0;

  while(!buttons)
  {
    buttons = ReadButtons();

    if (buttons == BUTTON_RIGHT && Setpoint[1] != 0 && timer[1] != 0 && sensors.getDeviceCount() >= 1) // Run program
    {
      // Prepare to transition to the RUN state
      sensors.requestTemperatures(); // Start an asynchronous temperature reading
      //turn the PID on
      myPID.SetMode(AUTOMATIC);
      windowStartTime = millis();
      opState = RUN; // start control
    }
    if (buttons == BUTTON_SELECT) // Continuously show temperature 
    {
      lcd.clear();
      ShowTemp();
    }

    if (buttons == BUTTON_UP) // Timer wizzard
    {
      lcd.clear();
      SpPos = 1;
      Tune_Sp();
    }
    if (buttons == BUTTON_DOWN) // Config wizzard
      {
        lcd.clear();
        Tune_TimerMargin();
      }
      if (buttons == BUTTON_LEFT) // Clear memory
      {
        Reset();
      }

   }
}



// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// RIGHT for tuning parameters
// LEFT for OFF
// SHIFT for 10x tuning
// ************************************************

void Tune_Sp()
{
  if (Setpoint[SpPos] == 0 && SpPos >= 2)
  {
    Setpoint[SpPos] = Setpoint[(SpPos-1)];
  }
  lcd.print(F("Set Temperature"));
  lcd.print(SpPos);
  delay(200); // delay for up   
   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();

      float increment = 1;
      if (buttons & BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons & BUTTON_LEFT)
      {
         if (SpPos == 1)
         {
          opState = OFF;
         return;
         }
         else
         {
          TmPos = SpPos - 1;
          opState = SET_TIMER;
          return;
         }
      }
      if (buttons & BUTTON_RIGHT)
      {
         TmPos = SpPos;
         opState = SET_TIMER;
         return;
      }
      if (buttons & BUTTON_UP && (Setpoint[SpPos] + increment) <= 100)
      {
         Setpoint[SpPos] += increment;
         delay(200);
      }
      if (buttons & BUTTON_DOWN && (Setpoint[SpPos] - increment) >= 0 && (Setpoint[SpPos] -increment >= Setpoint[(SpPos-1)] )       )
      {
         Setpoint[SpPos] -= increment;
         delay(200);
      }

      if ((millis() - lastInput) > 10000)  // return to OF after 10 seconds idle
      {
         opState = OFF;
         return;
      }

      lcd.setCursor(0,1);
      if (Setpoint[SpPos] >= 100)
      {
        lcd.print("Boil ");
      }
      else
      {
        int tt = Setpoint[SpPos];
        lcd.print(tt);
        lcd.print(" ");
        lcd.write(1);
        lcd.print(F("C   "));
      }
      DoControl();
   }
}

void Tune_Time()
{
  //if (timer[TmPos] == 0 && TmPos >= 2)
  //{
    //timer[TmPos] = timer[(TmPos-1)];
  //}

   lcd.print(F("Set duration "));
  lcd.print(TmPos);   
   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();

      float increment = 1;
      if (buttons & BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons & BUTTON_LEFT)
      {
        SpPos = TmPos;
         opState = SETP;
         return;
      }
      if (buttons & BUTTON_RIGHT && TmPos == 6)
      {
         opState = OFF;
         return;
      }

      if (buttons & BUTTON_RIGHT && Setpoint[SpPos] <= 99)
      {
         SpPos = TmPos + 1;
         opState = SETP;
         return;
      }
      if (buttons & BUTTON_RIGHT && Setpoint[SpPos] >= 100)
      {
         BoilTimer = timer[TmPos];
         opState = SETHOP;
         return;
      }
      
      if (buttons & BUTTON_UP)
      {
         timer[TmPos] += increment;
         delay(200);
      }
      if (buttons & BUTTON_DOWN && (timer[TmPos] - increment) >= 0)
      {
         timer[TmPos] -= increment;
         delay(200);
      }

   
      if ((millis() - lastInput) > 10000)  // return to OF after 10 seconds idle
      {
         opState = OFF;
         return;
      }
      int tt = timer[TmPos];
      lcd.setCursor(0,1);
      lcd.print(tt);
      lcd.print(" minutes  ");
      DoControl();
   }
}



void Hop_Alarm()
{
  if (hop[HopPos] == 0 && HopPos == 1)
  {
    hop[HopPos] = BoilTimer;
  }
  if (hop[HopPos] == 0)
  {
    hop[HopPos] = hop[(HopPos-1)] - 1;
  }

   lcd.print(F("Hop alarm "));
  lcd.print(HopPos);   
   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();

      float increment = 1;
      if (buttons & BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons & BUTTON_LEFT && HopPos > 1)
      {
        HopPos--;
         opState = SETHOP;
         return;
      }
      if (buttons & BUTTON_LEFT && HopPos == 1)
      {
         opState = OFF;
         return;
      }
      if (buttons & BUTTON_RIGHT && HopPos == 6)
      {
         opState = OFF;
         return;
      }

      if (buttons & BUTTON_RIGHT && hop[HopPos] == 0)
      {
         opState = OFF;
         return;
      }


      if (buttons & BUTTON_RIGHT && HopPos <= 5)
      {
         HopPos++;
         opState = SETHOP;
         return;
      }
      if (buttons & BUTTON_RIGHT && HopPos == 6)
      {
         opState = OFF;
         return;
      }
      
      if (buttons & BUTTON_UP && (hop[HopPos] +increment < hop[(HopPos-1)] ) && HopPos > 1 )
      {
         hop[HopPos] += increment;
         delay(200);
      }
      if (buttons & BUTTON_UP && HopPos == 1 &&     hop[HopPos] + increment <= BoilTimer   )
      {
         hop[HopPos] += increment;
         delay(200);
      }
      if (buttons & BUTTON_DOWN && (hop[HopPos] - increment) >= 0)
      {
         hop[HopPos] -= increment;
         delay(200);
      }

   
      if ((millis() - lastInput) > 10000)  // return to OF after 10 seconds idle
      {
         opState = OFF;
         return;
      }
      int tt = hop[HopPos];
      lcd.setCursor(0,1);
      lcd.print(tt);
      lcd.print(" minutes  ");
      DoControl();
   }
}



void Tune_TimerMargin()
{
   delay(200); // delay for buttons
   lcd.print(F("Timer margin"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();

      float increment = 0.10;
      if (buttons & BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons & BUTTON_LEFT)
      {
        //opState = OFF;
        //return;
        SaveParameters();
        return;
      }
      if (buttons & BUTTON_RIGHT)
      {
         opState = TUNE_P;
         return;
      }
      if (buttons & BUTTON_UP)
      {
         TimerMargin+= increment;
         delay(200);
      }
      if (buttons & BUTTON_DOWN)
      {
         TimerMargin -= increment;
         delay(200);
      }
      if ((millis() - lastInput) > 10000)  // return to RUN after 3 seconds idle
      {
         //opState = OFF;
         //return;
         SaveParameters();
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(TimerMargin);
      lcd.print(" ");
      DoControl();
   }
}

void TuneP()
{
   delay(200); // delay for buttons
   //lcd.setBacklight(TEAL);
   lcd.print(F("Set Kp"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();

      float increment = 1.0;
      if (buttons & BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons & BUTTON_LEFT)
      {
         opState = TIMERMARGIN;
         return;
      }
      if (buttons & BUTTON_RIGHT)
      {
         opState = TUNE_I;
         return;
      }
      if (buttons & BUTTON_UP)
      {
         Kp += increment;
         delay(200);
      }
      if (buttons & BUTTON_DOWN)
      {
         Kp -= increment;
         delay(200);
      }
      if ((millis() - lastInput) > 10000)  // return to RUN after 3 seconds idle
      {
         //opState = OFF;
         //return;
         SaveParameters();
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kp);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// RIGHT for Kd
// LEFT for Kp
// SHIFT for 10x tuning
// ************************************************
void TuneI()
{
   //lcd.setBacklight(TEAL);
   lcd.print(F("Set Ki"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();

      float increment = 0.01;
      if (buttons & BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons & BUTTON_LEFT)
      {
         opState = TUNE_P;
         return;
      }
      if (buttons & BUTTON_RIGHT)
      {
         opState = TUNE_D;
         return;
      }
      if (buttons & BUTTON_UP)
      {
         Ki += increment;
         delay(200);
      }
      if (buttons & BUTTON_DOWN)
      {
         Ki -= increment;
         delay(200);
      }
      if ((millis() - lastInput) > 10000)  // return to RUN after 3 seconds idle
      {
         //opState = OFF;
         //return;
         SaveParameters();
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Ki);
      lcd.print(" ");
      DoControl();
   }
}

void TuneD()
{
   //lcd.setBacklight(TEAL);
   lcd.print(F("Set Kd"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      float increment = 0.01;
      if (buttons & BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons & BUTTON_LEFT)
      {
         opState = TUNE_I;
         return;
      }
      if (buttons & BUTTON_RIGHT)
      {
         opState = SellSensor;
         return;
      }
      if (buttons & BUTTON_UP)
      {
         Kd += increment;
         delay(200);
      }
      if (buttons & BUTTON_DOWN)
      {
         Kd -= increment;
         delay(200);
      }
      if ((millis() - lastInput) > 10000)  // return to RUN after 3 seconds idle
      {
         //opState = OFF;
         //return;
         SaveParameters();
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kd);
      lcd.print(" ");
      DoControl();
   }
}


void SelectKettleSensor()
{
    int NumSens = sensors.getDeviceCount();
    //NumSens = 2;

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      float increment = 0.01;
      if (buttons & BUTTON_LEFT)
      {
         opState = TUNE_D;
         return;
      }
      if (buttons & BUTTON_RIGHT)
      {
         opState = SETALARM;
         return;
      }
      if (buttons & BUTTON_UP && NumSens >= 2)
      {
          lcd.setCursor(15, 0);
          lcd.write(4);
          lcd.setCursor(15, 1);
          lcd.print(" ");          
          IntKettleSensor = 1;
          StrKettleSensor = "1";

          //for (int i = 0; i < StrKettleSensor.length(); ++i)
          //{
            //EEPROM.write(41, StrKettleSensor[i]);
          //}
      }
      if (buttons & BUTTON_DOWN && NumSens >= 2)
      {
          lcd.setCursor(15, 1);
          lcd.write(4);
          lcd.setCursor(15, 0);
          lcd.print(" ");          
          IntKettleSensor = 2;
          StrKettleSensor = "2";

          //for (int i = 0; i < StrKettleSensor.length(); ++i)
          //{
            //EEPROM.write(41, StrKettleSensor[i]);
          //}
      }

    if (NumSens == 0)
    {
      lcd.setCursor(0,0);
      lcd.print(F("Sensor Error"));
    }

    if (NumSens == 1)
    {
      float temp1 = sensors.getTempC(tempSensor1);
      sensors.requestTemperatures(); 
      lcd.setCursor(0,0);
      lcd.print(temp1);

      IntKettleSensor = 1;
      lcd.setCursor(15, 0);
      lcd.write(4);

      IntKettleSensor = 1;
      StrKettleSensor = "1";

      //for (int i = 0; i < StrKettleSensor.length(); ++i)
      //{
      //  EEPROM.write(33, StrKettleSensor[i]);
      //}
    }

    if (NumSens >= 2)
    {
      float temp1 = sensors.getTempC(tempSensor1);
      float temp2 = sensors.getTempC(tempSensor2);
      sensors.requestTemperatures(); 
      lcd.setCursor(0,0);
      lcd.print(temp1);
      lcd.setCursor(0,1);
      lcd.print(temp2);

      if (IntKettleSensor == 1)
      {
        lcd.setCursor(15, 0);
        lcd.write(4);
      }

      if (IntKettleSensor == 2)
      {
        lcd.setCursor(15, 1);
        lcd.write(4);
      }
    }

      
      if ((millis() - lastInput) > 10000)  // return to RUN after 3 seconds idle
      {
         //opState = OFF;
         //return;
         SaveParameters();
         return;
      }
      DoControl();
   }
}

void SetAlarm()
{
   lcd.print(F("Set overheat"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      float increment = 1;
      if (buttons & BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons & BUTTON_LEFT)
      {
         opState = SellSensor;
         return;
      }
      if (buttons & BUTTON_RIGHT)
      {
         //opState = OFF;
         //return;
         SaveParameters();
         return;
      }
      if (buttons & BUTTON_UP)
      {
         AlarmSetpoint += increment;
         delay(200);
      }
      if (buttons & BUTTON_DOWN)
      {
         AlarmSetpoint -= increment;
         delay(200);
      }
      if ((millis() - lastInput) > 10000)  // return to RUN after 3 seconds idle
      {
         //opState = OFF;
         //return;
         SaveParameters();
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(AlarmSetpoint);
      lcd.print(" ");
      DoControl();
   }
}




// ************************************************
// PID COntrol State
// SHIFT and RIGHT for autotune
// RIGHT - Setpoint
// LEFT - OFF
// ************************************************
void Run()
{
  // 2 short beeps for starting program
  buzz(buzzerPin, NOTE_C7, 200, 2);

  AlarmRaised = 0;
  int IntTimerMargin = TimerMargin;

  int NumSens = sensors.getDeviceCount();
  int ShowSensor2 = 0;

  int TempReached = 0;
  if (stage == 0)
  {
    stage = 1;
    SpPos = 1;
    TmPos = 1;
    ElapsedMin = timer[TmPos];
    timeElapsed = 0;
    PIDsetpoint = Setpoint[SpPos];
  }
   // set up the LCD's number of rows and columns: 
   lcd.write(5);
   lcd.print(Setpoint[SpPos]);
   lcd.write(1);
   lcd.print(F("C"));

   myPID.SetTunings(Kp,Ki,Kd);

   uint8_t buttons = 0;
   while(true)
   {
      //setBacklight();  // set backlight based on state
      if (TempReached == 0)
      {
        lcd.setCursor(15, 0);
        lcd.print(" ");
      }

      buttons = ReadButtons();
      if ((buttons & BUTTON_SHIFT) && (buttons & BUTTON_RIGHT) && (abs(Input - Setpoint[SpPos]) < 0.5))  // Should be at steady-state
      {
         StartAutoTune();
      }
      else if ((buttons & BUTTON_LEFT) && (buttons & BUTTON_RIGHT) && ShowSensor2 == 1) 
      {
        ShowSensor2 = 0;
      }
      else if ((buttons & BUTTON_LEFT) && (buttons & BUTTON_RIGHT) && ShowSensor2 == 0) 
      {
        ShowSensor2 = 1;
      }
      else if (buttons & BUTTON_LEFT)
      {
        opState = OFF;
        return;
      }
      
      //int TempInput = Input;
      //int TempSetpoint = Setpoint[SpPos];
      float FloatInput = Input;
      float FloatSetpoint = Setpoint[SpPos];
      float FloatTimerMargin = TimerMargin;
      
      //if ( TempInput >= (TempSetpoint - IntTimerMargin) && TempReached == 0)
      if ( (FloatInput*1000) >= ( (FloatSetpoint*1000) - (FloatTimerMargin*1000) ) && TempReached == 0)
      {
        // first time temp reached. Set var and reset timer
        TempReached = 1;
        timeElapsed = 0;

        // display hourglass
        lcd.setCursor(15, 0);
        lcd.write(2);


        // Setpoint reached, 2 short beeps
        buzz(buzzerPin, NOTE_C7, 100, 2);
        //delay(100);
        //buzz(buzzerPin, NOTE_C7, 100);
        //delay(100);
      }
      

      //if (TempInput >= TempSetpoint || TempReached == 1)
      if ( (FloatInput*1000) >= (FloatSetpoint*1000) || TempReached == 1)
      {
        if (Setpoint[SpPos] < 100) { lcd.setCursor(9,0); }
        if (Setpoint[SpPos] >= 100) { lcd.setCursor(10,0); }
        lcd.print(ElapsedMin);
        lcd.print("m");

        if (timeElapsed > interval) 
        {
          ElapsedMin --;
          timeElapsed = 0;              // reset the counter to 0 so the counting starts over...
        }
      }
      if (ElapsedMin == 0)
        {
          if (Setpoint[SpPos] < 100) { lcd.setCursor(9,0); }
          if (Setpoint[SpPos] >= 100) { lcd.setCursor(10,0); }
          lcd.print("     ");
          if (stage == 6 || Setpoint[SpPos] == 100 || Setpoint == 0)
          {
            opState = OFF;
            return;
          }

          if (Setpoint[(SpPos+1)] != 0 && timer[(TmPos+1)] != 0 && stage <= 5)
          {
            stage++;
            SpPos++;
            TmPos++;
            TempReached = 0;
            PIDsetpoint = Setpoint[SpPos];
          }
          /*
          if (stage == 5)
          {
            ElapsedMin = timer[6];
            SpPos = 6;
            TmPos = 6;
            stage = 6;
            TempReached = 0;
            PIDsetpoint = Setpoint[6];
          }
          if (stage == 4)
          {
            ElapsedMin = timer[5];
            SpPos = 5;
            TmPos = 5;
            stage = 5;
            TempReached = 0;
            PIDsetpoint = Setpoint[5];
          }
          if (stage == 3)
          {
            ElapsedMin = timer[4];
            SpPos = 4;
            TmPos = 4;
            stage = 4;
            TempReached = 0;
            PIDsetpoint = Setpoint[4];
          }
          if (stage == 2)
          {
            ElapsedMin = timer[3];
            SpPos = 3;
            TmPos = 3;
            stage = 3;
            TempReached = 0;
            PIDsetpoint = Setpoint[3];
          }
          if (stage == 1)
          {
            ElapsedMin = timer[2];
            SpPos = 2;
            TmPos = 2;
            stage = 2;
            TempReached = 0;
            PIDsetpoint = Setpoint[2];
          }
          */
          lcd.setCursor(1,0);
          lcd.print(Setpoint[SpPos]);
        }

      if (Setpoint[SpPos] != 100)
      {
        DoControl();
      }
      if (Setpoint[SpPos] == 100)
      {
        digitalWrite(RelayPin,HIGH);
        // Read the input:
        if (sensors.isConversionAvailable(0))
        {
          if(IntKettleSensor == 1) Input = sensors.getTempC(tempSensor1);
          if(IntKettleSensor == 2) Input = sensors.getTempC(tempSensor2);
          sensors.requestTemperatures(); 
        }
      }

      lcd.setCursor(0,1);
      lcd.write(6);
      lcd.print(Input);
      lcd.write(1);
      lcd.print(F("C"));
      
        //lcd.setCursor(10,1);
        //lcd.print(ShowSensor2);
      if (ShowSensor2 == 0)
      {
        lcd.setCursor(9,1);
        lcd.print(F("      "));
      }
      if (ShowSensor2 == 1 && NumSens >= 2)
      {
        lcd.setCursor(9, 1);
        lcd.print(sensor2);
        //lcd.print(F("      "));
      }
      
      lcd.setCursor(14,0);
      if (tuning)
      {
        lcd.print("T");
      }
      else
      {
        lcd.print(" ");
      }
      
      int pinstate = digitalRead(RelayPin);
      if (pinstate == 1)
      {
          lcd.setCursor(15, 1);
          lcd.write(3);
      }
      else
      {
          lcd.setCursor(15, 1);
          lcd.print(" ");
      }
      CheckAlarm();
      if(AlarmRaised == 1)
      {
        opState = OFF;
        return;
      }


   }
}

void ShowTemp()
{
  int NumSens = sensors.getDeviceCount();
  //NumSens = 2;
  
  uint8_t buttons = 0;
  while(true)
  {
    buttons = ReadButtons();
    if(buttons & BUTTON_LEFT)
    {
      opState = OFF;
      return;
    }

    if (NumSens == 1)
    {
      if(IntKettleSensor == 1) { Input = sensors.getTempC(tempSensor1); }
      if(IntKettleSensor == 2) { Input = sensors.getTempC(tempSensor2); }
      sensors.requestTemperatures(); 
      lcd.setCursor(0,0);
      lcd.write(6);
      lcd.print(Input);
      lcd.write(1);
      lcd.print(F("C"));
    }

    if (NumSens >= 2)
    {
      if(IntKettleSensor == 1) { Input = sensors.getTempC(tempSensor1); }
      if(IntKettleSensor == 2) { Input = sensors.getTempC(tempSensor2); }
      sensors.requestTemperatures(); 
      lcd.setCursor(0,0);
      lcd.write(6);
      lcd.print(Input);
      lcd.write(1);
      lcd.print(F("C"));
      
      if(IntKettleSensor == 1) { Input = sensors.getTempC(tempSensor2); }
      if(IntKettleSensor == 2) { Input = sensors.getTempC(tempSensor1); }
      sensors.requestTemperatures(); 
      lcd.setCursor(0,1);
      lcd.write(3);
      lcd.print(Input);
      lcd.write(1);
      lcd.print(F("C"));
    }

    

  }
                 
      delay(100);
}


// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    if(IntKettleSensor == 1) { Input = sensors.getTempC(tempSensor1); sensor2 = sensors.getTempC(tempSensor2); }
    if(IntKettleSensor == 2) { Input = sensors.getTempC(tempSensor2); sensor2 = sensors.getTempC(tempSensor1); }
    sensors.requestTemperatures(); 
  }
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }

  if (Setpoint[SpPos] != 100) // We don't need PID during boil!
  {
    if((onTime > 100) && (onTime > (now - windowStartTime)))
    {
      digitalWrite(RelayPin,HIGH);
    }
    else
    {
      digitalWrite(RelayPin,LOW);
    }
  }
}

// ************************************************
// Set Backlight based on the state of control
// ************************************************
void setBacklight()
{
   if (tuning)
   {
      //lcd.setBacklight(VIOLET); // Tuning Mode
   }
   else if (abs(Input - Setpoint[SpPos]) > 1.0)  
   {
      //lcd.setBacklight(BLUE);  // High Alarm - off by more than 1 degree
   }
   else if (abs(Input - Setpoint[SpPos]) > 0.2)  
   {
      //lcd.setBacklight(YELLOW);  // Low Alarm - off by more than 0.2 degrees
   }
   else
   {
      //lcd.setBacklight(WHITE);  // We're on target!
   }
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
uint8_t ReadButtons()
{
  uint8_t buttons = lcd.readButtons();
  if (buttons != 0)
  {
    lastInput = millis();
  }
  return buttons;
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (AlarmSetpoint != EEPROM_readDouble(AlarmSpAddress))
   {
      EEPROM_writeDouble(AlarmSpAddress, AlarmSetpoint);
   }

   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
   if (TimerMargin != EEPROM_readDouble(TMarginAddress))
   {
      EEPROM_writeDouble(TMarginAddress, TimerMargin);
   }

    for (int i = 0; i < StrKettleSensor.length(); ++i)
    {
        EEPROM.write(41, StrKettleSensor[i]);
    }

   
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("     e.Brew"));
    lcd.setCursor(0,1);
    lcd.print(" Settings saved");
    delay(2000);  // Splash screen

    opState = OFF;
    //return;
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   AlarmSetpoint = EEPROM_readDouble(AlarmSpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   TimerMargin = EEPROM_readDouble(TMarginAddress);

  for (int i = 41; i < 42; ++i)
  {
    if (EEPROM.read(i) != 0) { StrKettleSensor += char(EEPROM.read(i));  }
  }
  if(StrKettleSensor == "")
  {
    IntKettleSensor = 1;
  }
  else
  {
    IntKettleSensor = StrKettleSensor.toInt();
  }   
   
   // Use defaults if EEPROM values are invalid
   if (isnan(AlarmSetpoint))
   {
      AlarmSetpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
   if (isnan(TimerMargin))
   {
     TimerMargin = 0.5;
   }  

}

void Reset()
{
    stage = 0;
    SpPos = 1;
    TmPos = 1;    
    HopPos = 1;    

    for (int i = 1; i < 7; ++i)
    {
      Setpoint[i] = 0;
    }

    for (int i = 1; i < 7; ++i)
    {
      timer[i] = 0;
    }

    for (int i = 1; i < 7; ++i)
    {
      hop[i] = 0;
    }

    lcd.setCursor(0,1);
    lcd.print(" Memory cleared");
    buzz(buzzerPin, NOTE_C7, 100,2);
    delay(3000);  // Splash screen
}

// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}

void buzz(int targetPin, long frequency, long length, int repeats) {
  for (int i = 0; i < repeats; ++i)
  {
    long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
    //// 1 second's worth of microseconds, divided by the frequency, then split in half since
    //// there are two phases to each cycle
    long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
    //// multiply frequency, which is really cycles per second, by the number of seconds to
    //// get the total number of cycles to produce
    for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
      digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
      delayMicroseconds(delayValue); // wait for the calculated delay value
      digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
      delayMicroseconds(delayValue); // wait again or the calculated delay value
    }
    delay(100);
  }
}

void CheckAlarm()
{
    uint8_t buttons = 0;
    float tTempInput;
    if(IntKettleSensor == 1) {  tTempInput = sensors.getTempC(tempSensor2); }
    if(IntKettleSensor == 2) {  tTempInput = sensors.getTempC(tempSensor1); }
    sensors.requestTemperatures();
    int TempInput = tTempInput;
    int ttAlarmSetpoint = AlarmSetpoint;

  if (TempInput >= ttAlarmSetpoint)
  {
      AlarmRaised = 1;
      digitalWrite(RelayPin, LOW);  // make sure it is off
      opState = OFF;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("!!!!Overheat!!!!"));
      lcd.setCursor(0, 1);
      lcd.print(F("down to reset"));

      while(!buttons)
      {
        buttons = ReadButtons();
        buzz(buzzerPin, NOTE_C7, 300,1);
        if (buttons == BUTTON_DOWN)
        {
          return;
        }
      }  
  }
}
