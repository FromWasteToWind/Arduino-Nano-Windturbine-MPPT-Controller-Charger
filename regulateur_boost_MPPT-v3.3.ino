/*
  Wind Turbine MPTT Regulator

  _________________________________________________________________
  |                                                               |
  |       author : Philippe de Craene <dcphilippe@yahoo.fr        |
  |       Free of use - Any feedback is welcome                   |
  _________________________________________________________________

  Materials :
  • 1* Arduino Uno R3 - IDE version 1.8.7
  • 2* 20A current sensor ACS712 modules
  • 1* Power MOSFET drivers SN754410
  • 1* LCD 1602 with I2C extension
  • 1 DC-DC boost converter shield: see manual.

  Arduino Uno pinup / with Atmega328p matching:
• input voltage sensor   : VpriPin   => input A0  = ADC0 / pin23
• output voltage sensor  : VsorPin   => input A1  = ADC1 / pin24
• input current sensor   : IpriPin   => input A2  = ADC2 / pin25
• battery current sensor : IbatPin   => input A3  = ADC3 / pin26
• SDA for I2C LCD        : SDA       => output A4 = ADC4 / pin27
• SCL for I2C LCD        : SCD       => output A5 = ADC5 / pin28
• wind turbine speed     : FpriPin   => input  2  = PD2 / pin4
• driving PWM signal     : gatePin   => output 3  = PD3 / pin5  - DC-DC converter driver signal
• dumpload signal        : loadPin   => output 4  = PD4 / pin6  - dumpload resistor
• inverter enable signal : ondulPin  => output 5  = PD5 / pin11 - blue LED + inverter
• enable SN754410 signal : enablePin => output 6  = PD6 / pin12 - enable the SN754410 driver
• overhead alarm         : alarmPin  => output 7  = PD7 / pin13 - red LED
• external charger       : ssrN_Pin  => output 8  = PB0 / pin14 - Normal output to SSR
• external charger       : ssrI_Pin  => output 9  = PB1 / pin15 - Inverted output to SSR
• "ok" push button       : pbE_Pin   => output 10 = PB2 / pin16
• "-" push button        : pbM_Pin   => output 11 = PB3 / pin17
• "+" push button        : pbP_Pin   => output 12 = PB4 / pin18
• MPPT indicator         : mpptPin   => output 13 = PB5 / pin19 - green LED

  Versions history :
  version 0.4 - 26 march 2019 - Fpri sensor rebuild with interrupt function
  version 0.5 - 27 march 2019 - Ipri sensor rebuild for average value
  version 0.6 - 26 april 2019 - MPPT algorithm rebuild without Fpri
  version 0.7 - 27 april 2019 - DC-DC converter rebuilt from buck-boost inverter to boost
  version 1.0 -  2 june 2019 - First full working version
  version 1.1 -  5 july 2019 - added EEPROM and menus
  version 1.2 -  5 july 2019 - improvement of the display of voltages
  version 1.3 - 13 july 2019 - improvement of security underload and overload and Ibat measure
  version 1.4 -  8 oct. 2019 - new LiquidCrystal-I2C-library and direct injection bug correction
  version 2.0 -  9 oct. 2019 - update for PCB
  version 2.1 -  8 dec. 2019 - bug correction for VpriMax and Dumpload
  version 2.3 -  3 jan. 2020 - VpriMax and VsorMax treatment improvement, SSR for external battery charger if available
  version 2.4 - 18 march 2020 - ext battery charger threshold voltages review
  version 3.0 - 11 june 2020 - TC428 replacement with SN754410 + leave of the "injection mode" specific parameters
  version 3.1 - 20 dec. 2020 - lastest updates from V2 circuit v2.54 : no li-ions option needed anymore : 
                               leave instead the 2 zener diodes in Vpri and Vsor sensor
  version 3.2 -  3 feb. 2021 - various improvents for better Fpri accuracy, thanks to algray.ak@gmail.com:
                               - replacement of LiquidCrystal for the faster hd44780
                               - minimize the number of lcd.print() in normal use
                               - put the Fri calculation in the ISR with micros() instead of millis() time measure
                               - modify the way to calculate analog values
  version 3.21 - 1 april 21  - red / green LED bug correction
  Version 3.22 - 9 nov 2021  - add #if
  Version 3.3  - 11 sept 22  - change R15 and R16 = 82K for Vpri and Vsor = VpriMaxRef = 46V maxi

*/

#define VERSION 3.30              // current version of code
//#define VERBOSE  1                // if uncommented : debugging mode => very slow !
//#define EEPROM_SET 1              // if uncommented : force EEPROM write default values
bool USAGE_FPRI = true;           // if true : the turbine speed is calculated

#include <Wire.h>                 // i2c communication
#include <EEPROM.h>               // EEPROM to keep redefined parameters data 
#include <hd44780.h>              // https://github.com/duinoWitchery/hd44780
#include <hd44780ioClass/hd44780_I2Cexp.h>  // i2c expander i/o class header

// Battery parameters (floats numbers)
//-------------------
// for LifePo4 battery pack:
float VsorMin = 24.8;       // discharged battery voltage : switch off the inverter (see battery datasheet for exact value)
float VsorFlo = 27.5;       // floating voltage (acid) or nominal voltage (lithium) (see battery datasheet for exact value)
float VsorMax = 28.5;       // maximum voltage (see battery datasheet for exact value)
int   IbatMax = 6;          // acid lead batteries : must be = 0,23 time the battery capacity => 13A for 54Ah batteries

// Wind turbine parameters
//------------------------
// voltage model : 12 / 24 / 48V
// VpriMaxRef is calculated to be about twice the optimal wind turbine voltage :
// a 24V wind turbine can reach 50V  => VpriMaxRef = 50.0V
// a 48V wind turbine can reach 100V => VpriMaxRef = 100.0V
int VpriMaxRef = 46;        // max range for a 24V wind turbine model
int VpriMin = 12;           // the voltage that will start the MPPT process. Too low the wind turbine may have difficulties to start

// Current sensor parameters for both ACS712, ACS724, etc:
//--------------------------------------------------------
// 5A ACS712 module  => 185mV/A
// 20A ACS712 module => 100mV/A
// 30A ACS712 module => 66mV/A
// 50A ACS724 module => 40mV/A
#define  convI  100.0             // for 20A model (float number is required)
// Depending of the way the ACS7xx module is wired, polarity adjustment may be required
// to get the charging batteries current positive, values can be 1 or -1
#define IbatPolarity  -1  
int Ioffset = 510;                // offset to get Ibat=0 with no current (~512)

// Other parameters
//-----------------
#define  pwm_gate_Max  200        // Max PWM value allowed (<250)
int Vpri_calibrate = 100;         // adjust Vpri to match real value with multimeter 100 = 100%
int Vsor_calibrate = 100;         // adjust Vsor to match real value with multimeter 100 = 100%

// inputs outputs declaration
//---------------------------
#define VpriPin  A0         // input to Vpri sensor
#define VsorPin  A1         // input to Vsor sensor
#define IpriPin  A2         // input to Ipri sensor 
#define IbatPin  A3         // input to Ibat sensor
#define FpriPin   2         // input to Fpri sensor
#define gatePin   3         // pwm output to drive the DC-DC converter circuit (pwm_gate)
#define loadPin   4         // output to dumpload resistor
#define ondulPin  5         // output inverter enabling + blue LED
#define enablePin 6         // output to enable SN754410 
#define alarmPin  7         // output to red LED
#define ssrN_Pin  8         // output for SSR for external battery charger
#define ssrI_Pin  9         // inverted output for external battery charger
#define pbE_Pin  10         // push-button for parameters access
#define pbM_Pin  11         // push-button -
#define pbP_Pin  12         // push-button +
#define mpptPin  13         // output to green LED

// variables for treatment
//------------------------
float Vpri, memo_Vpri, Vsor;         // input and output voltage
float Ipri, Ibat;                    // input and battery current
float Puiss = 0, memo_Puiss;         // input power
unsigned int Fpri = 0;               // turbine speed in Hertz
int kept_VpriMax = 0;                // Max measured Vpri for display
int kept_IpriMax = 0;                // Max measured Ipri for display
int kept_PuissMax = 0;               // Max calculated Puiss for display
int kept_FpriMax = 0;                // Max measured Fpri for display
unsigned int analogReadsCount = 0;   // number of analog measures
float somme_lect_Ipri = 0;           // Ipri measures added between two interrupts
float somme_lect_Ibat = 0;           // Ibat measures added between two interrupts
float somme_lect_Vpri = 0;           // Vpri measures added between two interrupts
float somme_lect_Vsor = 0;           // Vsor measures added between two interrupts
volatile bool Fpri_flag = false;     // Fpri flag interruption
volatile unsigned int Fpri_tempo = 0;
unsigned int memo_Fpri_tempo = 0, duration;  // time spent for Fpri measure
int Step = 0;                        // pwm ratio update for pwm_gate
int pwm_gate = 0;                    // pwm signal command for DC-DC converter
byte VsorMinInt, VsorMinDec, VsorFloInt, VsorFloDec, VsorMaxInt, VsorMaxDec;
bool extCharger = LOW;               // inverter enabled only if extCharger is LOW
bool okInverter = LOW;               // flag to allow inverter after the external charger stops

// variables for display and menus
//--------------------------------
unsigned int memo_tempo = millis();  // time flag when Fpri=0
unsigned int memo_tempo_LCD = 0;     // time flag for LCD refresh
unsigned int refresh_tempo = 1000;   // refresh delay for LCD update - > 1000 for 1 second
bool whichLine = HIGH;               // to reduce display time, print one line at a time
static char flt2str[8];              // used throughout for dtostrf() formatting of values to display
char lcdRow[17];                     // LCD line print string, 16 + 1 for end char
bool pbM, memo_pbM, pbP, memo_pbP;
byte ret_push_button = 0;
byte window = 0;
byte count_before_timeout = 0;
byte timeout = 20;

// LCD with I2C declaration => Arduino Uno R3 pinup : SDA to A4, SCL to A5
//LiquidCrystal_I2C lcd(0x27, 16, 2);
hd44780_I2Cexp lcd;                  // declare lcd object: auto locate & auto config expander chip
#define LCD_COLS  16                 // LCD geometry
#define LCD_ROWS   2


//
// SETUP
//_____________________________________________________________________________________________

void setup() {

// inputs outputs declaration
  pinMode(enablePin, OUTPUT); digitalWrite(enablePin, LOW);  // output to enable SN754410
  pinMode(gatePin,   OUTPUT); analogWrite(gatePin, 0);       // pwm output pwm_gate
  pinMode(loadPin,   OUTPUT); digitalWrite(loadPin, LOW);    // output for dumpload resistor
  pinMode(ondulPin,  OUTPUT); digitalWrite(ondulPin, LOW);   // output for enabling injection
  pinMode(ssrN_Pin,  OUTPUT); digitalWrite(ssrN_Pin, LOW);   // output for SSR for external battery charger
  pinMode(ssrI_Pin,  OUTPUT); digitalWrite(ssrI_Pin, HIGH);  // inverted output for external battery charger
  pinMode(mpptPin,   OUTPUT);       // output to green LED
  pinMode(alarmPin,  OUTPUT);       // output to red LED 
  pinMode(pbE_Pin, INPUT_PULLUP);   // push-button for menus acces
  pinMode(pbM_Pin, INPUT_PULLUP);   // push-button -
  pinMode(pbP_Pin, INPUT_PULLUP);   // push-button +
  pinMode(FpriPin, INPUT);          // input for Fpri sensor - turbine speed

// Console initialization
  #ifdef VERBOSE
   Serial.begin(250000);
   Serial.println();
   Serial.println(F("Ready to start..."));
  #endif

// LCD initialisation
  //lcd.begin();                // initialize the lcd for 16 chars 2 lines
  lcd.begin(LCD_COLS, LCD_ROWS);
  Wire.setClock(400000);      // Change clock speed from 100k(default) to 400kHz
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  Wind Turbine  ");
  lcd.setCursor(0, 1);
  lcd.print(" MPPT regulator ");
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("V");
  lcd.print(VERSION);
  lcd.print("          ");
  delay(1000);
  lcd.clear();

// EEPROM check and data upload :
// stored data are always positive from 0 to 255. it seems that in case of first use all are set to 255.
  VsorMinInt = VsorMin;
  VsorMinDec = 10 * (VsorMin - VsorMinInt);
  VsorFloInt = VsorFlo;
  VsorFloDec = 10 * (VsorFlo - VsorFloInt);
  VsorMaxInt = VsorMax;
  VsorMaxDec = 10 * (VsorMax - VsorMaxInt);
  #ifndef EEPROM_SET
   if(EEPROM.read(0) < 2)    USAGE_FPRI = EEPROM.read(0);  else EEPROM.write(0, USAGE_FPRI);
   if(EEPROM.read(3) < 51)   VpriMin =    EEPROM.read(3);  else EEPROM.write(3, VpriMin);
   if(EEPROM.read(4) < 41)   IbatMax =    EEPROM.read(4);  else EEPROM.write(4, IbatMax);
   if(EEPROM.read(5) < 65)   VsorMinInt = EEPROM.read(5);  else EEPROM.write(5, VsorMinInt);
   if(EEPROM.read(6) < 100)  VsorMinDec = EEPROM.read(6);  else EEPROM.write(6, VsorMinDec);
   if(EEPROM.read(7) < 65)   VsorFloInt = EEPROM.read(7);  else EEPROM.write(7, VsorFloInt);
   if(EEPROM.read(8) < 100)  VsorFloDec = EEPROM.read(8);  else EEPROM.write(8, VsorFloDec);
   if(EEPROM.read(9) < 131)  VsorMaxInt = EEPROM.read(9);  else EEPROM.write(9, VsorMaxInt);
   if(EEPROM.read(10) < 100) VsorMaxDec = EEPROM.read(10); else EEPROM.write(10, VsorMaxDec);
   VsorMin = 1.0 * VsorMinInt + (1.0 * VsorMinDec) / 10.0;
   VsorFlo = 1.0 * VsorFloInt + (1.0 * VsorFloDec) / 10.0;
   VsorMax = 1.0 * VsorMaxInt + (1.0 * VsorMaxDec) / 10.0;
   if(EEPROM.read(13) < 120) Vpri_calibrate = EEPROM.read(13); else EEPROM.write(13, Vpri_calibrate);
   if(EEPROM.read(14) < 120) Vsor_calibrate = EEPROM.read(14); else EEPROM.write(14, Vsor_calibrate);
   if(EEPROM.read(15) < 30)  Ioffset = EEPROM.read(15) + 500;  else EEPROM.write(15, (Ioffset - 500));
  #else
   EEPROM.write(0, USAGE_FPRI);
   EEPROM.write(3, VpriMin);
   EEPROM.write(4, IbatMax);
   EEPROM.write(5, VsorMinInt);
   EEPROM.write(6, VsorMinDec);
   EEPROM.write(7, VsorFloInt);
   EEPROM.write(8, VsorFloDec);
   EEPROM.write(9, VsorMaxInt);
   EEPROM.write(10, VsorMaxDec);
   EEPROM.write(13, Vpri_calibrate);
   EEPROM.write(14, Vsor_calibrate);
   EEPROM.write(15, (Ioffset - 500));
  #endif

// Set clock divider for timer 2 at 1 = PWM frequency of 31372.55 Hz
// Arduino Uno R3 pins 3 and 11
// https://etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
  TCCR2B = TCCR2B & 0b11111000 | 0x01;

  attachInterrupt(digitalPinToInterrupt(FpriPin), Fpri_detect, RISING);
// Every state update from down to up of FpripPin the function 'Fpri_detect' is called
// documentation : https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

// Enable the SN754410
  digitalWrite(enablePin, HIGH);
  #ifdef VERBOSE
   Serial.println("Drivers ready");
  #endif
}   // fin de setup

//
// LOOP
//_____________________________________________________________________________________________

void loop() {

  unsigned int tempo = millis();             // time count

// analog measures & overcurrent security
//_______________________________________________

  int lect_Ipri = analogRead(IpriPin);
  int lect_Ibat = analogRead(IbatPin);
  if((lect_Ipri < 2) || (lect_Ipri > 1021) || (lect_Ibat < 2) || (lect_Ibat > 1021)) {
    analogWrite(gatePin, 0);                 // no driving control to MPPT
    digitalWrite(ondulPin, LOW);             // swith off inverter
    digitalWrite(alarmPin, HIGH);
    return;                                  // nothing else is done
  }

  // cumulative measures between 2 interrupts = one turbine coil rotation
  // bits to volt convertion: divide by 1023 or 1024?  see bottom of https://www.gammon.com.au/adc
  // float voltage = ((float) myRead  + 0.5 ) / 1024.0 * vRef;
  somme_lect_Ipri += (float)lect_Ipri + 0.5;
  somme_lect_Ibat += (float)lect_Ibat + 0.5;

  int lect_Vpri = analogRead(VpriPin);
  somme_lect_Vpri += (float)lect_Vpri + 0.5;

  int lect_Vsor = analogRead(VsorPin);
  somme_lect_Vsor += (float)lect_Vsor + 0.5;

  analogReadsCount++;

// Every turbine period, or every second if no wind, or every 50ms if Fpri not measured
//_______________________________________________

  if((USAGE_FPRI && (Fpri_flag || ((tempo - memo_tempo) > 100)))
      || (!USAGE_FPRI && ((tempo - memo_tempo) > 50))) {
      // note about elapsed time measures: http://www.thetaeng.com/designIdeas/TimerWrap.html 
    noInterrupts();                     // disable any possible interruption
    Fpri_flag = false;                  // flag reset, will be ready to be set at next interrupt
    memo_tempo = tempo;
    memo_Puiss = Puiss;                 // memorization of the previous Puiss measurement
    memo_Vpri = Vpri;                   // memorization of the previous Vpri measurement

// Measures and calculation of power values
//_______________________________________________

//  time spent between 2 interrupts => Fpri calculation
    duration = Fpri_tempo - memo_Fpri_tempo;  // time spent between 2 interrupts
    memo_Fpri_tempo = Fpri_tempo;             // memorization of the previous time measurement
    if( duration < 100 ) Fpri = 0;            // ignore if less than 10ms
    else if( duration < 10001 ) Fpri = 10000 / duration;  // in Hertz = number of turbine pole changes/seconde
    else Fpri = 0;                            // set Fpri = 0 if delay over 1s

    // analogRead measures are in bits : from 0 to 1023, to convert to :
    // -> voltage from 0 to VpriMaxRef for Vpri and Vsor
    // -> current : 511 in bits is the 0mA, 0 in bits matches -Imax, 1023 matches +Imax
    Vpri = (somme_lect_Vpri / (float)analogReadsCount / 1024.0) * VpriMaxRef * (Vpri_calibrate / 100.0);
    Vsor = (somme_lect_Vsor / (float)analogReadsCount / 1024.0) * VpriMaxRef * (Vsor_calibrate / 100.0);
    Ipri = ((somme_lect_Ipri / (float)analogReadsCount) - (float)Ioffset) * 5000.0 / convI / 1024.0;
    if( Ipri < 0 ) Ipri = -Ipri;                // to be sure to get a positive value despite the way of wiring
    Ibat = ((somme_lect_Ibat / (float)analogReadsCount) - (float)Ioffset) * 5000.0 / convI / 1024.0;
    if( IbatPolarity ) Ibat = -Ibat;
    somme_lect_Ipri = 0;
    somme_lect_Ibat = 0;
    somme_lect_Vpri = 0;
    somme_lect_Vsor = 0;
    analogReadsCount = 0;

    Puiss = Vpri * Ipri;

    // keep the maximum measured values for display only
    if( Vpri > kept_VpriMax ) kept_VpriMax = Vpri;
    if( Ipri > kept_IpriMax ) kept_IpriMax = Ipri;
    if( Puiss > kept_PuissMax ) kept_PuissMax = Puiss;
    if( Fpri > kept_FpriMax && Fpri < 1000 ) kept_FpriMax = Fpri;

// setup of the MPPT algorithm
//_______________________________________________

// 2 ways that make a lower power :
// either the wind turbine runs too fast, Vpri increases, so 'step' increases to increase the current (and so Vpri may decrease)
// either less wind, Vpri decreases, so 'step' decreases to decreases the current (and so Vpri may increase)

    if( memo_Vpri <= Vpri ) {
      if( Puiss <= memo_Puiss ) Step = 1;
      else Step = -1;
    }
    else {
      if( Puiss <= memo_Puiss ) Step = -1;
      else Step = 1;
    }

// Management of DC-DC converter cutting control
//_______________________________________________

    if( Vsor < VsorMin ) {
      digitalWrite(ondulPin, LOW);     // stop the inverter
      Step = 1;                        // try to increase Vsor
    }    // end of test Vsor < VsorMin

    if( Vpri < VpriMin ) {             // lower limit input voltage value reached
      Step = -1;                       // try to increase Vpri
      digitalWrite(mpptPin, LOW);      // green LED is OFF
    }    // end of test Vpri < VpriMin
    else {
      digitalWrite(mpptPin, HIGH);     // display MPPT is working
      if( extCharger ) {               // stop ext charger if ON
        digitalWrite(ssrN_Pin, LOW);   // output for SSR inactive for external battery charger
        digitalWrite(ssrI_Pin, HIGH);  // inverted output inactive for external battery charger
        extCharger = false;            // that will allow inverter activation in display timeout cycle
        okInverter = true;             // allow the inverter
      }  // end of test extCharger
    }    // end of test Vpri > VpriMin

    if(( Vsor > VsorFlo ) && okInverter ) digitalWrite(ondulPin, HIGH);  // start the inverter

// Overcharge security & pwm ratio update
//_______________________________________________

    if( Ibat > IbatMax ) Step = -1;           // decrease pwm

    if( Vsor > VsorMax ) {                    // when VsorMax is reached
      Step = -1;                              // decrease pwm ratio to decrease Vsor
      digitalWrite(alarmPin, HIGH);           // red LED is ON
      digitalWrite(loadPin, HIGH);            // dumpload is ON
    }
    else {
      digitalWrite(loadPin, LOW);             // dumpload is OFF
      digitalWrite(alarmPin, LOW);            // red LED is OFF
    }

// constrain the pwm ratio
    pwm_gate += Step;
    if( pwm_gate > pwm_gate_Max ) pwm_gate = pwm_gate_Max;  // high value limit
    else if( pwm_gate < 0 )  pwm_gate = 0;                  // low value limit

    analogWrite(gatePin, pwm_gate);       // cutting command update before any dumpload evaluation
    interrupts();                         // interrupts enable again
// for debugging purpose only
    #ifdef VERBOSE
      Serial.print("Fpri= ");               Serial.print(Fpri);
      Serial.print("  Vpri= ");             Serial.print(Vpri);
      Serial.print("  Ipri= ");             Serial.print(Ipri);
      //Serial.print("  Puiss= ");            Serial.print(Puiss);
      //Serial.print("  Puiss-memo_Puiss= "); Serial.print(Puiss - memo_Puiss);
      Serial.print("  Step : ");            Serial.print(Step);
      Serial.print("  pwm_gate : ");        Serial.print(pwm_gate);
      Serial.print("  Vsor= ");             Serial.print(Vsor);
      Serial.print("  Ibat= ");             Serial.print(Ibat);
      Serial.println();
    #endif
  }   // end of Fpri 1 period cycle

// LCD and menus management + ext battery charger
//_______________________________________________

// every 'refresh_tempo' have a look for push-button activity and update display, and setup external charger
  if( tempo - memo_tempo_LCD > refresh_tempo ) {
    memo_tempo_LCD = tempo;
    ret_push_button = push_button();        // reading push-button status here only
    count_before_timeout++;

// external charger & display timeout management
    if( count_before_timeout > timeout ) {  // no activity timeout
      if( !extCharger ) okInverter = true;  // ext charger is off, after a timeout delay the inverter starts
      else if( Vsor > VsorFlo ) {
        digitalWrite(ssrN_Pin, LOW);        // output for SSR inactive for external battery charger
        digitalWrite(ssrI_Pin, HIGH);       // inverted output inactive for external battery charger
        extCharger = false;                 // that will allow inverter activation next timeout cycle
      }
      if( Vsor < VsorMin ) {
        digitalWrite(ssrN_Pin, HIGH);       // output for SSR active for external battery charger
        digitalWrite(ssrI_Pin, LOW);        // inverted output active for external battery charger
        okInverter = false;                 // prevent later againt any inverter activation
        extCharger = true;                  // flag that ext cahrged is activated
      }
      count_before_timeout = 0;             // reset the timeout counter
      window = 0;                           // return to first display
      lcd.clear();
      lcd.noBacklight();                    // light off display
    }

// push button management
    if( ret_push_button == 1 ) {
      if ( window != 4 ) next_window();
      else window = 0;
    }

// usual display with 2 choises : window 0 and window 1
    if( window < 2 ) {
      whichLine = !whichLine;
      if(whichLine) {
      // first line display only / limit the number of lcd.print() to gain time
        lcd.setCursor(0, 0);
        // usage : dtostrf( number_value, number_of_digits, nulber_of_decimal, char_output)
        dtostrf(Vpri, 4, 1, flt2str);
        strcpy(lcdRow, "Ve="); strcat(lcdRow, flt2str); strcat(lcdRow, "  Vs=");
        dtostrf(Vsor, 4, 1, flt2str);
        strcat(lcdRow, flt2str);
      }
      else {
      // second line display
        lcd.setCursor(0, 1);
        if( window == 0 ) {
          if( USAGE_FPRI ) {
            dtostrf(Fpri, 3, 0, flt2str);
            strcpy(lcdRow, flt2str); strcat(lcdRow, "Hz  Pe=");
          }
          else strcpy(lcdRow, "-----  Pe=");
          dtostrf(Puiss, 6, 1, flt2str);
          strcat(lcdRow, flt2str);
          
        }
        else if( window == 1 ) {
          dtostrf(Ipri, 4, 1, flt2str);
          strcpy(lcdRow, "Ie="); strcat(lcdRow, flt2str);
          dtostrf(Ibat, 4, 1, flt2str);
          strcat(lcdRow, "  Ib="); strcat(lcdRow, flt2str);
        }
      }
      lcd.print(lcdRow);
    }    // end of usual display
    else {

// if window >= 2 we are entering in max values display and parameters setup

      if( window == 2 ) {
        dtostrf(kept_VpriMax, 3, 0, flt2str);
        strcpy(lcdRow, "VM="); strcat(lcdRow, flt2str);
        dtostrf(kept_IpriMax, 3, 0, flt2str);
        strcat(lcdRow, "    IM="); strcat(lcdRow, flt2str);
        lcd.print(lcdRow);
        lcd.setCursor(0, 1);
        if ( USAGE_FPRI ) {
          dtostrf(kept_FpriMax, 3, 0, flt2str);
          strcpy(lcdRow, flt2str); strcat(lcdRow, "Hz  Pe=");
        }
        else strcpy(lcdRow, "-----  Pe=");
        dtostrf(kept_PuissMax, 6, 1, flt2str);
        strcat(lcdRow, flt2str);
        lcd.print(lcdRow);
      }  // end of window 2

      if( window == 3 ) {
        lcd.print("Reset MAX val. ?");
        lcd.setCursor(0, 1);
        lcd.print("push + to reset ");
        if (ret_push_button == 2) {
          kept_VpriMax = 0;
          kept_IpriMax = 0;
          kept_PuissMax = 0;
          kept_FpriMax = 0;
          lcd.setCursor(0, 1);
          lcd.print("values reseted  ");
          window = 2;
        }
      }  // end of window 3

      if( window == 4 ) {
        lcd.print("Parameters setup");
        lcd.setCursor(0, 1);
        lcd.print("push + to review");
        if ( ret_push_button > 1 ) next_window();
      }   // end of wondows 4

      if( window == 5 ) {
        if( ret_push_button > 1 ) USAGE_FPRI = ! USAGE_FPRI;
        lcd.print("Turbine speed = ");
        lcd.setCursor(0, 1);
        if ( USAGE_FPRI ) lcd.print("measured        ");
        else              lcd.print("not measured    ");
      }   // end of window 5

      if( window == 6 ) {
        if(ret_push_button == 2) VpriMin++;
        if(ret_push_button == 3) VpriMin--;
        VpriMin = constrain(VpriMin, 4, 80);
        lcd.print("Input: V MINI   ");
        lcd.setCursor(0, 1);
        dtostrf(VpriMin, 4, 1, flt2str);
        strcpy(lcdRow, "VpriMin = "); strcat(lcdRow, flt2str); strcat(lcdRow, "V ");
        lcd.print(lcdRow);
      }   // end of wondows 6

      if( window == 7 ) {
        if(ret_push_button == 2) IbatMax++;
        if(ret_push_button == 3) IbatMax--;
        IbatMax = constrain(IbatMax, 4, 30);
        lcd.print("Bat: I charg MAX");
        lcd.setCursor(0, 1);
        dtostrf(IbatMax, 3, 0, flt2str);
        strcpy(lcdRow, "IbatMax = "); strcat(lcdRow, flt2str); strcat(lcdRow, "A  ");
        lcd.print(lcdRow);
      }  // end of wondows 7

      if( window == 8 ) {
        if(ret_push_button == 2) VsorMin = VsorMin + 0.1;
        if(ret_push_button == 3) VsorMin = VsorMin - 0.1;
        VsorMinInt = VsorMin;
        VsorMinDec = 10 * (VsorMin - VsorMinInt);
        lcd.print("Bat. V MINIMAL  ");
        lcd.setCursor(0, 1);
        dtostrf(VsorMin, 4, 1, flt2str);
        strcpy(lcdRow, "VsorMin = "); strcat(lcdRow, flt2str); strcat(lcdRow, "V ");
        lcd.print(lcdRow);
      }  // end of window 8

      if( window == 9 ) {
        if(ret_push_button == 2) VsorFlo = VsorFlo + 0.1;
        if(ret_push_button == 3) VsorFlo = VsorFlo - 0.1;
        VsorFloInt = VsorFlo;
        VsorFloDec = 10 * (VsorFlo - VsorFloInt);
        lcd.print("Bat: V NOMINAL  ");
        lcd.setCursor(0, 1);
        dtostrf(VsorFlo, 4, 1, flt2str);
        strcpy(lcdRow, "VsorFlo = "); strcat(lcdRow, flt2str); strcat(lcdRow, "V ");
        lcd.print(lcdRow);
      }  // end of window 9

      if( window == 10 ) {
        if(ret_push_button == 2) VsorMax = VsorMax + 0.1;
        if(ret_push_button == 3) VsorMax = VsorMax - 0.1;
        VsorMaxInt = VsorMax;
        VsorMaxDec = 10 * (VsorMax - VsorMaxInt);
        lcd.print("Bat: V MAXIMAL  ");
        lcd.setCursor(0, 1);
        dtostrf(VsorMax, 4, 1, flt2str);
        strcpy(lcdRow, "VsorMax = "); strcat(lcdRow, flt2str); strcat(lcdRow, "V ");
        lcd.print(lcdRow);
      }  // end of window 10

      if( window == 11 ) {
        if(ret_push_button == 2) Vpri_calibrate++;
        if(ret_push_button == 3) Vpri_calibrate--;
        lcd.print("Ve calibrate    ");
        lcd.setCursor(0, 1);
        dtostrf(Vpri, 4, 1, flt2str);
        strcpy(lcdRow, "-/+ modify: "); strcat(lcdRow, flt2str);
        lcd.print(lcdRow);
      }  // end of window 11

      if( window == 12 ) {
        if(ret_push_button == 2) Vsor_calibrate++;
        if(ret_push_button == 3) Vsor_calibrate--;
        lcd.print("Bat: V calibrate");
        lcd.setCursor(0, 1);
        dtostrf(Vsor, 4, 1, flt2str);
        strcpy(lcdRow, "-/+ modify: "); strcat(lcdRow, flt2str);
        lcd.print(lcdRow);
      }  // end of window 12

      if( window == 13 ) {
        if(ret_push_button == 2) Ioffset++;
        if(ret_push_button == 3) Ioffset--;
        Ioffset = constrain( Ioffset, 500, 524 );
        lcd.print("Ib=0 calibrate  ");
        lcd.setCursor(0, 1);
        dtostrf(Ioffset, 3, 0, flt2str);
        strcpy(lcdRow, flt2str); strcat(lcdRow, "  ->   ");
        dtostrf(Ibat, 6, 2, flt2str);
        strcat(lcdRow, flt2str);
        lcd.print(lcdRow);
      }  // end of window 13

      if( window == 14 ) {
        lcd.print("Debug mode view ");
        lcd.setCursor(0, 1);
        dtostrf(Step, 3, 0, flt2str);
        strcpy(lcdRow, "s: "); strcat(lcdRow, flt2str);
        dtostrf(pwm_gate, 4, 0, flt2str);
        strcat(lcdRow, "   p: "); strcat(lcdRow, flt2str);
        lcd.print(lcdRow);
        count_before_timeout = 0;             // no timeout in debug mode
      }  // end of window 14

// EEPROM updated if needed
      EEPROM.update(0, USAGE_FPRI);
      EEPROM.update(3, VpriMin);
      EEPROM.update(4, IbatMax);
      EEPROM.update(5, VsorMinInt);
      EEPROM.update(6, VsorMinDec);
      EEPROM.update(7, VsorFloInt);
      EEPROM.update(8, VsorFloDec);
      EEPROM.update(9, VsorMaxInt);
      EEPROM.update(10, VsorMaxDec);
      EEPROM.update(13, Vpri_calibrate);
      EEPROM.update(14, Vsor_calibrate);
      EEPROM.update(15, (Ioffset - 500));
    }    // end of parameters review
  }      // end of LCD display
}        // end of loop

//============================================================================================
// list of functions
//============================================================================================

//
// Fpri_detect : what is done at each interruption
//____________________________________________________________________________________________

void Fpri_detect() {
  // everything about interruption: http://www.gammon.com.au/interrupts
  Fpri_tempo = micros()/100;        // précision of 0.1 ms 
  Fpri_flag = true;
}

//
// NEXT_WINDOW : next window procedure
//____________________________________________________________________________________________

void next_window() {

  window = (window + 1) % 15;       // next window - total number of windows to review +1
  ret_push_button = 0;              // reset the buttun state
  lcd.setCursor(0, 0);
}     // end of next_window function

//
// PUSH_BUTTON : return value depending of the state of the 3 push-buttons
//____________________________________________________________________________________________

byte push_button() {

  memo_pbM = pbM; memo_pbP = pbP;     // memorization for past state of + - push-button
  pbP = digitalRead(pbP_Pin);
  pbM = digitalRead(pbM_Pin);

  if(!digitalRead(pbE_Pin)) {
    count_before_timeout = 0;         // reset the timeout counter
    refresh_tempo = 1000;
    lcd.backlight();                  // switch on display
    return 1;
  }
  if(!pbP) {
    count_before_timeout = 0;           // reset the timeout counter
    if(!memo_pbP) refresh_tempo = 300;  // temporary lower display update duration
    lcd.backlight();                    // switch on display
    return 2;
  }
  if(!pbM) {
    count_before_timeout = 0;           // reset the timeout counter
    if(!memo_pbM) refresh_tempo = 300;  // temporary lower display update duration
    lcd.backlight();                    // switch on display
    return 3;
  }
  refresh_tempo = 1000;              // return back to usual display update duration
  return 0;
}     // end of push_button function
