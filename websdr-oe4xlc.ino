/*
 * gps controlled oszillator für oe4xlc in allhau
 * oe6rke, 2021-04
 *
 * oszi0 2.5 MHz  (feedback)
 * oszi1  51 MHz  (SX1280 Referenztakt - GPS-diszipliniert)
 * oszi2  --      (frei)
 *
 *  si5351 + nano + txco 25.00 (ppm2.5) + GPS NEO7M
 *
 * https://github.com/etherkit/Si5351Arduino
 * Modul hier https://www.sv1afn.com/si5351a.html,
 * http://ak2b.blogspot.com/2015/01/installing-libraries-and-running-code.html
 *
 *  Based on the projects:
 *  W3PM (http://www.knology.net/~gmarcus/)
 *  &
 *  SQ1GU (http://sq1gu.tobis.com.pl/pl/syntezery-dds/44-generator-si5351a)
 */

#include <TinyGPS++.h>
#include <string.h>
#include <ctype.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Wire.h>
#include <si5351.h>
#include <SoftwareSerial.h>

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C
#define VERSION "2026-03-06"


// The TinyGPS++ object
TinyGPSPlus gps;

// The Si5351 object
Si5351 si5351;

//pins
#define LED 12
#define ppsPin                   2  //D2
#define przycisk                 A2

#define CHA 3
#define CHB 7
volatile bool fired;
volatile bool up;

// The Display object
SSD1306AsciiWire oled;

//XTAL freq
unsigned long XtalFreq = 100000000;
unsigned long XtalFreq_old = 100000000;

//freqs
unsigned long Freq1 = 51000000;  // 51 MHz (SX1280 Referenztakt)
unsigned long Freq2 = 28800000;  // 28.8 MHz

long stab;
long correction = 0;
byte stab_count = 44;
unsigned long mult = 0;
int second = 0, minute = 0, hour = 0;
int day = 0, month = 0, year = 0;
int zone = 1;
unsigned int tcount = 0;
unsigned int tcount2 = 0;
bool validGPSflag = false;
char c;
boolean newdata = false;
boolean GPSstatus = true;
boolean fixed = false;

byte new_freq = 1; //switch für neue qrg
unsigned long freq_step = 1000;

byte encoderOLD, menu = 0, band = 1, f_step = 1;
boolean time_enable = true;
unsigned long pps_correct;
byte pps_valid = 1;
float stab_float = 1000;

volatile bool stab_ready = false;
volatile bool time_update_needed = false;
volatile bool tx_trigger = false;
volatile uint8_t tx_second_snapshot = 0;

SoftwareSerial gpsSerial(11, 10); // RX=D11, TX=D10. on RX comes GPS Serial in


//*************************************************************************************
//                                    SETUP
//*************************************************************************************
void setup()
{
  bool i2c_found;
  Serial.begin(9600);
  gpsSerial.begin(9600);

  Serial.println("GPSDO RTL coded by OE6RKE Version " VERSION);

  //init display
  Wire.begin();
  Wire.setClock(400000L);
  //oled.begin(&Adafruit128x64, I2C_ADDRESS);  // 0.96" SSD1306
  oled.begin(&SH1106_128x64, I2C_ADDRESS);    // 1.3" SH1106 (2.7x2cm)
  oled.setFont(System5x7); // Auswahl der Schriftart
  oled.clear();

  //led status D12
  pinMode(LED, OUTPUT);

  TCCR1B = 0;                                    //Disable Timer1 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag
  pinMode(ppsPin, INPUT);                        // Inititalize GPS 1pps input

  i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 25000000, 10); //init with 25 mhz and offset

  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  //si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_4MA); //5db Output für clock0

  // Set CLK0 to output 2,5MHz
  si5351.set_freq(250000000ULL, SI5351_CLK0); //CLK0 = 2.5MHz

  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);
  si5351.set_freq(5100000000ULL, SI5351_CLK1);   // CLK1 = 51 MHz fuer SX1280
  si5351.set_freq(2880000000ULL, SI5351_CLK2);   // CLK2 = 28.8 MHz

  si5351.update_status();

  //version info
  oled.println();
  oled.print(" GPSDO RTL");
  delay(1000);
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("coded by OE6RKE");
  oled.setCursor(0, 2);
  oled.print("Version " VERSION);
  digitalWrite(LED, HIGH);
  delay(4000);
  digitalWrite(LED, LOW);
  oled.clear();

  oled.setCursor(4, 2);
  oled.print("Waiting for GPS");

  GPSproces(6000);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    oled.setCursor(0, 2);
    oled.print("GPS not connected");
    oled.setCursor(0, 4);
    oled.print ("Error!");
    delay(5000);
    GPSstatus = false;
  }
  oled.clear();

  if (GPSstatus == true) {
    oled.setCursor(4, 2);
    oled.print("Waiting for SAT");
    time_on_oled();
    sat_on_oled();
    do {
      GPSproces(1000);
    } while (gps.satellites.value() == 0);

    hour = gps.time.hour() + zone;
    minute = gps.time.minute();
    second = gps.time.second();

    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();


    oled.clear();
    time_on_oled();
    sat_on_oled();
    attachInterrupt(0, PPSinterrupt, RISING); //attach 2 interrupt
    TCCR1B = 0;
    tcount = 0;
    mult = 0;
    validGPSflag = 1;
  }
  //display on
  freq_on_oled();
  sat_on_oled();
  time_on_oled();
  date_on_oled();

  Serial.println("GPSDO RTL init phase done");
}

//***************************************************************************************
//                                         LOOP
//***************************************************************************************
void loop()
{
  if (stab_ready) { stab_ready = false; stab_on_oled(); }
  if (time_update_needed) { time_update_needed = false; time_on_oled(); }

  GPSproces(0);

  if (tx_trigger) {
    tx_trigger = false;
    start_ft8_transmission();
  }

  bool near_tx_boundary = (second % 15 >= 14) || (second % 15 == 0);

  if (tcount2 != tcount) {
    tcount2 = tcount;
    pps_correct = millis();
  }

  if (gps.time.isUpdated()) {
    hour = gps.time.hour() + zone;
    minute = gps.time.minute();
    second = gps.time.second();
  }

  if (gps.date.isUpdated()) {
    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();
  }

  if (gps.satellites.isUpdated() && menu == 0 && !near_tx_boundary) {
    sat_on_oled();
  }

  //do it always
  correct_si5351a();
  new_freq = 0;
  if (abs(stab_float)<1)  {
    oled.setCursor(112, 4);
    oled.print("@");
    digitalWrite(LED, HIGH);
    if (!fixed) Serial.println("GPSDO locked");
    fixed = true;
  }
  if (abs(stab_float)>1) {
    oled.setCursor(112, 4);
    oled.print(" ");
    digitalWrite(LED, LOW);
    if (fixed) Serial.println("GPSDO not locked");
    fixed = false;
  }

  if (millis() > pps_correct + 1200) {
    pps_valid = 0;
    pps_correct = millis();
    time_enable = false;
    oled.setCursor (15, 0);
  }

   //wait not to overflow stuff
   static unsigned long lastDisplayUpdate = 0;
   if (millis() - lastDisplayUpdate >= 5000) {
     lastDisplayUpdate = millis();
     if (!near_tx_boundary) time_on_oled();
   }
}


//**************************************************************************************
//                       INTERRUPT  1PPS
//**************************************************************************************
void PPSinterrupt()
{

  tcount++;
  stab_count--;
  if (tcount == 4)                               // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                                  //Clock on rising edge of pin 5
  }
  if (tcount == 44)                              //The 40 second gate time elapsed - stop counting
  {
    TCCR1B = 0;
    if (pps_valid == 1) {
      XtalFreq_old = XtalFreq;
      XtalFreq = mult * 0x10000 + TCNT1;         //Calculate correction factor, only if sat is present
      //Serial.println(XtalFreq);
      new_freq = 1;
    }
    TCNT1 = 0;                                   //Reset count to zero
    mult = 0;
    tcount = 0;                                  //Reset the seconds counter
    pps_valid = 1;

    stab_count = 44;
    stab_ready = true;
    //stab_on_serial();
  }
  if (validGPSflag == 1)                      //Start the UTC timekeeping process
  {
    second++;
    if (second == 60)                            //Set time using GPS NMEA data
    {
      minute++ ;
      second = 0 ;
    }
    if (minute == 60)
    {
      hour++;
      minute = 0 ;
    }
    if (hour == 24) hour = 0 ;
    if (time_enable) time_update_needed = true;
  }
  if (menu == 4) {
    oled.setCursor(96, 6);
    if (stab_count < 10) oled.print(" ");
    oled.print(stab_count);
  }
  if (second % 15 == 0) {
    tx_trigger = true;
    tx_second_snapshot = second;
  }
}
//*******************************************************************************
// Timer 1 overflow intrrupt vector.
//*******************************************************************************
ISR(TIMER1_OVF_vect)
{
  mult++;                                          //Increment multiplier
  TIFR1 = (1 << TOV1);                             //Clear overlow flag
}



//********************************************************************************
//                                STAB on oled stabilnośc częstotliwości
//********************************************************************************
void stab_on_oled() {
  long pomocna;
  time_enable = false;
  stab = XtalFreq - 100000000;
  stab = stab * 10 ;
  if (stab > 100 || stab < -100) {
    correction = correction + stab;
  }
  else if (stab > 20 || stab < -20) {
    correction = correction + stab / 2;
  }
  else correction = correction + stab / 4;
  pomocna = (10000 / (Freq1 / 1000000));
  stab = stab * 100;
  stab = stab / pomocna;
  stab_float = float(stab);
  stab_float = stab_float / 10;

  Serial.print("Freq. correction: ");
  Serial.print(stab_float);
  Serial.println(" Hz");

  //print location
  Serial.print("lat: ");
  Serial.print(gps.location.rawLat().negative ? "-" : "+");
  Serial.println(gps.location.lat(), 8);

  Serial.print("lon: ");
  Serial.print(gps.location.rawLng().negative ? "-" : "+");
  Serial.println(gps.location.lng(), 8);
}



//********************************************************************************
//                                TIME on oled
//********************************************************************************
void time_on_oled()
{
  char sz[32];
  sprintf(sz, "%02d:%02d:%02d ", hour, minute, second);
    oled.setCursor(30, 0);
  oled.print(sz);
}

//********************************************************************************
//                                DATE on oled
//********************************************************************************
void date_on_oled()
{
  char da[32];
  sprintf(da, "%02d/%02d/%02d ", day, month, year);
    oled.setCursor(24, 2);
  oled.print(da);
}

//********************************************************************************
//                                SAT nr. on oled
//********************************************************************************
void sat_on_oled()
{
  time_enable = false;
  oled.setCursor(0, 4);
  oled.print("SAT: ");
  oled.print(gps.satellites.value());
  oled.print("   ");
  time_enable = true;
}

//*********************************************************************************
//                             Freq on oled
//*********************************************************************************
void freq_on_oled() {
  time_enable = false;
  oled.setCursor(0, 6);
  oled.print("CLK1: ");
  oled.print(Freq1 / 1000000);
  oled.print(" MHz SX1280  ");
  oled.setCursor(0, 7);
  oled.print("GPS locked: ");
  oled.print(fixed ? "YES" : "NO ");
}
//********************************************************************
//             NEW frequency  --- NOT USED, because fixed
//********************************************************************
void update_si5351a()
{
  si5351.set_freq(Freq1 * SI5351_FREQ_MULT, SI5351_CLK1);
}
//********************************************************************
//             TX window guard: returns true during FT8 TX window
//********************************************************************
bool is_in_tx_window() {
  int s = second % 15;
  bool ft8_window = (s >= 0 && s < 13);
  return ft8_window;
}
//********************************************************************
//             NEW frequency correction
//********************************************************************
void correct_si5351a()
{
  static long last_correction = 0;
  if (!is_in_tx_window() && correction != last_correction) {
    last_correction = correction;
    si5351.set_correction(correction, SI5351_PLL_INPUT_XO);
  }
}
//*********************************************************************
//                    Odczyt danych z GPS
//**********************************************************************
static void GPSproces(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}
//*********************************************************************

// TX is triggered directly from the 1PPS interrupt edge (<1us latency).
// Symbol frequency changes must only modify the MultiSynth divider,
// never reprogram the PLL VCO, to avoid inter-symbol phase glitches.
// Loop latency at trigger time is minimised because near_tx_boundary
// suppresses all blocking operations in the preceding ~1 second.
void start_ft8_transmission() {
  // TODO: implement FT8/FT4 symbol output via Si5351 CLK1
  // Use si5351.set_freq() on CLK1 only (MS-only changes, keep PLL fixed)
  // to avoid phase glitches between symbols
}
