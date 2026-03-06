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
#include <math.h>
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

// FT8 Konfiguration:
// true  = gerade Perioden (0s, 30s pro Minute) – typisch für CQ-rufende Station
// false = ungerade Perioden (15s, 45s pro Minute) – typisch für antwortende Station
bool ft8_tx_even = true;

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

  // 1 Sekunde Vorlauf vor jedem TX-Fensterstart: keine blockierenden Display-Updates
  bool near_tx_boundary = (second % 15 == 14);

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
  // FT8: tx_trigger nur auf UNSERER Periode auslösen
  if (second % 15 == 0) {
    bool our_period = ft8_tx_even ? (second == 0 || second == 30)
                                  : (second == 15 || second == 45);
    if (our_period) {
      tx_trigger = true;
      tx_second_snapshot = second;
    }
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
  // Verstärkung bewusst niedrig halten (alle Bereiche /4):
  // Verstärkung 1.0 (stab ungeteilt) führt bei TCXO-Thermodrift zu
  // anhaltenden Überschwingern. Mit /4 konvergiert die Schleife
  // zuverlässig, auch wenn der TCXO noch driftet.
  if (stab > 100 || stab < -100) {
    correction = correction + stab / 4;
  }
  else if (stab > 20 || stab < -20) {
    correction = correction + stab / 4;
  }
  else correction = correction + stab / 4;
  pomocna = (10000 / (Freq1 / 1000000));
  stab = stab * 100;
  stab = stab / pomocna;
  stab_float = float(stab);
  stab_float = stab_float / 10;

  stab_on_serial();
}

//********************************************************************************
//                          Maidenhead Locator (6 Stellen)
//********************************************************************************
void maidenhead(float lat, float lon, char *grid) {
  lon += 180.0f;
  lat += 90.0f;
  grid[0] = 'A' + (int)(lon / 20);
  grid[1] = 'A' + (int)(lat / 10);
  grid[2] = '0' + (int)(fmod(lon, 20.0f) / 2);
  grid[3] = '0' + (int)(fmod(lat, 10.0f));
  grid[4] = 'a' + (int)(fmod(lon, 2.0f) * 12);
  grid[5] = 'a' + (int)(fmod(lat, 1.0f) * 24);
  grid[6] = '\0';
}

//********************************************************************************
//                          Status auf Serial Monitor
//********************************************************************************
void stab_on_serial() {
  char grid[7];
  char buf[32];

  Serial.println(F("--- GPSDO Status ---"));

  sprintf(buf, "Time: %02d:%02d:%02d UTC", hour, minute, second);
  Serial.println(buf);

  Serial.print(F("Sats: "));
  Serial.println(gps.satellites.value());

  if (gps.location.isValid()) {
    Serial.print(F("Lat:  "));
    Serial.println(gps.location.lat(), 6);
    Serial.print(F("Lon:  "));
    Serial.println(gps.location.lng(), 6);
    maidenhead(gps.location.lat(), gps.location.lng(), grid);
    Serial.print(F("Grid: "));
    Serial.println(grid);
  } else {
    Serial.println(F("Pos:  no fix"));
  }

  Serial.print(F("Corr: "));
  Serial.print(stab_float);
  Serial.println(F(" Hz"));

  Serial.print(F("Lock: "));
  Serial.println(fixed ? F("YES") : F("NO"));

  if (!fixed) {
    Serial.println(F("Lock-Grund(e):"));
    if (!validGPSflag)
      Serial.println(F("  - kein gueltiger GPS-Fix"));
    if (pps_valid == 0)
      Serial.println(F("  - kein PPS-Signal (>1.2s)"));
    if (abs(stab_float) > 1)  {
      Serial.print(F("  - Freq.abw. "));
      Serial.print(stab_float, 1);
      Serial.println(F(" Hz (Schwelle: 1 Hz)"));
    }
    if (stab_count > 0 && stab_count < 44) {
      Serial.print(F("  - Messung laeuft noch ("));
      Serial.print(stab_count);
      Serial.println(F("s verbleibend)"));
    }
  }
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
//             TX window guard: true NUR während UNSERER FT8-TX-Periode
//             Gerade:   Sekunden  0-12 und 30-42 jeder Minute
//             Ungerade: Sekunden 15-27 und 45-57 jeder Minute
//             Nur in diesen Fenstern darf CLK1 NICHT verändert werden.
//********************************************************************
bool is_in_tx_window() {
  if (ft8_tx_even) {
    return (second >= 0 && second < 13) || (second >= 30 && second < 43);
  } else {
    return (second >= 15 && second < 28) || (second >= 45 && second < 58);
  }
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

// FT8-Sendefenster gestartet.
// Der SX1280 sendet FT8 auf 2.4 GHz eigenständig; CLK1 (51 MHz) ist sein
// Referenztakt. Während des TX-Fensters (is_in_tx_window() == true) wird
// CLK1 durch correct_si5351a() NICHT verändert. Korrekturen werden erst
// nach Ende des TX-Fensters (ab Sekunde 13/28/43/58) angewendet.
void start_ft8_transmission() {
  // Status auf Display
  oled.setCursor(0, 4);
  oled.print("FT8 TX         ");

  // Status auf Serial
  char buf[40];
  sprintf(buf, "FT8 TX start: %02d:%02d:%02d UTC (%s Periode)",
    hour, minute, tx_second_snapshot,
    ft8_tx_even ? "gerade" : "ungerade");
  Serial.println(buf);
}
