#include <si5351.h>          // Etherkit https://github.com/etherkit/Si5351Arduino
#include <Rotary.h>          // Ben Buxton https://github.com/brianlow/Rotary
#include <Wire.h>            // IDE Standard
#include <SoftwareSerial.h>  // IDE Standard
#define IF 10700             // Enter your IF frequency, ex: 455 = 455kHz, 10700 = 10.7MHz, 0 = to direct convert receiver or RF generator, + will add and - will subtract IF offset.
#define BAND_INIT 20         // Enter your initial Band (1-21) at startup, ex: 1 = Freq Generator, 2 = 800kHz (MW), 7 = 7.2MHz (40m), 11 = 14.1MHz (20m).
#define XT_CAL_F 33000       // Si5351 calibration factor, adjust to get exatcly 10MHz. Increasing this value will decreases the frequency and vice versa.
#define S_GAIN 505           // Adjust the sensitivity of Signal Meter A/D input: 101 = 500mv; 202 = 1v; 303 = 1.5v; 404 = 2v; 505 = 2.5v; 1010 = 5v (max).
#define tunestep 9           // The pin used by tune step push button.
#define band A0              // The pin used by band selector push button.
#define adc A1               // The pin used by Signal Meter A/D input.
#define rx_tx 10             // The pin used by RX / TX selector switch, RX = switch open, TX = switch closed to GND. When in TX, the IF value is not considered.
#define rotLeft 3            // The pin used by rotary-left input.
#define rotRight 2           // The pin used by rotary-right input.
#define TX_PIN 11            // SoftwareSerial TX pin
#define RX_PIN 12            // SoftwareSerial RX pin
#define BAUD 9600            // SoftwareSerial baud rate
//------------------------------------------------------------------------------------------------------------
Rotary r = Rotary(rotRight, rotLeft);
Si5351 si5351(0x60);
SoftwareSerial rp(TX_PIN, RX_PIN);
unsigned long freq, freqold, fstep;
long interfreq = IF, interfreqold = 0;
long cal = XT_CAL_F;
unsigned int smval;
byte encoder = 1;
byte stp, n = 1;
byte count, x, xo;
bool sts = 0;
unsigned int period = 100;
unsigned long time_now = 0;
static uint8_t encPrev = 0;  // previous 2-bit state AB
static int8_t encAccum = 0;  // accumulate transitions to 1 detent
static const int8_t encTable[16] = {
  // Transition table: index = (prev<<2) | curr, value = -1,0,+1 per step
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};
void rplog(const char* msg, Stream& io, bool addNewline = false, unsigned perCharDelayMs = 50) {
  if (!msg) return;
  for (const char* p = msg; *p; ++p) {
    delay(perCharDelayMs);
    io.write((uint8_t)*p);
    // yield() is a no-op on most boards; on ESP it keeps WiFi alive
    yield();
  }
  if (addNewline) {
    delay(perCharDelayMs);
    io.write('\n');
  }
}
void rplog(const String& msg, Stream& io, bool addNewline = false, unsigned perCharDelayMs = 50) {
  for (size_t i = 0; i < msg.length(); ++i) {
    delay(perCharDelayMs);
    io.write((uint8_t)msg[i]);
    yield();
  }
  if (addNewline) {
    delay(perCharDelayMs);
    io.write('\n');
  }
}
inline void pollEncoder() {
  // Using INPUT_PULLUP, LOW means active, HIGH means idle
  // Map to 2-bit state: bit1=A=rotRight, bit0=B=rotLeft (order matches Rotary(rotRight, rotLeft))
  uint8_t a = (digitalRead(rotRight) == LOW) ? 1 : 0;
  uint8_t b = (digitalRead(rotLeft) == LOW) ? 1 : 0;
  uint8_t curr = (a << 1) | b;
  uint8_t idx = (encPrev << 2) | curr;
  int8_t delta = encTable[idx];
  if (delta != 0) {
    encAccum += delta;
    // One detent typically = 4 valid transitions; adjust if your encoder is half-step
    if (encAccum >= 4) {
      //set_frequency(1);
      rplog("R", rp);
      encAccum = 0;
    } else if (encAccum <= -4) {
      //set_frequency(-1);
      rplog("L", rp);
      encAccum = 0;
    }
  }
  encPrev = curr;
}
int freeRam() {
  // free RAM check for debugging. SRAM for ATmega328p = 2048Kb.
  // Use 1024 with ATmega168
  int size = 2048;
  byte* buf;
  while ((buf = (byte*)malloc(--size)) == NULL)
    ;
  free(buf);
  return size;
}
//------------------------------------------------------------------------------------------------------------
void setup() {
  Wire.setClock(100000);
  Wire.begin();
  pinMode(rotLeft, INPUT_PULLUP);
  pinMode(rotRight, INPUT_PULLUP);
  pinMode(tunestep, INPUT_PULLUP);
  pinMode(band, INPUT_PULLUP);
  pinMode(rx_tx, INPUT_PULLUP);
  rp.begin(BAUD);
  //rp.println("Nano: Starting now..");
  //pinMode(TX_PIN, OUTPUT);  // ICSP pin 4
  //pinMode(RX_PIN, OUTPUT);  // ICSP pin 1
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal, SI5351_PLL_INPUT_XO);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK0, 1);  // 1 - Enable / 0 - Disable CLK
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);
  // Initialize encoder previous state
  {
    uint8_t a = (digitalRead(rotRight) == LOW) ? 1 : 0;
    uint8_t b = (digitalRead(rotLeft) == LOW) ? 1 : 0;
    encPrev = (a << 1) | b;
    encAccum = 0;
  }
  /*count = BAND_INIT;
  bandpresets();
  stp = 4;
  setstep();*/
  /*for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      rplog("Nano: Detected I2C device on bus: (DEC)", rp);
      rplog((String)addr, rp);
      rplog("\n", rp);
    }
  }
  delay(1500);
  int totalRAM = 2048;  // Total RAM for Arduino Nano (ATmega328P)
  int currentFreeRAM = freeRam();
  int usedRAM = totalRAM - currentFreeRAM;
  rplog("Nano: Total RAM: ", rp);
  rplog((String)totalRAM, rp);
  rplog(" Free RAM: ", rp);
  rplog((String)currentFreeRAM, rp);
  rplog(" Used RAM: ", rp);
  rplog((String)usedRAM, rp);
  rplog("\n", rp);
  rplog("Nano: Running loop now..\n", rp);*/
}
void loop() {
  pollEncoder();
  if (rp.available()) {
    delay(50);
    rp.write(rp.read());
  }
  if (digitalRead(tunestep) == LOW) {
    time_now = (millis() + 300);
    //setstep();
    rplog("T", rp);
    delay(300);
  }
  if (digitalRead(band) == LOW) {
    time_now = (millis() + 300);
    //inc_preset();
    rplog("B", rp);
    delay(300);
  }
  /*if (digitalRead(rx_tx) == LOW) {
    time_now = (millis() + 300);
    sts = 1;
  } else sts = 0;*/
  //sgnalread();
}
/*void sgnalread() {
  smval = analogRead(adc);
  x = map(smval, 0, S_GAIN, 1, 14);
  if (x > 14) x = 14;
}*/
