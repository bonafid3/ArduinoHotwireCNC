#include <Wire.h>
#include <TMCStepper.h>
#include "U8glib.h"

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_FAST);  // Dev 0, Fast I2C / TWI
#define CSN3        2

#define DRV_EN4     3
#define CSN4        4
#define CSN5        10
#define DIR5        11
#define DRV_EN5     12
#define STEP5       13
#define STEP1       22
#define DRV_EN1     23
#define DIR1        24
#define STEP2       25
#define DIR2        26
#define SDO         27
#define SDI         28
#define SCK         29
#define SELECT      30
#define CSN1        31
#define DRV_EN2     32
#define CSN2        33
#define STEP3       34
#define DRV_EN3     35
#define DIR3        36
#define STEP4       37
#define DIR4        38
#define HEAT_TO_PC  39
#define TIMER_TO_PC 40
#define HEAT_OUT    44

// ANALOG INPUT
#define MOT1        A0
#define MOT2        A1
#define MOT3        A2
#define MOT4        A3
#define MOT5        A4
#define HEAT_IN     A5
#define SPD_IN      A6
#define RUN_IN      A7
#define SELECT_SW   A8
#define SENSE       A9

#define MIN_DELAY 15
#define MAX_DELAY 50

int gCnt = 0;
int gDelay = 0;
int gDir = 0;

int gSenseIdx = 0;
int gSense[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

boolean gStall = false;

boolean toggleTimer = 0;

TMC2160Stepper stepper1(CSN1, SDI, SDO, SCK);
TMC2160Stepper stepper2(CSN2, SDI, SDO, SCK);
TMC2160Stepper stepper3(CSN3, SDI, SDO, SCK);
TMC2160Stepper stepper4(CSN4, SDI, SDO, SCK);
TMC2160Stepper stepper5(CSN5, SDI, SDO, SCK);

void setupStepper(TMC2160Stepper& stepper)
{
  stepper.begin();

  stepper.intpol(1);
  stepper.microsteps(0);

  stepper.en_pwm_mode(true);

  stepper.rms_current(1500);
  stepper.push();

  delay(250);

  Serial.print("IHOLD: ");
  Serial.println(stepper.ihold());

  Serial.print("IRUN: ");
  Serial.println(stepper.irun());

  Serial.print("INTPOL: ");
  Serial.println(stepper.intpol());

  Serial.print("MRES: ");
  Serial.println(stepper.mres());
}

void setup()
{
  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  
  Serial.begin(9600);

  pinMode(SELECT, OUTPUT);
  pinMode(HEAT_OUT, OUTPUT);
  pinMode(TIMER_TO_PC, OUTPUT);
  
  pinMode(STEP1, OUTPUT);
  pinMode(DRV_EN1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  digitalWrite(DRV_EN1, LOW); // enable stepper1

  pinMode(STEP2, OUTPUT);
  pinMode(DRV_EN2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  digitalWrite(DRV_EN2, LOW); // enable stepper2

  pinMode(STEP3, OUTPUT);
  pinMode(DRV_EN3, OUTPUT);
  pinMode(DIR3, OUTPUT);
  digitalWrite(DRV_EN3, LOW); // enable stepper3

  pinMode(STEP4, OUTPUT);
  pinMode(DRV_EN4, OUTPUT);
  pinMode(DIR4, OUTPUT);
  digitalWrite(DRV_EN4, LOW); // enable stepper4

  pinMode(STEP5, OUTPUT);
  pinMode(DRV_EN5, OUTPUT);
  pinMode(DIR5, OUTPUT);
  digitalWrite(DRV_EN5, LOW); // enable stepper5

  setupStepper(stepper1);
  setupStepper(stepper2);
  setupStepper(stepper3);
  setupStepper(stepper4);
  //setupStepper(stepper5);

  cli();

  // PC 8 bit TIMER SETUP 4kHz
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 4khz increments
  OCR0A = 30;// = (16*10^6) / (2000*128) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | ( 1 << CS00);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

  // STEP 16 bit TIMER SETUP
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for ? increments
  OCR1A = 100;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 64 prescaler
  //TCCR1B |= (1 << CS01) | (1 << CS00);
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei(); 
}

int remap(int input, int ins, int ine, int ous, int oue)
{
  double slope = 1.0 * (oue - ous) / (ine - ins);
  return ous + slope * (input - ins);
}

// PC TIMER FOR GFMC software, provides a timer for steps
ISR(TIMER0_COMPA_vect)
{
  digitalWrite(TIMER_TO_PC, !digitalRead(TIMER_TO_PC));
}

// STEP TIMER
ISR(TIMER1_COMPA_vect)
{
  // if delay is max delay, then we want to stop the motors
  if(gDelay == MAX_DELAY)
    return;

  // this is when no run dir button pressed
  if(gDir < 250)
    return;

  // as long as cnt not reaches gDelay, skip this step
  if(gCnt++ < gDelay)
    return;

  if(digitalRead(MOT1) == HIGH) digitalWrite(STEP1, HIGH);
  if(digitalRead(MOT2) == HIGH) digitalWrite(STEP2, HIGH);
  if(digitalRead(MOT3) == HIGH) digitalWrite(STEP3, HIGH);
  if(digitalRead(MOT4) == HIGH) digitalWrite(STEP4, HIGH);

  delayMicroseconds(1);

  if(digitalRead(MOT1) == HIGH) digitalWrite(STEP1, LOW);
  if(digitalRead(MOT2) == HIGH) digitalWrite(STEP2, LOW);
  if(digitalRead(MOT3) == HIGH) digitalWrite(STEP3, LOW);
  if(digitalRead(MOT4) == HIGH) digitalWrite(STEP4, LOW);

  gCnt = 0;
}

void draw(int heat, int current, int spd)
{
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);

  char buf[64];
  sprintf(buf, "HEAT: %d%%", heat);
  u8g.drawStr( 0, 15, buf);
  sprintf(buf, "CURR: %d", current);
  u8g.drawStr( 0, 38, buf);
  sprintf(buf, "SPEED: %d%%", spd);
  u8g.drawStr( 0, 60, buf);
}

void displayMode(const char* mode)
{
  u8g.setFont(u8g_font_unifont);
  u8g.drawStr( 110, 15, mode);
}

void displayRunForward()
{
  const uint8_t uparrow[] PROGMEM = {
  0b00000001,0b10000000,
  0b00000011,0b11000000,
  0b00000111,0b11100000,
  0b00001111,0b11110000,
  0b00011111,0b11111000,
  0b00000111,0b11100000,
  0b00000111,0b11100000,
  0b00000111,0b11100000,
  0b00000111,0b11100000,
  0b00000111,0b11100000,
  0b00000111,0b11100000
  };
  
  u8g.drawBitmap(111, 27, 2, 11, uparrow);
}

void displayRunBackward()
{
  const uint8_t downarrow[] PROGMEM = {
  0b00000111,0b11100000,
  0b00000111,0b11100000,
  0b00000111,0b11100000,
  0b00000111,0b11100000,
  0b00000111,0b11100000,
  0b00000111,0b11100000,
  0b00011111,0b11111000,
  0b00001111,0b11110000,
  0b00000111,0b11100000,
  0b00000011,0b11000000,
  0b00000001,0b10000000
  };

  u8g.drawBitmap(111, 50, 2, 11, downarrow);
}
void loop()
{  
  gSense[gSenseIdx++] = analogRead(SENSE);
  if(gSenseIdx == 10) gSenseIdx = 0;

  int senseAvg = 0;
  for(int i=0; i<10; i++) senseAvg += gSense[i];
  senseAvg /= 10;
  
  int mode = analogRead(SELECT_SW);

  // which direction to run
  gDir = analogRead(RUN_IN);

  if(gCnt > gDelay) {
    Serial.println("STALL CANCELLED");
    gCnt = 0;
  }
  
  int speed = analogRead(SPD_IN);
  gDelay = remap(speed, 1024, 0, MIN_DELAY, MAX_DELAY);
  
  // remaps analog potentiometer range 0-1023 to PWM duty cycle 0-255 which is 0-100%
  int heat = remap(analogRead(HEAT_IN), 0, 1023, 0, 255);
  analogWrite(HEAT_OUT, heat);

  Serial.println(gDir);
  
  u8g.firstPage();
  do {
    if(gDir > 250 && gDir < 750) { // direction backward
      digitalWrite(DIR1,HIGH);
      digitalWrite(DIR2,HIGH);
      digitalWrite(DIR3,HIGH);
      digitalWrite(DIR4,HIGH);
      displayRunBackward();
    } else if(gDir >= 750) { // direction forward
      digitalWrite(DIR1,LOW);
      digitalWrite(DIR2,LOW);
      digitalWrite(DIR3,LOW);
      digitalWrite(DIR4,LOW);
      displayRunForward();
    }
    
    draw(remap(heat, 0, 255, 0, 100), senseAvg, remap(speed, 0, 1023, 0, 100));
    
    if(mode < 512) {
      displayMode("PC"); // PC
      digitalWrite(SELECT, HIGH);
    } else {
      displayMode("MA"); // Manual
      digitalWrite(SELECT, LOW);
    }
  } while( u8g.nextPage() );

}
