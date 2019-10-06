/*things to note: somehow rgb affects mic sensitivity*/
/*improvements: auto find golden number and silnt_set*/
/*FIXED ENVIRONMENT CONSTANTS, PLAY WITH THESE VALUES TILL IDEAL SENSITIVITY IS ACHIEVED*/
#define GOLDEN_NMB 531 //silent number should e golden nmb - 2
#define SILENT_SET 527
#define FREQ_SIZE 7
#define F_THRES 0
#define OFF_TRIGGER 2
/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/
#include <Adafruit_NeoPixel.h>
/*PINS*/
#define RED A5
#define GREEN A3
#define BLUE A2
#define BLUE_LED 10
#define PIN 6
#define N 14
#define MIC A6
#define interruptPin 2
/*EVENT TIME CONSTANTS (MINIMUM 20MS APART) */
#define TIME_FREQ_READ 29
#define TIME_ROOF_ON 140
#define TIME_ROOF_OFF 120
#define TIME_RGB_ON 230
#define TIME_RGB_OFF 470
/*INTERUPT VARS*/
volatile unsigned long duration=0; // accumulates pulse width
volatile unsigned int pulsecount=0;
volatile unsigned long previousMicros=0;
/*TIME VARS*/
long prev_freq_read = 0; // will store last time of the cycle end
long prev_roof_on = 0;
long prev_roof_off = 0;
long prev_rgb_on = 0;
long prev_rgb_off = 0;
/*NEOPIX VARS*/
int red[12];
int green[12];
int blue[12];
int white[12];
int wave = 0;
bool off = false;
Adafruit_NeoPixel roof = Adafruit_NeoPixel(N, PIN, NEO_GRB + NEO_KHZ800);
/*RGB VARS*/
float iter = 1;
/*KEY VARS*/
int input;
int freq[FREQ_SIZE]; //freq sample to see max, min n avg for a given time
int amplitude[FREQ_SIZE];
/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/
void calibration() {
  int countFreq = 0;
  while (countFreq < 200) {
    if (countFreq > 300) {
      countFreq = 300;
    }
    input = analogRead(MIC);
    Serial.println(input);
    if (input >= GOLDEN_NMB - 2 && input <= GOLDEN_NMB + 2) {
      countFreq ++;
      digitalWrite(BLUE_LED, HIGH);
      green[6] = 0;
      green[0] = 0;
    }
    else {
      countFreq --;
      if (input < SILENT_SET) {
        green[0] = 5;
        green[6] = 0;
      }
      else {
        green[6] = 5;
        green[0] = 0;
      }
      digitalWrite(BLUE_LED, LOW);
    }
    set_();
  }
  red[0] = 5;
  red[1] = 5;
  red[5] = 5;
  red[6] = 5;
  set_();
  delay(2000);
  red[0] = 0;
  red[1] = 0;
  red[5] = 0;
  red[6] = 0;
  set_();
}

void set_() {
  for (int i = 7; i < 10; i ++) {
    red[i] = 0;
    green[i] = 0;
    blue[i] = 0;
  }
  roof.setPixelColor(0,red[0],green[0],blue[0]);
  roof.setPixelColor(1,red[1],green[1],blue[1]);
  roof.setPixelColor(2,red[8],green[8],blue[8]);
  roof.setPixelColor(3,green[4],white[8],red[4]);
  roof.setPixelColor(4,white[4],blue[4],green[10]);
  roof.setPixelColor(5,blue[10],red[10],white[10]);
  roof.setPixelColor(6,red[9],green[9],blue[9]);
  roof.setPixelColor(7,green[2],white[9],red[2]);
  roof.setPixelColor(8,white[2],blue[2],green[7]);
  roof.setPixelColor(9,blue[7],red[7],white[7]);
  roof.setPixelColor(10,red[3],green[3],blue[3]);
  roof.setPixelColor(11,green[5],white[3],red[5]);
  roof.setPixelColor(12,green[6],blue[5],red[6]);
  roof.setPixelColor(13,0,blue[6],0);
  roof.show();
}

void isr() // interrupt service routine //execute when param 3 happens
{
  unsigned long currentMicros = micros();
  duration += currentMicros - previousMicros;
  previousMicros = currentMicros;
  pulsecount++;
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr, RISING);
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(BLUE_LED, OUTPUT);
  roof.begin();
  for (int i = 1; i < 12; i ++) { //initalie all to 0
    red[i] = 0;
    green[i] = 0;
    blue[i] = 0;
    set_();
  }
  //calibration();
}

void loop() {
  input = analogRead(MIC);
  unsigned long currentMillis = millis();
  digitalWrite(BLUE_LED, (input == GOLDEN_NMB));
  if (currentMillis - prev_freq_read >= TIME_FREQ_READ)
  {
    prev_freq_read = currentMillis;    // need to bufferize to avoid glitches
    unsigned long _duration = duration; //duration = period
    unsigned long _pulsecount = pulsecount;
    float Freq;
    duration = 0; // clear counters
    pulsecount = 0;
    if (_duration != 0) {
      Freq = 1e6 / float(_duration); //F = 1/T units in micro 1e6
      Freq *= _pulsecount;
    }
    else {
      Freq = -1;
    }
    Serial.print(digitalRead(2));
    Serial.print("    ");
    Serial.print(input);
    Serial.print("    ");
    Serial.println(Freq);
    for (int i = FREQ_SIZE - 1; i > 0; i--) { //creating a log to observe trends in freq
      freq[i] = freq[i - 1];
      amplitude[i] = amplitude[i - 1];
    }
    freq[0] = Freq;
    amplitude[0] = input;
  }

  else if (currentMillis - prev_rgb_on >= TIME_RGB_ON) { //rgb on
    prev_rgb_on = currentMillis;
    int dimmer = 255;
    if (off) {
      dimmer = 100;
    }
    analogWrite(RED, round(iter * 5) % dimmer);
    analogWrite(GREEN, round(iter * 3) % dimmer);
    analogWrite(BLUE, round(iter * 2) % dimmer);
    iter += 0.5;
  }

  else if (currentMillis - prev_roof_on >= TIME_ROOF_ON && !off) { //roof on
    prev_roof_on = currentMillis;
    for (int i = 0; i < 7; i ++) {//redder: greater difference from GOLDEN NUM, more green: higher freq, more blue: create waveform
      red[i] = ((abs(amplitude[i] - GOLDEN_NMB) * 27) % 200);
      green[i] = (freq[i] / 16) % 230;
      blue[i] = ((wave + i) * 30) % 210 + 45;
    }
    wave ++;
    set_();
  }

  if (currentMillis - prev_roof_off >= TIME_ROOF_OFF) { //roof on
    off = false;
    int off_counter = 0; //if off counter hits 2, off = true
    prev_roof_off = currentMillis;
    for (int i = 0; i < FREQ_SIZE; i ++) {
      if (freq[i] < F_THRES) {
        off_counter ++;
      }
    }
    if (off_counter > OFF_TRIGGER) {
      off = true;
    }
    if (off) {
      for (int j= 0; j < 7; j ++) {
        red[j] = 0;
        green[j] = 0;
        blue[j] = 0;
      }
    }
    set_();
  }/*
  if (currentMillis - prev_rgb_off >= TIME_RGB_OFF) { //rgb on
    prev_rgb_off = currentMillis;
    analogWrite(RED, 0);
    analogWrite(GREEN, 0);
    analogWrite(BLUE, 0);
  }*/
}
