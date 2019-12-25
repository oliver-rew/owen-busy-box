//outputs
#define GREEN 5
#define BLUE 3
#define RED 6
#define LED1 9
#define LED2 10
#define BUZZ 11
#define LED3 12
#define RGB_DELAY 10
#define STATEMAX 6
#define CHANGETIME 250

struct pin {
  uint8_t pin;
  bool active; //active level
};

//inputs
pin START = {A2, LOW};
pin STOP = {A3, HIGH};
pin ZBUT = {13, LOW};
pin CENTER = {2, LOW};
pin START_ROUND = {8, LOW};
pin STOP_ROUND = {7, HIGH};
pin POT = {A0}; // analog input pin

uint8_t redVal;
uint8_t blueVal;
uint8_t greenVal;

volatile uint8_t state = -1;
long lastChange;
uint8_t prevState = -1;

//IMPORTED STUFF
#include "pitches.h"

uint8_t LEDS[] = {
  RED, LED2, BLUE, LED1, GREEN
};

// note is type that can be used in an array to make a song
struct note {
  uint16_t pitch; // pitch of note, from pitches.h
  uint8_t dur; // note duration 4 = quarter note, 8 = eighth note, etc.:
};

// shaveHaircutNotes with duration
note shaveHaircut[] = {
  {NOTE_C4, 4},
  {NOTE_G3, 8},
  {NOTE_G3, 8},
  {NOTE_A3, 4},
  {NOTE_G3, 4},
  {0,       4},
  {NOTE_B3, 4}, 
  {NOTE_C4, 4},
  {NULL, NULL} //null terminate song
};

note beethoven8th[] = {
  {NOTE_E4, 4},
  {NOTE_E4, 4},
  {NOTE_F4, 4},
  {NOTE_G4, 4},
  {NOTE_G4, 4},
  {NOTE_F4, 4},
  {NOTE_E4, 4},
  {NOTE_D4, 4},
  {NOTE_C4, 4},
  {NOTE_C4, 4},
  {NOTE_D4, 4},
  {NOTE_E4, 4},
  {NOTE_E4, 2},
  {NOTE_D4, 8},
  {NOTE_D4, 2},
  {NULL, NULL},
};

note jingleBells[] = {
  {NOTE_E4, 4},
  {NOTE_E4, 4},
  {NOTE_E4, 2},
  {NOTE_E4, 4},
  {NOTE_E4, 4},
  {NOTE_E4, 2},
  {NOTE_E4, 4},
  {NOTE_G4, 4},
  {NOTE_C4, 4},
  {NOTE_D4, 4},
  {NOTE_E4, 1},

  {NOTE_F4, 4},
  {NOTE_F4, 4},
  {NOTE_F4, 4},
  {NOTE_F4, 4},
  {NOTE_F4, 4},
  {NOTE_E4, 4},
  {NOTE_E4, 4},
  {NOTE_E4, 4},
  {NOTE_E4, 4},
  {NOTE_D4, 4},
  {NOTE_D4, 4},
  {NOTE_E4, 4},
  {NOTE_D4, 2},
  {NOTE_G4, 2},

  {NULL, NULL},
};


void setup() {

  //init outputs
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(BLUE, LOW);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);

  //init inputs with pullups
  pinMode(START_ROUND.pin, INPUT_PULLUP);
  pinMode(STOP_ROUND.pin, INPUT_PULLUP); //ACTIVE HIGH
  pinMode(CENTER.pin, INPUT_PULLUP);
  pinMode(ZBUT.pin, INPUT_PULLUP);
  pinMode(START.pin, INPUT_PULLUP);
  pinMode(STOP.pin, INPUT_PULLUP);

  // center pin is attached to interrupt that changes the state
  attachInterrupt(digitalPinToInterrupt(CENTER.pin), intHandler, (CENTER.pin == LOW ? FALLING : RISING)); //interrupt on center button

  //other
  Serial.begin(115200);
  //randomSeed(analogRead(A5));

  redVal = 0;
  blueVal = 80;
  greenVal = 160;

  // wait for center button to be pressed before proceeding 
  waitForCenterButton();

  // make some noise at start
  ping();
  ping();
}

void loop()
{
  // if state was changed in interrupt handler, make noise
  if (prevState != state) {
    prevState = state;
    ping();
  }

  // play the songs based on what button is pressed
  if (active(ZBUT))
    playSong(beethoven8th);
  
  if(active(START))
    playSong(shaveHaircut);

  if(active(STOP))
    playSong(jingleBells);

  // play siren if start button is pressed
  if (active(START_ROUND)) 
    siren();

  // beep if stop is pressed
  if (active(STOP_ROUND))
    beep();

  pulse();
}

// intHandler changes the state when the center button is pressed
void intHandler() {
  // debounce the pin. If enough time has not passed since the last 
  // press, return without changing the state
  if ((millis() - lastChange) < CHANGETIME)
    return;
  
  // record now as the last pressed time
  lastChange = millis();

  // change the state
  state++;
  if (state >= STATEMAX)
    state = 0;

}

void ping() {
  for (int i = 300 ; i < 1000 ; i++)
    tone(BUZZ, i);
  for (int i = 1000 ; i > 300 ; i--)
    tone(BUZZ, i);
  noTone(BUZZ);
}

// playSong plays the specified song, using a the 
// reading of the pot to set the tempo
void playSong(note* song) {
  // check for NULL pointer
  if(song == NULL)
    return;

  // loop through all notes, will return at end
  for (int i = 0; ; i++) {
    // song is NULL terminated, so return when both are NULL
    if(song[i].pitch == NULL && song[i].dur == NULL)
      return;

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / song[i].dur;

    // this mapping makes the pitch higher as the tempo gets slower
    //int noteScale = map(analogRead(POT.pin), 0, 1023, 80, 200);

  // this mapping makes the pitch lower as the tempo gets slower
    int noteScale = map(analogRead(POT.pin), 0, 1023, 200, 80);

    float noteFloat = float(noteScale) / float(100);
    tone(BUZZ, song[i].pitch*noteFloat, noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    int potScale = map(analogRead(POT.pin), 0, 1023, -100, 200);
    delay(pauseBetweenNotes + potScale);
    // stop the tone playing:
    noTone(BUZZ);
  }
}

void clearLEDs() {
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
}

void randomRGB() {
  redVal++;
  greenVal++;
  blueVal++;
  setColor(redVal, blueVal, greenVal);
  delay(10);
}

void setColor(int red, int green, int blue) {
  analogWrite(RED, red);
  analogWrite(GREEN, green);
  analogWrite(BLUE, blue);
}

// pulse pulses a certain LED based on the current state
void pulse() {
  uint8_t startState = state;
  switch (state) {
    case 5:
      randomRGB();
      break;
    default:
      clearLEDs();
      for (int i = 0; i < 255 ; i++)
      {
        if (startState != state || checkPins()) //this is to interrupt pusling without using interrupts
        {
          clearLEDs();
          return;
        }
        analogWrite(LEDS[startState], i);
        delayMicroseconds(2 * analogRead(POT.pin));
      }
      for (int i = 255; i > 0 ; i--)
      {
        if (startState != state || checkPins())
        {
          clearLEDs();
          return;
        }
        analogWrite(LEDS[startState], i);
        delayMicroseconds(2 * analogRead(POT.pin));
      }
      clearLEDs();
      break;
  }
}

bool checkPins() {
  if (active(ZBUT))
    return 1;

  if (active(START_ROUND))
    return 1;

  if (active(STOP_ROUND))
    return 1;
  return 0;
}

// active() reads the current digital value of the pin and 
// returns true if its state matches the pin's active state
bool active(pin in) {
  if(digitalRead(in.pin) == in.active)
    return true;
  else
    return false;
}

// waitForCenterButton flashes the center button light and
// only returns when the center button is pressed
void waitForCenterButton() {
  while (state == 255) {
    digitalWrite(LED3, LOW);
    delay(200);
    digitalWrite(LED3, HIGH);
    delay(200);
  }
}

// siren makes a siren noise
void siren() {
  for (int i = 140 ; i < 2000 ; i++) {
    int offset = map(analogRead(POT.pin), 0, 1023, -100, 500);
    tone(BUZZ, i + offset);
    delayMicroseconds(analogRead(POT.pin)/2);
  }

  for (int i = 2000 ; i > 140 ; i--) {
    int offset = map(analogRead(POT.pin), 0, 1023, -100, 500);
    tone(BUZZ, i + offset);
    delayMicroseconds(analogRead(POT.pin)/2);
  }
  noTone(BUZZ);
}

// beep makes a beeping noise with a period based on the pot
void beep() {
  int offset = map(analogRead(POT.pin), 0, 1023, -20, 100);
  tone(BUZZ, 100);
  delay(50 + offset);
  tone(BUZZ, 2000);
  delay(50 + offset);
  noTone(BUZZ);
}