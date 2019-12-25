//outputs
#define GREEN 5
#define BLUE 3
#define RED 6
#define LED1 9
#define LED2 10
#define BUZZ 11
#define LED3 12
#define RGB_DELAY 10

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
#define STATEMAX 6
long lastChange;
#define CHANGETIME 250
uint8_t prevState = -1;

//IMPORTED STUFF
#include "pitches.h"

uint8_t LEDS[] = {
  RED, LED2, BLUE, LED1, GREEN
};

// notes in the melody:
int melody1[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int melody1Notes[] = {
  4, 8, 8, 4, 4, 4, 4, 4
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

  //init inputs
  pinMode(START_ROUND.pin, INPUT_PULLUP);
  pinMode(STOP_ROUND.pin, INPUT_PULLUP); //ACTIVE HIGH
  pinMode(CENTER.pin, INPUT_PULLUP);
  pinMode(ZBUT.pin, INPUT_PULLUP);
  //pinMode(START.pin, INPUT_PULLUP); //TODO
  attachInterrupt(digitalPinToInterrupt(CENTER.pin), intHandler, (CENTER.pin == LOW ? FALLING : RISING)); //interrupt on center button

  //other
  Serial.begin(115200);
  //randomSeed(analogRead(A5));

  redVal = 0;
  blueVal = 80;
  greenVal = 160;

  while (state == 255)
  {
    digitalWrite(LED3, LOW);
    delay(200);
    digitalWrite(LED3, HIGH);
    delay(200);
  }
  ping();
  ping();
}

bool active(pin in)
{
  if(digitalRead(in.pin) == in.active)
    return true;
  else
    return false;
}

void loop()
{
//  if (active(START))
//  {
//    digitalWrite(LED3, HIGH);
//    Serial.println("pressed");
//  }
//  else
//  {
//    digitalWrite(LED3, LOW);
//    Serial.println("not pressed");
//  }
//
//  return;

  if (prevState != state)
  {
    prevState = state;
    ping();
  }
  if (!digitalRead(ZBUT.pin))
  {
    playSong1();
  }

  if (!digitalRead(START_ROUND.pin))
  {
    for (int i = 140 ; i < 2000 ; i++)
    {
      int offset = map(analogRead(POT.pin), 0, 1023, -100, 500);
      tone(BUZZ, i + offset);
      delayMicroseconds(analogRead(POT.pin)/2);
    }
    for (int i = 2000 ; i > 140 ; i--)
    {
      int offset = map(analogRead(POT.pin), 0, 1023, -100, 500);
      tone(BUZZ, i + offset);
      delayMicroseconds(analogRead(POT.pin)/2);
    }
    noTone(BUZZ);
  }

  if (digitalRead(STOP_ROUND.pin))
  {
    int offset = map(analogRead(POT.pin), 0, 1023, -20, 100);
    tone(BUZZ, 100);
    delay(50 + offset);
    tone(BUZZ, 2000);
    delay(50 + offset);
    noTone(BUZZ);
  }

  pulse();
}

void intHandler()
{

  //debounce the interrupt
  if ((millis() - lastChange) < CHANGETIME)
    return;
  lastChange = millis();

  state++;
  if (state >= STATEMAX)
  {
    state = 0;
  }
}

void ping()
{
  for (int i = 300 ; i < 1000 ; i++)
    tone(BUZZ, i);
  for (int i = 1000 ; i > 300 ; i--)
    tone(BUZZ, i);
  noTone(BUZZ);
}

void playSong1()
{
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / melody1Notes[thisNote];
    int noteScale = map(analogRead(POT.pin), 0, 1023, 80, 200);
    float noteFloat = float(noteScale) / float(100);
    tone(BUZZ, melody1[thisNote]*noteFloat, noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    int potScale = map(analogRead(POT.pin), 0, 1023, -100, 200);
    delay(pauseBetweenNotes + potScale);
    // stop the tone playing:
    noTone(BUZZ);
  }
}

void clearLEDs()
{
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
}

void randomRGB()
{
  redVal++;
  greenVal++;
  blueVal++;
  setColor(redVal, blueVal, greenVal);
  delay(10);
}

void setColor(int red, int green, int blue)
{
  analogWrite(RED, red);
  analogWrite(GREEN, green);
  analogWrite(BLUE, blue);
}

void pulse()
{
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

bool checkPins()
{
  if (!digitalRead(ZBUT.pin))
    return 1;

  if (!digitalRead(START_ROUND.pin))
    return 1;

  if (digitalRead(STOP_ROUND.pin))
    return 1;
  return 0;
}

