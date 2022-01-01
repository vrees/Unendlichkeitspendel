#include <Arduino.h>

#include "TimerOne.h"
#include "Plotter.h"

#define PinCoilA 2
#define PinCoilB 3
#define LED 13
#define LOOP_DELAY 3
#define COIL_ON_TIME 100
#define COIL_RELAX_TIME 250
#define MOTOR_START_POS 360 // so wählen, dass comparePosition ~250 ist
#define COIL_MIDDLE_COMPENSATION 85
#define MOTOR_TIMER_VALUE 0 // Wenn bekannt: Wert hier eintragen

#define Monitor
// #define PlotterOutput

void stepper();      // ISR für den Schrittmotor
byte CheckButtons(); // Tasten eingabe
void printCoilValues(int a, int b);
void plot();
void readMeanVoltage();
void setupPortAndPlotter();
void readCoilValuesInLoop();
int calculatePeriod();
void manualCoilControl();
void initMotorTimer();

int coilAValue;
int coilBValue;
int meanCoilA;
int meanCoilB;
int period;

byte pauseAtTransit = 0;

Plotter p; // create plotter

int motorTimerValue = MOTOR_TIMER_VALUE;
int motorPosition; // Motor Position beim Transit
bool isMotorRunning = false;

long pendelTimerStart = 0;
long pendelTimerValue;
unsigned long lastTouchA;
unsigned long lastTouchB;
int comparePosition;
long passage;
long meanPassage;
long varianzPassage;
byte direction = 1;
bool isMotorControl = false;
bool isMotorDelay = false;
bool isShutdown = false;

void setup()
{
  setupPortAndPlotter();
  readMeanVoltage();
  // readCoilValuesInLoop();
  // manualCoilControl();

  period = calculatePeriod();

  initMotorTimer();

#ifdef Monitor
  Serial.print("Periode=");
  Serial.print(period);
  Serial.print(", MotorTimerValue=");
  Serial.println(motorTimerValue);
  Serial.println("");
  Serial.println("PassageTime  Varianz");
#endif
}

void loop()
{
  long delta1;
  long delta2;
  unsigned long CurrentMillis;

  CurrentMillis = millis();

  /******************************************/
  /* Behandle Coil A                          */
  /******************************************/
  coilAValue = analogRead(A0); // Lese CoilA Spannung
  if (pendelTimerStart == 0)   // Wenn CoilA nicht gefeuert hat, dann:
  {
    if (coilAValue < (meanCoilA - 8)) // Magnet über CoilA ?
    {
      delta1 = CurrentMillis - lastTouchA;
      lastTouchA = CurrentMillis;
      if (delta1 > (period >> 1))
      {
        digitalWrite(PinCoilB, HIGH);
        pendelTimerStart = CurrentMillis;
        passage = (lastTouchA - lastTouchB) >> 1;
        if (!isMotorRunning)
        {                                        // Berechne die Varianz der Passage, um stabilität des Pendulums zu eruieren
          delta1 = passage - (meanPassage >> 5); // Abweichung old Mean
          meanPassage = meanPassage + delta1;
          delta2 = passage - (meanPassage >> 5); // Abweichung new Mean
          varianzPassage = (varianzPassage - varianzPassage >> 5) + delta1 * delta2;

#ifdef Monitor
          Serial.print(passage);
          Serial.print(", ");
          Serial.println(varianzPassage);
#endif
        }
      }
      else // delta1 < (Period >> 1)
      {
        if (isMotorControl == true)
        {
          isMotorControl = false;
          if (isMotorRunning)
          {
            if (motorPosition > comparePosition)
            {
              motorTimerValue = motorTimerValue + 1;
              Timer1.setPeriod(motorTimerValue >> 2);
            }
            if (motorPosition < comparePosition)
            {
              stepper();
              motorTimerValue = motorTimerValue - 1;
              Timer1.setPeriod(motorTimerValue >> 2);
            }
#ifdef Monitor
            Serial.print(motorPosition);
            Serial.print(",");
            Serial.println(motorTimerValue >> 2);
#endif
          }
          else //  if (!isMotorRunning)
          {
            if (varianzPassage < 100) // Warte, bis Pendel stabil schwingt, befor Motor gestartet wird
            {
#ifndef PlotterOutput
              isMotorRunning = true;
              motorPosition = MOTOR_START_POS;              // Hier müssen wir bei dem nächsten Transit wieder hin!
              passage = passage + COIL_MIDDLE_COMPENSATION; // Bei Motorstart erstmalig die Kompensation berücksichtigen
              direction = 1;
              delay(passage);                                                            // Warte, bis Pendel im Transit ist
              Timer1.attachInterrupt(stepper);                                           // Und starte motor
              comparePosition = motorPosition - (int)(((long)(passage << 10)) / period); // Theoretische Motor Position kurz vorher, als Pendel über CoilA war
#endif

#ifdef Monitor
              Serial.println(" ");
              Serial.println("Motor Started");
              Serial.println("comparePosition");
              Serial.print(comparePosition);
              Serial.println(" ");
              Serial.println("comparePosition  motorTimerValue");
#endif
            }
          }
        }
      }
    }
  }

  /******************************************/
  /* Behandle Coil B                          */
  /******************************************/
  coilBValue = analogRead(A1); // Lese CoilB Spannung
  if (pendelTimerStart == 0)   // Wenn CoilB nicht gefeuert hat, dann:
  {
    if (coilBValue < (meanCoilB - 8)) // ist Pendel über CoilB ?
    {
      delta1 = CurrentMillis - lastTouchB;
      lastTouchB = CurrentMillis;
      if (delta1 > (period >> 1))
      {
        digitalWrite(PinCoilA, HIGH);
        pendelTimerStart = CurrentMillis;
      }
      else
        isMotorControl = true;
    }
  }

  /******************************************/
  /* Pendel Timer -> Spule aktivieren       */
  /******************************************/
  if (pendelTimerStart > 0)
  {
    pendelTimerValue = CurrentMillis - pendelTimerStart;
    if (pendelTimerValue > COIL_RELAX_TIME)
      pendelTimerStart = 0; // Stoppe timer und ermögliche Sensornutzung der Spulen
    else
    {
      if (pendelTimerValue > COIL_ON_TIME)
        PORTD = (PORTD & 0xF3); // PinCoilA und PinCoilB ausschalten
    }
  }

  if (CheckButtons() == 2) // Shutdown starten, wenn beide Tasten gedrückt
    isShutdown = true;     // Motor stop in Transit Position

#ifdef PlotterOutput
  Serial.print(coilBValue);
  Serial.print(",");
  Serial.print(meanCoilA);
  Serial.print(",");
  Serial.print(pendelTimerValue << 1);
  Serial.print(",");
  Serial.println(coilAValue);
#endif

  delay(LOOP_DELAY);
}

/******************************************/
/* ISR Schrittmotor                            */
/******************************************/
void stepper() // Interrupt Routine, die von Timer 1 alle x Microsekunden (motorTimerValue) aufgerrufen wird.
{
  static byte Steps;
  static byte Pause;
  if (!isMotorDelay)
  {
    if ((motorPosition != MOTOR_START_POS) || (Pause == 0)) // Sind wir nicht im Transit?
    {                                                       // dann Motor weiterdrehen
      digitalWrite(LED, LOW);                               // Stroboskop LED ausschalten
      Steps = (Steps + direction) & 0x07;
      motorPosition = (motorPosition + (int)direction);
      motorPosition = motorPosition & 0x01FF; // Wir werten nur die relative motor position aus
      Pause = pauseAtTransit;                 // Pausenlänge im Transit in "IR counts"
      switch (Steps)
      {
      case 0:
        PORTB = (PORTB & 0xf0) | B00001000;
        break;
      case 1:
        PORTB = (PORTB & 0xf0) | B00001100;
        break;
      case 2:
        PORTB = (PORTB & 0xf0) | B00000100;
        break;
      case 3:
        PORTB = (PORTB & 0xf0) | B00000110;
        break;
      case 4:
        PORTB = (PORTB & 0xf0) | B00000010;
        break;
      case 5:
        PORTB = (PORTB & 0xf0) | B00000011;
        break;
      case 6:
        PORTB = (PORTB & 0xf0) | B00000001;
        break;
      case 7:
        PORTB = (PORTB & 0xf0) | B00001001;
      }
    }
    if (motorPosition == MOTOR_START_POS) // in TransitPosition!!!
    {
      digitalWrite(LED, HIGH); // Schalte Stoboskop LED ein
      if (Pause > 0)
        Pause = Pause - 1; // Pause im Transit
      if (isShutdown)      // Wenn Shutdown gewünscht:
      {
        PORTB = (PORTB & 0xD0); // Motor und LED ausschalten
        PORTD = (PORTD & 0xF3); // Spulen ebenfalls ausschalten
        Timer1.detachInterrupt();
        while (1)
          ; // Wir warten auf "Power loss"
      }
    }
  }
  else // Eine kurze Pause für den Motor
    isMotorDelay = false;
}

/******************************************/
/* Behandlung der Tasten                  */
/******************************************/
byte CheckButtons()
{
  static byte button1 = 0;
  static byte button2 = 0;

  button1++;
  if (digitalRead(5) == HIGH)
    button1 = 0;

  button2++;
  if (digitalRead(6) == HIGH)
    button2 = 0;

  if (!isMotorRunning) // Justierung der Lochscheiben Position bevor der Motor läuft
  {
    if ((button1 & 0x0F) == 0x0F)
    {
      direction = -1;
      stepper();
    }
    if ((button2 & 0x0F) == 0x0F)
    {
      direction = 1;
      stepper();
    }
  }
  else // Justierung der Lochscheiben Position nach Motorstart
  {
    if ((button1 & 0x1F) == 0x1F)
      isMotorDelay = true;
    if ((button2 & 0x1F) == 0x1F)
      stepper();
  }
  return (bool)button1 + (bool)button2; // 0: no press, 1: one key, 2: both keys
}

void manualCoilControl()
{
  do
  {
    coilAValue = analogRead(A0);
    coilBValue = analogRead(A1);
    plot();
    printCoilValues(coilAValue, coilBValue);

    // Button1 pressed
    if (digitalRead(5) == LOW)
      digitalWrite(PinCoilA, HIGH);
    else
      digitalWrite(PinCoilA, LOW);

    // Button2 pressed
    if (digitalRead(6) == LOW)
      digitalWrite(PinCoilB, HIGH);
    else
      digitalWrite(PinCoilB, LOW);

    delay(1);
  } while (1);
}

void printCoilValues(int a, int b)
{
#ifdef Monitor
  Serial.print("MeanCoil A/B: ");
  Serial.print(a);
  Serial.print(" / ");
  Serial.println(b);
#endif
}

void plot()
{
#ifdef PlotterOutput
  p.Plot();
#endif
}

void

readMeanVoltage()
{
  meanCoilA = analogRead(A0); // Ruhe-Spulenspannung
  meanCoilB = analogRead(A1);
  printCoilValues(meanCoilA, meanCoilB);
}

void setupPortAndPlotter()
{
  DDRB = 0x2F;  // Motor und LED sind output
  DDRD = 0x0C;  // Tasten sind input, Spulen output
  PORTD = 0x60; // Pullups für die Tasten

  Serial.begin(9600);
#ifdef PlotterOutput
  p.Begin(); // start plotter
  p.AddTimeGraph("Spannung coilAValue", 1500, "CoilA", coilAValue, "CoilB", coilBValue);
#endif
}

void readCoilValuesInLoop()
{
  do
  {
    coilAValue = analogRead(A0);
    coilBValue = analogRead(A1);
    delay(3);
    plot();
    printCoilValues(coilAValue, coilBValue);
  } while (1);
}

int calculatePeriod()
{
  long coilBTouches[8];

  for (int i = 0; i < 8; i++)
  {
    int valB = analogRead(A1);
    while (analogRead(A1) < (meanCoilB + 8))
    { // Magnet ist über CoilA
      valB = analogRead(A1);
    }
    coilBTouches[i] = millis();
    while (analogRead(A1) > (meanCoilB - 8))
    { // Warte bis Magnet weg ist
    }

#ifdef Monitor
    Serial.print("coilBTouche ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(coilBTouches[i]);
#endif
  }
  return (coilBTouches[6] - coilBTouches[2]) >> 1; // Periode abschätzen
}

void initMotorTimer()
{
  motorTimerValue = ((long)period * 2000) / (512 + pauseAtTransit - 1);

  Timer1.initialize(motorTimerValue >> 2);
}
