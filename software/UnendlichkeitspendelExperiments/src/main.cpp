// #include <Arduino.h>

#include "TimerOne.h"
#include "Plotter.h"

#define PinCoilA 2
#define PinCoilB 3
#define LED 13
#define LOOP_DELAY 3
#define COIL_ON_TIME 100
#define COIL_RELAX_TIME 250
#define MOTOR_START_POS 360 // so wählen, dass ComparePosition ~250 ist
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

int coilAValue;
int coilBValue;
int meanCoilA;
int meanCoilB;
int period;

Plotter p; // create plotter

int MotorTimerValue = MOTOR_TIMER_VALUE;
int MotorPosition; // Motor Position beim Transit

bool MotorRunning = false;
long PendelTimerStart = 0;
long PendelTimerValue;
unsigned long LastTouchA;
unsigned long LastTouchB;
int ComparePosition;
long Passage;
long MeanPassage;
long VarianzPassage;
byte Direction = 1;
bool MotorControl = false;
bool MotorDelay = false;
byte Shutdown = 0;
byte PauseAtTransit = 0;

void setup()
{
  setupPortAndPlotter();
  readMeanVoltage();
  //readCoilValuesInLoop();

  period = calculatePeriod();

  if (MotorTimerValue == 0)
    MotorTimerValue = ((long)period * 2000) / (512 + PauseAtTransit - 1);
  else
    MotorTimerValue = MotorTimerValue << 2; // Wenn MotorTimerValue bekannt ist (Deklaration) diesen nehmen !!!!

  Timer1.initialize(MotorTimerValue >> 2);

#ifdef Monitor
  Serial.println("Periode  MotorTimerValue");
  Serial.print(period);
  Serial.print(",");
  Serial.println(MotorTimerValue >> 2);
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
  if (PendelTimerStart == 0)   // Wenn CoilA nicht gefeuert hat, dann:
  {
    if (coilAValue < (meanCoilA - 8)) // Magnet über CoilA ?
    {
      delta1 = CurrentMillis - LastTouchA;
      LastTouchA = CurrentMillis;
      if (delta1 > (period >> 1))
      {
        digitalWrite(PinCoilB, HIGH);
        PendelTimerStart = CurrentMillis;
        Passage = (LastTouchA - LastTouchB) >> 1;
        if (!MotorRunning)
        {                                        // Berechne die Varianz der Passage, um stabilität des Pendulums zu eruieren
          delta1 = Passage - (MeanPassage >> 5); // Abweichung old Mean
          MeanPassage = MeanPassage + delta1;
          delta2 = Passage - (MeanPassage >> 5); // Abweichung new Mean
          VarianzPassage = (VarianzPassage - VarianzPassage >> 5) + delta1 * delta2;

#ifdef Monitor
          Serial.print(Passage);
          Serial.print(",");
          Serial.println(VarianzPassage);
#endif
        }
      }
      else // delta1 < (Period >> 1)
      {
        if (MotorControl == true)
        {
          MotorControl = false;
          if (MotorRunning)
          {
            if (MotorPosition > ComparePosition)
            {
              MotorTimerValue = MotorTimerValue + 1;
              Timer1.setPeriod(MotorTimerValue >> 2);
            }
            if (MotorPosition < ComparePosition)
            {
              stepper();
              MotorTimerValue = MotorTimerValue - 1;
              Timer1.setPeriod(MotorTimerValue >> 2);
            }
#ifdef Monitor
            Serial.print(MotorPosition);
            Serial.print(",");
            Serial.println(MotorTimerValue >> 2);
#endif
          }
          else //  if (!MotorRunning)
          {
            if (VarianzPassage < 100) // Warte, bis Pendel stabil schwingt, befor Motor gestartet wird
            {
#ifndef PlotterOutput
              MotorRunning = true;
              MotorPosition = MOTOR_START_POS;              // Hier müssen wir bei dem nächsten Transit wieder hin!
              Passage = Passage + COIL_MIDDLE_COMPENSATION; // Bei Motorstart erstmalig die Kompensation berücksichtigen
              Direction = 1;
              delay(Passage);                                                            // Warte, bis Pendel im Transit ist
              Timer1.attachInterrupt(stepper);                                           // Und starte motor
              ComparePosition = MotorPosition - (int)(((long)(Passage << 10)) / period); // Theoretische Motor Position kurz vorher, als Pendel über CoilA war
#endif

#ifdef Monitor
              Serial.println(" ");
              Serial.println("Motor Started");
              Serial.println("ComparePosition");
              Serial.print(ComparePosition);
              Serial.println(" ");
              Serial.println("ComparePosition  MotorTimerValue");
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
  if (PendelTimerStart == 0)   // Wenn CoilB nicht gefeuert hat, dann:
  {
    if (coilBValue < (meanCoilB - 8)) // ist Pendel über CoilB ?
    {
      delta1 = CurrentMillis - LastTouchB;
      LastTouchB = CurrentMillis;
      if (delta1 > (period >> 1))
      {
        digitalWrite(PinCoilA, HIGH);
        PendelTimerStart = CurrentMillis;
      }
      else
        MotorControl = true;
    }
  }

  /******************************************/
  /* Pendel Timer -> Spule aktivieren       */
  /******************************************/
  if (PendelTimerStart > 0)
  {
    PendelTimerValue = CurrentMillis - PendelTimerStart;
    if (PendelTimerValue > COIL_RELAX_TIME)
      PendelTimerStart = 0; // Stoppe timer und ermögliche Sensornutzung der Spulen
    else
    {
      if (PendelTimerValue > COIL_ON_TIME)
        PORTD = (PORTD & 0xF3); // PinCoilA und PinCoilB ausschalten
    }
  }

  if (CheckButtons() == 2) // Shutdown starten, wenn beide Tasten gedrückt
    Shutdown = 1;          // Motor stop in Transit Position

#ifdef PlotterOutput
  Serial.print(coilBValue);
  Serial.print(",");
  Serial.print(meanCoilA);
  Serial.print(",");
  Serial.print(PendelTimerValue << 1);
  Serial.print(",");
  Serial.println(coilAValue);
#endif

  delay(LOOP_DELAY);
}

/******************************************/
/* ISR Schrittmotor                            */
/******************************************/
void stepper() // Interrupt Routine, die von Timer 1 alle x Microsekunden (MotorTimerValue) aufgerrufen wird.
{
  static byte Steps;
  static byte Pause;
  if (!MotorDelay)
  {
    if ((MotorPosition != MOTOR_START_POS) || (Pause == 0)) // Sind wir nicht im Transit?
    {                                                       // dann Motor weiterdrehen
      digitalWrite(LED, LOW);                               // Stroboskop LED ausschalten
      Steps = (Steps + Direction) & 0x07;
      MotorPosition = (MotorPosition + (int)Direction);
      MotorPosition = MotorPosition & 0x01FF; // Wir werten nur die relative motor position aus
      Pause = PauseAtTransit;                 // Pausenlänge im Transit in "IR counts"
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
    if (MotorPosition == MOTOR_START_POS) // in TransitPosition!!!
    {
      digitalWrite(LED, HIGH); // Schalte Stoboskop LED ein
      if (Pause > 0)
        Pause = Pause - 1; // Pause im Transit
      if (Shutdown > 0)    // Wenn Shutdown gewünscht:
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
    MotorDelay = false;
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

  if (!MotorRunning) // Justierung der Lochscheiben Position bevor der Motor läuft
  {
    if ((button1 & 0x0F) == 0x0F)
    {
      Direction = -1;
      stepper();
    }
    if ((button2 & 0x0F) == 0x0F)
    {
      Direction = 1;
      stepper();
    }
  }
  else // Justierung der Lochscheiben Position nach Motorstart
  {
    if ((button1 & 0x1F) == 0x1F)
      MotorDelay = true;
    if ((button2 & 0x1F) == 0x1F)
      stepper();
  }
  return (bool)button1 + (bool)button2; // 0: no press, 1: one key, 2: both keys
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

void readMeanVoltage()
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
  long coilATouches[8];

  for (int i = 0; i < 8; i++)
  {
    int valB = analogRead(A1);
    while (analogRead(A1) < (meanCoilB + 8))
    { // Magnet ist über CoilA
      printCoilValues(0, valB);
      valB = analogRead(A1);
    }
    coilATouches[i] = millis();
    while (analogRead(A1) > (meanCoilB - 8))
    { // Warte bis Magnet weg ist
    }
  }
  return (coilATouches[6] - coilATouches[2]) >> 1; // Periode abschätzen
}