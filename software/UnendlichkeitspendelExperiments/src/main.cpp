#include <Arduino.h>

#include "TimerOne.h"
#include "Plotter.h"
#include "StateMachine.h"

#define PinCoilA 2
#define PinCoilB 3
#define LED 13
#define LOOP_DELAY 3
#define COIL_ON_TIME 100
#define COIL_RELAX_TIME 150
#define OUTER_PENDEL_DELAY 150
#define MOTOR_START_POS 360 // so wählen, dass comparePosition ~250 ist
#define COIL_MIDDLE_COMPENSATION 85
#define MOTOR_TIMER_VALUE 0 // Wenn bekannt: Wert hier eintragen

const int STATE_DELAY = 3;

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
void coilControl();
void initMotorTimer();
void handleMotorControl();
void plotCoilValues();
void switchOffBothCoils();
void switchOnCoilB();
void switchOnCoilA();
void enterExternalState();
void enterInternalState();
bool detectCoilAPass();
bool detectCoilBPass();
bool checkOnTime();
bool checkCoilRelaxTime();

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
unsigned long lastSwitchOn;
unsigned long startOfCoilRelaxTime;
int comparePosition;
long passage;
long meanPassage;
long varianzPassage;
byte direction = 1;
bool isMotorControl = false;
bool isMotorDelay = false;
bool isShutdown = false;

StateMachine machine = StateMachine();

State *EXTERNAL_PASS = machine.addState(&enterExternalState);
State *COIL_B_ON = machine.addState(&switchOnCoilB);
State *COIL_A_ON = machine.addState(&switchOnCoilA);
State *INTERNAL_PASS = machine.addState(&enterInternalState);

void setup()
{
  setupPortAndPlotter();
  readMeanVoltage();
  // readCoilValuesInLoop();
  // manualCoilControl();
  // coilControl();

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

  EXTERNAL_PASS->addTransition(&detectCoilAPass, COIL_B_ON);
  EXTERNAL_PASS->addTransition(&detectCoilBPass, COIL_A_ON);
  COIL_A_ON->addTransition(&checkOnTime, INTERNAL_PASS);
  COIL_B_ON->addTransition(&checkOnTime, INTERNAL_PASS);
  INTERNAL_PASS->addTransition(&checkCoilRelaxTime, EXTERNAL_PASS);

  machine.transitionTo(EXTERNAL_PASS);
}

void loop()
{
  unsigned long currentMillis;
  currentMillis = millis();

  machine.run();
  delay(STATE_DELAY);

  // switchOffBothCoils();
  // detectCoilAPass(currentMillis);
  // detectCoilBPass(currentMillis);
  // plotCoilValues();
}

void enterExternalState()
{
  if (machine.executeOnce)
  {
    Serial.println("enterExternalState:");
    switchOffBothCoils();
    startOfCoilRelaxTime = 0;
  }
}

void enterInternalState()
{
  if (machine.executeOnce)
  {
    Serial.println("enterInternalState:");
    switchOffBothCoils();
    lastSwitchOn = 0;
    startOfCoilRelaxTime = millis();
  }
}

void switchOnCoilA()
{
  if (machine.executeOnce)
  {
    digitalWrite(PinCoilA, HIGH);
    Serial.println("CoilA ON: ");
    lastSwitchOn = millis;
  }
}

void switchOnCoilB()
{
  if (machine.executeOnce)
  {
    digitalWrite(PinCoilB, HIGH);
    Serial.println("CoilB ON: ");
    lastSwitchOn = millis;
  }
}

bool checkOnTime()
{
  if ((millis() - lastSwitchOn) > COIL_ON_TIME)
  {
    return true;
  }
  return false;
}

bool checkCoilRelaxTime()
{
  if ((millis() - startOfCoilRelaxTime) > COIL_RELAX_TIME)
  {
    return true;
  }
  return false;
}

bool detectCoilAPass()
{
  coilAValue = analogRead(A0);      // Lese CoilA Spannung
  if (coilAValue < (meanCoilA - 8)) // Magnet über CoilA ?
  {
    Serial.print("CoilA pass: ");
    Serial.println(millis() - lastTouchA);
    return true;
  }
  return false;
}

bool detectCoilBPass()
{
  coilBValue = analogRead(A1);      // Lese CoilB Spannung
  if (coilBValue < (meanCoilB - 8)) // ist Pendel über CoilB ?
  {
    Serial.print("CoilB pass: ");
    Serial.println(millis() - lastTouchB);
    return true;
  }
  return false;
}

void switchOffBothCoils()
{
  Serial.println("All switched Off");
  PORTD = (PORTD & 0xF3); // PinCoilA und PinCoilB ausschalten
}

void loopOriginal()
{
  long delta1;
  long delta2;
  unsigned long currentMillis;

  currentMillis = millis();

  /******************************************/
  /* Behandle Coil A                          */
  /******************************************/
  coilAValue = analogRead(A0); // Lese CoilA Spannung
  if (pendelTimerStart == 0)   // Wenn CoilA nicht gefeuert hat, dann:
  {
    if (coilAValue < (meanCoilA - 8)) // Magnet über CoilA ?
    {
      delta1 = currentMillis - lastTouchA;
      lastTouchA = currentMillis;
      if (delta1 > (period >> 1))
      {
        digitalWrite(PinCoilB, HIGH);
        pendelTimerStart = currentMillis;
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
        handleMotorControl();
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
      delta1 = currentMillis - lastTouchB;
      lastTouchB = currentMillis;
      if (delta1 > (period >> 1))
      {
        digitalWrite(PinCoilA, HIGH);
        pendelTimerStart = currentMillis;
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
    pendelTimerValue = currentMillis - pendelTimerStart;
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

void handleMotorControl()
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

/******************************************/
/* ISR Schrittmotor                            */
/******************************************/
void stepper() // Interrupt Routine, die von Timer 1 alle x Microsekunden (motorTimerValue) aufgerrufen wird.
{
  /*   static byte Steps;
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
   */
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

void doLog()
{
  coilAValue = analogRead(A0);
  coilBValue = analogRead(A1);
  Serial.print("MeanCoil A/B: ");
  Serial.print(coilAValue);
  Serial.print(" / ");
  Serial.print(coilBValue);
  Serial.print(" t=");
  Serial.println(millis());
}
void coilControl()
{
  int coilOnTime = 40;
  int offTime = 1225 / 2 - 2 * coilOnTime;

  do
  {
    digitalWrite(PinCoilA, HIGH);
    delay(coilOnTime);
    doLog();
    digitalWrite(PinCoilA, LOW);
    delay(offTime);
    doLog();

    digitalWrite(PinCoilB, HIGH);
    delay(coilOnTime);
    doLog();
    digitalWrite(PinCoilB, LOW);
    delay(offTime);
    doLog();

  } while (1);
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
  p.AddTimeGraph("Spannung coilAValue", 400, "CoilA", coilAValue, "CoilB", coilBValue);
#endif
}

void plotCoilValues()
{
  coilAValue = analogRead(A0);
  coilBValue = analogRead(A1);
  delay(3);
}

void readCoilValuesInLoop()
{
  do
  {
    plotCoilValues();
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
    Serial.print("coilBTouches ");
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
