#include "TimerOne.h"

#define MOTOR_START_POS 300  // choose, so that ComparePosition is ~250

void stepper();       // ISR for stepper motor

int  MotorPosition = MOTOR_START_POS; // Motor Position beim Transit
byte PauseAtTransit = 0;
byte Direction = 1;

void setup()
{
}

void  loop() 
{
  DDRB = 0x2F;                      // Motor pins are output
  Timer1.initialize(1281);
  Timer1.attachInterrupt(stepper);  // And start motor
  while(1);
}
 
/******************************************/  
/* ISR Stepper                            */
/******************************************/
void stepper()    // Interrupt Routine, die von Timer 1 alle x Microsekunden (MotorSpeed) aufgerrufen wird.
{
  static byte Steps;
  static byte Pause;
  if ((MotorPosition != MOTOR_START_POS) || (Pause == 0))  // Check if we are in not in one of those 8 Transit positions
  {                                                      // then continue motor movement
    Steps = (Steps + Direction) & 0x07;
    MotorPosition = (MotorPosition + (int)Direction);
    MotorPosition = MotorPosition & 0x01FF;              // limit to 1/8 of all motor positions
    Pause = PauseAtTransit;                              // length of the Pause in the Transit position in IR counts
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
  if (MotorPosition == MOTOR_START_POS)   // in TransitPosition!!!
  {
    if (Pause > 0)
      Pause = Pause - 1;       // Take a rest at Transit
  }
 }
