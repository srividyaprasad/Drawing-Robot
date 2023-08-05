#include "CytronMotorDriver.h"

const byte EnA_LF = 7; //interrupt
const byte EnB_LF = 8;
const byte EnA_RF = ; //interrupt
const byte EnB_RF = ;
const byte EnA_LB = ; //interrupt
const byte EnB_LB = ;
const byte EnA_RB = ; //interrupt
const byte EnB_RB = ;

int n=LOW;
int count=0;

CytronMD LF(PWM_PWM, 1,0); 
CytronMD RF(PWM_PWM, 2,3);
CytronMD LB(PWM_PWM, 5,4);
CytronMD RB(PWM_PWM, 10,11);

void setup() 
{ 
  pinMode(EnA, INPUT_PULLUP);
  pinMode(EnB, INPUT);
  attachInterrupt(digitalPinToInterrupt(EnA), interrupt, CHANGE);
  Serial.begin(9600);
}

void loop() 
{LF.setSpeed(255.0/5.0);
  delay(500);}

void interrupt()
{
  n=digitalRead(EnB);
  if (n==HIGH)
  {
    count=count+1;
  }
  else
  {
    count=count-1;
  }
  Serial.println(count);
}
