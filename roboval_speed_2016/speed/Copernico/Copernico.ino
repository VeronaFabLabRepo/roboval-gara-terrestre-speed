
/*
* Squadra: Copernico
* Robot:   Only One Day
* @autori: Brosco Paolo, Ceoletta Marco, Rossiello Giordano, Saglia Paolo
* @scuola: Copernico-Pasoli
* @data: Maggio 2016
* @versione: -
* @evento: Roboval 2016
* 
*/

#include <QTRSensors.h>

#define OUT_LEFT_IN1    4
#define OUT_LEFT_PWM    5
#define OUT_LEFT_IN2    6
#define OUT_STBY        7
#define OUT_RIGHT_IN1   8
#define OUT_RIGHT_IN2   9
#define OUT_RIGHT_PWM  10
#define LEFT_MOTOR      0
#define RIGHT_MOTOR     1

#define NUM_SENSORS     6
#define TIMEOUT      2500
#define EMITTER_PIN     QTR_NO_EMITTER_PIN

#define MAX_SPEED     100 
#define BASE_SPEED    100

QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4, A5},NUM_SENSORS, TIMEOUT, EMITTER_PIN); 

  unsigned int sensors[6];
int lastError=0;
int motorSpeed=0;

void setup()
  {
  pinMode(OUT_STBY,OUTPUT);
  pinMode(OUT_LEFT_PWM,OUTPUT);
  pinMode(OUT_LEFT_IN1,OUTPUT);
  pinMode(OUT_LEFT_IN2,OUTPUT);
  pinMode(OUT_RIGHT_PWM,OUTPUT);
  pinMode(OUT_RIGHT_IN1,OUTPUT);
  pinMode(OUT_RIGHT_IN2,OUTPUT);
  for(int i=0;i<400;i++)
    qtrrc.calibrate();
  motor_standby(false);
  }

void motor_standby(boolean standby) //abilita-disabilita i motori
  {
  if(standby)
    digitalWrite(OUT_STBY,LOW);
  else
    digitalWrite(OUT_STBY,HIGH);
  }

void loop()
  {
  int aposition=qtrrc.readLine(sensors); 
  int error=aposition-2500;
//Ciclo di controllo PID (Proporzionale-Integrativo-Derivativo)
  motorSpeed=(int)(0.1*error+0.03*((float)(error+lastError)*0.5)+1.425*((error-lastError)));
  
  lastError=error;
  
  set_motor(LEFT_MOTOR,limitSpeed(BASE_SPEED+motorSpeed));
  set_motor(RIGHT_MOTOR,limitSpeed(BASE_SPEED-motorSpeed));
}

int limitSpeed(int motorSpeed) //limitatore di velocita
  {
  if(motorSpeed>MAX_SPEED)
    return MAX_SPEED;
  if(motorSpeed<-MAX_SPEED)
    return -MAX_SPEED;
  return motorSpeed;
  }

void set_motor(boolean motor,char speed) //imposta la velocità  da tra -100 (indietro) e +100 (avanti) a 0/255
  {
  byte PWMvalue=map(abs(speed),0,100,50,255);
  if(speed>0)
    motor_speed(motor,0,PWMvalue);
  else if(speed<0)
    motor_speed(motor,1,PWMvalue);
  else
    motor_coast(motor);
  }

void motor_speed(boolean motor,boolean direction,byte speed)//trasmetti velocita//trasmetti velocita
  {
  if(motor==LEFT_MOTOR)
    {
    if(direction==0)
      {
      digitalWrite(OUT_LEFT_IN1,HIGH);
      digitalWrite(OUT_LEFT_IN2,LOW);
      } 
    else
      {
      digitalWrite(OUT_LEFT_IN1,LOW);
      digitalWrite(OUT_LEFT_IN2,HIGH);
      }
    analogWrite(OUT_LEFT_PWM,speed);
    } 
  else //motor==RIGHT_MOTOR
    {
    if(direction==0)
      {
      digitalWrite(OUT_RIGHT_IN1,HIGH);
      digitalWrite(OUT_RIGHT_IN2,LOW);
      } 
    else
      {
      digitalWrite(OUT_RIGHT_IN1,LOW);
      digitalWrite(OUT_RIGHT_IN2,HIGH);
      }
    analogWrite(OUT_RIGHT_PWM,speed);
    }
  }

void motor_coast(boolean motor) //motore in folle
  { 
  if(motor==LEFT_MOTOR)
    {
    digitalWrite(OUT_LEFT_IN1,LOW);
    digitalWrite(OUT_LEFT_IN2,LOW);
    digitalWrite(OUT_LEFT_PWM,HIGH);
    } 
  else //motor==RIGHT_MOTOR
    {
    digitalWrite(OUT_RIGHT_IN1,LOW);
    digitalWrite(OUT_RIGHT_IN2,LOW);
    digitalWrite(OUT_RIGHT_PWM,HIGH);
    }
  }
