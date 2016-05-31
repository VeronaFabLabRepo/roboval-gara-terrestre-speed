
/*
 * @scuola:	IIS SILVA RICCI
 * @autori:	-
 * Squadra: Iannone6
 * Robot:   -
 * @data:   Maggio 2016
 * Versione: - 
 * @evento: Roboval 2016
 * 
 */

#include <QTRSensors.h>
#define Kp 0.53 //0.55
#define Kd 1.65 //1.55

#define out_STBY    7
#define out_B_PWM   10 //B motore destro
#define out_A_PWM   5  //A motore sinistro
#define out_A_IN2   6
#define out_A_IN1   4
#define out_B_IN1   8
#define out_B_IN2   9
#define left_motor  0
#define right_motor 1

#define leftFar          0
#define leftCenter       1
#define leftNear         2
#define rightNear        3
#define rightCenter      4
#define rightFar         5
#define NUM_SENSORS      6
#define TIMEOUT       2500
#define EMITTER_PIN   QTR_NO_EMITTER_PIN

int maxSpeed=100; //100

#define baseSpeed 100 

//versione 2
int stato,sp;
int scu;  // stati consecutivi uguali
#define SOG_ST 10
#define ST_N  0
#define ST_CD 1
#define ST_CS 2
#define ST_T 3
#define ST_V 4

int lastDebugTime = 0;
int lastError = 0;

int time=0;
int rightMotorSpeed;
int leftMotorSpeed;

QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5}
,NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

void setup(){
  Serial.begin(9600);
  pinMode(out_STBY,OUTPUT);
  pinMode(out_A_PWM,OUTPUT);
  pinMode(out_A_IN1,OUTPUT);
  pinMode(out_A_IN2,OUTPUT);
  pinMode(out_B_PWM,OUTPUT);
  pinMode(out_B_IN1,OUTPUT);
  pinMode(out_B_IN2,OUTPUT);
  for (int i = 0; i < 400; i++){
    qtrrc.calibrate();
  }
  Serial.print("fine calibrazione");
  motor_standby(false);
  /* gestione stato per evitare i rumori*/
  stato=0;
  sp=1000;
  scu=0;

}



void loop()
{

 /* boolean cd= false; //curva a destra
  boolean cs = false;//curva a sinistra
  boolean t = false;//e t
  boolean v= false;//vuoto*/

  unsigned int sensors[6];
  int aposition = qtrrc.readLine(sensors); 
  int error = aposition - 2500;


  int motorSpeed = (int)(Kp * error + Kd * (error - lastError));
    rightMotorSpeed = baseSpeed - motorSpeed;
    leftMotorSpeed = baseSpeed + motorSpeed;
  
  

  
  

  if (rightMotorSpeed > maxSpeed ) rightMotorSpeed = maxSpeed; 
  if (leftMotorSpeed > maxSpeed ) leftMotorSpeed = maxSpeed; 
  if (rightMotorSpeed < -maxSpeed) rightMotorSpeed = -maxSpeed; 
  if (leftMotorSpeed < -maxSpeed) leftMotorSpeed = -maxSpeed; 
  
  motors(leftMotorSpeed,rightMotorSpeed);
  lastError = error;
}


void motors(int l, int r){
  set_motor(right_motor,r);
  set_motor(left_motor,l);
}
void set_motor(boolean motor, char speed) { // imposta la velocitÃ  tra -100 (indietro) e +100 (avanti)
  byte PWMvalue=0;
  PWMvalue = map(abs(speed),0,100,50,255);
  if (speed > 0)
    motor_speed(motor,0,PWMvalue);
  else if (speed < 0)
    motor_speed(motor,1,PWMvalue);
  else {
    motor_coast(motor);
  }
}
void motor_speed(boolean motor, boolean direction, byte speed) { // imposta la velocitÃ  tra 0 e 255
  if (motor == left_motor) {
    if (direction == 0) {
      digitalWrite(out_A_IN1,HIGH);
      digitalWrite(out_A_IN2,LOW);
    } 
    else {
      digitalWrite(out_A_IN1,LOW);
      digitalWrite(out_A_IN2,HIGH);
    }
    analogWrite(out_A_PWM,speed);
  } 
  else {
    if (direction == 0) {
      digitalWrite(out_B_IN1,HIGH);
      digitalWrite(out_B_IN2,LOW);
    } 
    else {
      digitalWrite(out_B_IN1,LOW);
      digitalWrite(out_B_IN2,HIGH);
    }
    analogWrite(out_B_PWM,speed);
  }
}
void motor_standby(boolean state) { // abilita/disabilita i motori
  if (state == true)
    digitalWrite(out_STBY,LOW);
  else
    digitalWrite(out_STBY,HIGH);
}

void motor_coast(boolean motor) { // motore in folle
  if (motor == left_motor) {
    digitalWrite(out_A_IN1,LOW);
    digitalWrite(out_A_IN2,LOW);
    digitalWrite(out_A_PWM,HIGH);
  } 
  else {
    digitalWrite(out_B_IN1,LOW);
    digitalWrite(out_B_IN2,LOW);
    digitalWrite(out_B_PWM,HIGH);
  }
}



