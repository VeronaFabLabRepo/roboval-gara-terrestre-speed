/*

 * @scuola:  Lavini Mondin

 * @autori: Matteo Bifone, Alessandro Palazzo, Michele Manara
 * 
 * Squadra: Crosby 

 * Robot:  Hope 

 * @data:   Maggio 2017

 * Versione: - 

 * @evento: Roboval 2017

 * 

 */





#include <QTRSensors.h>

float Kp=0.03; // 0.3

float Kd=1.9; //2.5  0.85

float Ki=0.000; //0.005

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



#define DEBFULL 0

#define SOGLIA 200 //  -----------------------------------------------------------------------------------------------------------------------------------------



#define MAXSP  225// Max speeed -------------------------------------------------------------------------------------------------------------------------------

#define MINSP  60 // Min speed ---------------------------------------------------------------------------------------------------------------------------------



int maxSpeed=100; //di default 100 max speed of the robot

int baseSpeed=100;//di default 50 this is the speed at which the motors should spin when the robot is perfectly on the line

#define soglia 200



#define SOG_ST 10

#define ST_N  0

#define ST_CD 1

#define ST_CS 2

#define ST_T 3

#define ST_V 4





int lc,ln,lf,rc,rn,rf;

int lastDebugTime = 0;

int lastError = 0;

int I=0;

int time=0;

int rightMotorSpeed;

int leftMotorSpeed;

int scarto=600;



QTRSensorsRC qtrrc((unsigned char[]) {

  A0, A1, A2, A3, A4, A5}

,NUM_SENSORS, TIMEOUT, EMITTER_PIN); 

unsigned int sensorValues[NUM_SENSORS];



int matrice[6][6];

int i;//valore per la matrice

int sensori[6];

int numCamp=0;          // num campionamenti

unsigned long lastMs,currMs;  

int ps=0;

char riga[100];





void setup(){

  Serial.begin(9600);

  pinMode(out_STBY,OUTPUT);

  pinMode(out_A_PWM,OUTPUT);

  pinMode(out_A_IN1,OUTPUT);

  pinMode(out_A_IN2,OUTPUT);

  pinMode(out_B_PWM,OUTPUT);

  pinMode(out_B_IN1,OUTPUT);

  pinMode(out_B_IN2,OUTPUT);

  Serial.println("inizio");

  for (int i = 0; i < 400; i++){

    qtrrc.calibrate();

  }

  Serial.print("finio");

  motor_standby(false);

  numCamp=0;

  lastMs=millis();

}







void loop()

{

  numCamp++;

  currMs=millis();

  if (currMs-lastMs >=1000){

    Serial.print("NC");

    Serial.println(numCamp);

    lastMs=currMs;

    numCamp=0;    



  }



  unsigned int sensors[6];

  int aposition = qtrrc.readLine(sensors); 

  int error = aposition - 2500;



  int motorSpeed = (int)(Kp * error + Kd * (error - lastError) + I * Ki);

  lastError = error;

  I=I+(error*0.01);

  //Serial.println(I);

  rightMotorSpeed = baseSpeed - motorSpeed;

  leftMotorSpeed =  baseSpeed + motorSpeed;



  if (rightMotorSpeed > maxSpeed ) rightMotorSpeed = maxSpeed; 

  if (leftMotorSpeed > maxSpeed ) leftMotorSpeed = maxSpeed; 

  if (rightMotorSpeed < -maxSpeed) rightMotorSpeed = -maxSpeed; 

  if (leftMotorSpeed < -maxSpeed) leftMotorSpeed = -maxSpeed; 

  motors(leftMotorSpeed,rightMotorSpeed);



  if (Serial.available()>0){

    int c=Serial.read();

    if (c!='\n')

      riga[ps++]=c;

    else{

      riga[ps]=0;

      //Serial.print(riga);

      float f=atof(riga+1);

      ps=0;

      if (riga[0]=='I') {

        Ki=f; 

        Serial.println("\nPIDM");

        Serial.println(Kp);

        Serial.println(Ki);

        Serial.println(Kd);

        Serial.println(maxSpeed);



      }

      if (riga[0]=='D') {

        Kd=f;

        Serial.println("\nPIDM");

        Serial.println(Kp);

        Serial.println(Ki);

        Serial.println(Kd);

        Serial.println(maxSpeed);

      }

      if (riga[0]=='P') {

        Kp=f;

        Serial.println("\nPIDM");

        Serial.println(Kp);

        Serial.println(Ki);

        Serial.println(Kd);

        Serial.println(maxSpeed);

      }

      if (riga[0]=='M') {

        maxSpeed=(int)f;

        Serial.println("\nPIDM");

        Serial.println(Kp);

        Serial.println(Ki);

        Serial.println(Kd);

        Serial.println(maxSpeed);



      }

      if (riga[0]=='B') {

        baseSpeed=(int)f;

        Serial.println("\nPIDM");

        Serial.println(Kp);

        Serial.println(Ki);

        Serial.println(Kd);

        Serial.println(maxSpeed);

        Serial.println(baseSpeed);



      }      

    }

  }   





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



void motor_brake(boolean motor) { // freno motore

  if (motor == left_motor) {

    digitalWrite(out_A_IN1,HIGH);

    digitalWrite(out_A_IN2,HIGH);

  } 

  else {

    digitalWrite(out_B_IN1,HIGH);

    digitalWrite(out_B_IN2,HIGH);

  }

}
