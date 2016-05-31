
/*
* Squadra: Ferraris 1
* Robot:   Bea
* @autori: Carlini,Poletto,Cestone,Bertolasi
* @scuola: Ferraris
* @data: Maggio 2016
* @versione: -
* @evento: Roboval 2016
* 
*/


#include <QTRSensors.h>

#define black 600
#define rotation 25
#define line 90

#define out_STBY 7
#define out_B_PWM 10 //B motore destro
#define out_A_PWM 5 //A motore sinistro
#define out_A_IN2 6
#define out_A_IN1 4
#define out_B_IN1 8
#define out_B_IN2 9
#define left_motor 0
#define right_motor 1

//SENSORI
#define leftFar          0
#define leftNear         1
#define leftCenter       2
#define rightCenter      3
#define rightNear        4
#define rightFar         5
#define NUM_SENSORS      6
#define TIMEOUT       2500
#define EMITTER_PIN   QTR_NO_EMITTER_PIN


// dichiarazione variabili globali
int lastDebugTime = 0;
int error, lastError = 0;
int m1Speed, m2Speed;
int state = 0;
int motorSpeed;
unsigned int position;

QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5
}
, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

/**
  set di funzioni

*/
void set_motor(boolean motor, char speed) { // imposta la velocità tra -100 (indietro) e +100 (avanti)
  byte PWMvalue = 0;
  PWMvalue = map(abs(speed), 0, 100, 50, 255);
  if (speed > 0)
    motor_speed(motor, 0, PWMvalue);
  else if (speed < 0)
    motor_speed(motor, 1, PWMvalue);
  else {
    motor_coast(motor);
  }
}

void motor_speed(boolean motor, boolean direction, byte speed) { // imposta la velocità tra 0 e 255
  if (motor == left_motor) {
    if (direction == 0) {
      digitalWrite(out_A_IN1, HIGH);
      digitalWrite(out_A_IN2, LOW);
    }
    else {
      digitalWrite(out_A_IN1, LOW);
      digitalWrite(out_A_IN2, HIGH);
    }
    analogWrite(out_A_PWM, speed);
  }
  else {
    if (direction == 0) {
      digitalWrite(out_B_IN1, HIGH);
      digitalWrite(out_B_IN2, LOW);
    }
    else {
      digitalWrite(out_B_IN1, LOW);
      digitalWrite(out_B_IN2, HIGH);
    }
    analogWrite(out_B_PWM, speed);
  }
}

void motor_standby(boolean state) { // abilita/disabilita i motori
  if (state == true)
    digitalWrite(out_STBY, LOW);
  else
    digitalWrite(out_STBY, HIGH);
}

void motor_coast(boolean motor) { // motore in folle
  if (motor == left_motor) {
    digitalWrite(out_A_IN1, LOW);
    digitalWrite(out_A_IN2, LOW);
    digitalWrite(out_A_PWM, HIGH);
  }
  else {
    digitalWrite(out_B_IN1, LOW);
    digitalWrite(out_B_IN2, LOW);
    digitalWrite(out_B_PWM, HIGH);
  }
}

void motor_brake(boolean motor) { // freno motore
  if (motor == left_motor) {
    digitalWrite(out_A_IN1, HIGH);
    digitalWrite(out_A_IN2, HIGH);
  }
  else {
    digitalWrite(out_B_IN1, HIGH);
    digitalWrite(out_B_IN2, HIGH);
  }
}


void setup() {
  Serial.begin(9600);
  motor_standby(false);
  pinMode(out_STBY, OUTPUT);
  pinMode(out_A_PWM, OUTPUT);
  pinMode(out_A_IN1, OUTPUT);
  pinMode(out_A_IN2, OUTPUT);
  pinMode(out_B_PWM, OUTPUT);
  pinMode(out_B_IN1, OUTPUT);
  pinMode(out_B_IN2, OUTPUT);
  Serial.println("calib");
  for (int i = 0; i < 200; i++) {
    qtrrc.calibrate();
  }
}

void loop() {

      position = qtrrc.readLine(sensorValues);
      error = position - 2500;
      motorSpeed = 0.1 * error + 5 * (error - lastError);
      lastError = error;
      m1Speed = line + motorSpeed;
      m2Speed = line - motorSpeed;
      if (m1Speed < 10)
        m1Speed = 10;
      if (m2Speed < 10)
        m2Speed = 10;
      if (m1Speed > 75)
        m1Speed = 100;
      if (m2Speed > 75)
        m2Speed = 100;
      set_motor(left_motor, m1Speed);
      set_motor(right_motor, m2Speed);

      qtrrc.readLine(sensorValues);

}



