#include <QTRSensors.h>


#define NUM_SENSORS  6     // number of sensors used
#define TIMEOUT      2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN  2      // emitter is controlled by digital pin 2

#define soglia       200 

#define rightMotor1 8
#define rightMotor2 9
#define rightMotorPWM 10
#define leftMotor1 6
#define leftMotor2 4
#define leftMotorPWM 5
#define motorPower 7

QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4, A5}
,NUM_SENSORS, TIMEOUT, EMITTER_PIN); 

unsigned int sensorValues[NUM_SENSORS];

//float error=0;
float lastError=0;
float PV =0;
float kp = 0;
float kd =0;
int m1Speed=0;
int m2Speed=0;
float line_position=0;

void setup()
{
  Serial.begin(115200);

  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);
  
  analogWrite(rightMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
  
 for (int i = 0; i < 100; i++)
  {
    qtrrc.calibrate();
    delay (20);
  }
} 

void loop()
{
  unsigned int sensors[6];
  
  int position = qtrrc.readLine(sensors);
  int error = position - 2500;
  
  kp= 15;
  kd= 150;
 
  PV = kp * error + kd * (error - lastError);
  lastError = error;
  
  m1Speed = 170 + PV;
  m2Speed = 170 - PV;
  

  if (m1Speed < 0) m1Speed = 0; 
  if (m2Speed < 0) m2Speed = 0; 
  if (m1Speed > 220) m1Speed = 220;
  if (m2Speed > 220) m2Speed = 220;
  
  {
  digitalWrite(motorPower, HIGH);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, m1Speed);
  digitalWrite(motorPower, HIGH);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, m2Speed);
  }
  
  Serial.println("\n");
  Serial.println(PV);
  
}

