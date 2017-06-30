#include <QTRSensors.h>

static const int OUT_LEFT_IN1=4;
static const int OUT_LEFT_PWM=5;
static const int OUT_LEFT_IN2=6;
static const int OUT_STBY=7;
static const int OUT_RIGHT_IN1=8;
static const int OUT_RIGHT_IN2=9;
static const int OUT_RIGHT_PWM=10;

static const boolean LEFT_MOTOR=false;
static const boolean RIGHT_MOTOR=true;

static const unsigned char NUM_SENSORS=6;                   //numero dei sensori
static const unsigned int TIMEOUT=2500;                   //tempo di debounce per full black in us

static const byte MAX_SPEED=255;                   //velocita' massima dei motori
static const byte MIN_SPEED=50;
static const byte MID_SPEED=255;                   //velocità di crociera

static const float Kp=0.075;               // costante PID proporzionale   default=0.25
static const float Kd=2.1;                 // costante PID differenziale    default=2.1
static const float Ki=0;                   // costante PID integrale         default=0

static const boolean FORWARD=false;
static const boolean BACKWARD=true;
 
QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4, A5}, NUM_SENSORS, TIMEOUT);
unsigned int sensors[6];      //array delle letture raw dei sensori

inline int dist(){
  return qtrrc.readLine(sensors)-2500;
  }

inline void set_motors(int cSpeed)
  {
  set_motor(RIGHT_MOTOR,limitSpeed(MID_SPEED-cSpeed));
  set_motor(LEFT_MOTOR,limitSpeed(MID_SPEED+cSpeed));
  }

inline int limitSpeed(int rSpeed)
  {
  if(MAX_SPEED<rSpeed)
    return MAX_SPEED;
  else if(0<=rSpeed&&rSpeed<MIN_SPEED)
    return MIN_SPEED;
  else if(-MIN_SPEED<rSpeed&&rSpeed<0)
    return -MIN_SPEED;
  else if(rSpeed<-MAX_SPEED)
    return -MAX_SPEED;
  else
    return rSpeed;
  }
  
inline boolean dir(int lSpeed)
  {
  return lSpeed<0;
  }

inline byte power(int lSpeed)
  {
  return abs(lSpeed);
  }

inline void set_motor(boolean motor,int lSpeed)
  {
  motor_speed(motor,dir(lSpeed),power(lSpeed));
  }

inline void motor_speed(boolean right_motor,boolean forward,byte PWMvalue)
  {
  if(right_motor)
    {
    if(forward)
      {
      digitalWrite(OUT_RIGHT_IN1,LOW);
      digitalWrite(OUT_RIGHT_IN2,HIGH);
      }
    else //backward
      {
      digitalWrite(OUT_RIGHT_IN1,HIGH);
      digitalWrite(OUT_RIGHT_IN2,LOW);
      } 
    analogWrite(OUT_RIGHT_PWM,PWMvalue);
    }
  else //left_motor
    {
    if(forward)
      {
      digitalWrite(OUT_LEFT_IN1,LOW);
      digitalWrite(OUT_LEFT_IN2,HIGH);
      }
    else //backward
      {
      digitalWrite(OUT_LEFT_IN1,HIGH);
      digitalWrite(OUT_LEFT_IN2,LOW);
      } 
    analogWrite(OUT_LEFT_PWM,PWMvalue);
    }
  }

inline void motor_standby(boolean standby) //abilita-disabilita i motori
  {
  if(standby)
    digitalWrite(OUT_STBY,LOW);
  else
    digitalWrite(OUT_STBY,HIGH);
  }

void setup()
  {
  Serial.begin(9600); 
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
  
int integrale=0;
int differenziale=0;        
int errore=0;
int errorePrecedente=0;
int cSpeed=0;

void loop()
  {
  errore=dist();
  integrale=integrale+errore;                
  differenziale=errore-errorePrecedente;
  cSpeed=Kp*errore+Ki*integrale+Kd*differenziale;
  set_motors(cSpeed);
  errorePrecedente=errore;
  }
