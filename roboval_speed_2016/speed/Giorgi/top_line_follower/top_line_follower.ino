
/*
* Squadra: SF16-H
* Robot:   Top Line Follower
* @autori: Venturi
* @scuola: Giorgi
* @data:   Maggio 2016
* @vrsione: -
* @evento: Roboval 2016
* 
*/


#include <QTRSensors.h>

#define Kp 0.026
#define Kd 2.50

#define rightMotor1 8
#define rightMotor2 9
#define rightMotorPWM 10
#define leftMotor1 6
#define leftMotor2 4
#define leftMotorPWM 5
#define motorPower 7

#define NUM_SENSORS      6
#define TIMEOUT       2500
#define EMITTER_PIN   QTR_NO_EMITTER_PIN

#define maxSpeed 100
#define baseSpeed 70

char left_motor = 'l', right_motor = 'r';
int position = 0, error = 0, motorSpeed = 0, lastError = 0, rightMotorSpeed = 0, leftMotorSpeed = 0;

QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4, A5}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup() {
	Serial.begin(115200);
	pinMode(motorPower, OUTPUT);
	pinMode(leftMotor1, OUTPUT);
	pinMode(leftMotor2, OUTPUT);
	pinMode(rightMotor1, OUTPUT);
	pinMode(rightMotor2, OUTPUT);

	setPwmFrequency(leftMotorPWM, 1);
	setPwmFrequency(rightMotorPWM, 1);

	int i;
	for (i = 0; i < 100; i++) {
		qtrrc.calibrate();
	}
	digitalWrite(motorPower, HIGH);
}

void loop() {
	unsigned int sensors[6];
	position = qtrrc.readLine(sensors);
	error = (float)position - 2500;

	motorSpeed = (Kp * error + Kd * (error - lastError));
	lastError = error;

	rightMotorSpeed = baseSpeed - motorSpeed;
	leftMotorSpeed = baseSpeed + motorSpeed;

	if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
	if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;
	if (rightMotorSpeed < -maxSpeed) rightMotorSpeed = -maxSpeed;
	if (leftMotorSpeed < -maxSpeed) leftMotorSpeed = -maxSpeed;

	set_motors_speed(left_motor, leftMotorSpeed);
	set_motors_speed(right_motor, rightMotorSpeed);
}

void set_motors_speed(char motor, int speed) {
	byte PWMvalue = 0;
	PWMvalue = map(abs(speed), 0, 100, 200, 255);
	if (speed > 0 && motor == 'l') {
		digitalWrite(leftMotor1, LOW);
		digitalWrite(leftMotor2, HIGH);
		analogWrite(leftMotorPWM, PWMvalue);
	}
	else if (speed < 0 && motor == 'l') {
		digitalWrite(leftMotor1, HIGH);
		digitalWrite(leftMotor2, LOW);
		analogWrite(leftMotorPWM, PWMvalue);
	}
	else if (speed > 0 && motor == 'r') {
		digitalWrite(rightMotor1, HIGH);
		digitalWrite(rightMotor2, LOW);
		analogWrite(rightMotorPWM, PWMvalue);
	}
	else if (speed < 0 && motor == 'r') {
		digitalWrite(rightMotor1, LOW);
		digitalWrite(rightMotor2, HIGH);
		analogWrite(rightMotorPWM, PWMvalue);
	}
	else {
		digitalWrite(leftMotor1, LOW);
		digitalWrite(leftMotor2, HIGH);
		digitalWrite(leftMotorPWM, 255);
		digitalWrite(rightMotor1, LOW);
		digitalWrite(rightMotor2, HIGH);
		digitalWrite(rightMotorPWM, 255);
	}
}

void setPwmFrequency(int pin, int divisor) {
	byte mode;
	if (pin == 5 || pin == 10) {
		switch (divisor) {
		case 1: mode = 0x01; break;
		case 8: mode = 0x02; break;
		case 64: mode = 0x03; break;
		case 256: mode = 0x04; break;
		case 1024: mode = 0x05; break;
		default: return;
		}
		if (pin == 5 || pin == 10)
			TCCR1B = TCCR1B & (0b11111000 | mode);
	}
 }
