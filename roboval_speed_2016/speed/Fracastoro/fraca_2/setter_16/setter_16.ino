//---------------------------------------------------------------------------//
//                           Fracastoro Hacking Lab                          //
//                                roboval.it                                 //
//                              2016 Sequencer                               //
//                                  Setter                                   //
//---------------------------------------------------------------------------//

// Copyright (c) 2011 Daniele Zambelli
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 3 as
// published by the Free Software Foundation
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

//Lib.
#include <QTRSensors.h>

//Connection PIN
#define MOTOR_STBY             7
#define MOTOR_RIGHT_PWM       10
#define MOTOR_LEFT_PWM         5
#define MOTOR_RIGHT_IN1        8
#define MOTOR_RIGHT_IN2        9
#define MOTOR_LEFT_IN2         6
#define MOTOR_LEFT_IN1         4

//Global parameters
#define NUM_SENSORS            6
#define NUM_CALI             200

#define MAX_SPEED            250    

#define K_PROPORTIONAL         0.10
#define K_DERIVATIVE           1.4
#define K_INTEGRAL             0.00008
#define K_LINEAR               1

unsigned int sensorValues[NUM_SENSORS];
QTRSensorsRC qtrrc((unsigned char[]) { A0, A1, A2, A3, A4, A5 }, NUM_SENSORS);

int proportional=0, last_proportional;
long integral;
int derivative;
int correction;
int std_speed;

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_STBY, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

	motors_go();

	//Start calibration 
	motors_test();
	for (int i = 0; i < NUM_CALI; i++) qtrrc.calibrate();
	motors_test();

	delay(2000);
}

void loop() {  
	//Sensors update.
	read_sensors();
	//Proportional, Integral, Derivative (PID) correction system.
	integral += proportional;
	derivative = proportional - last_proportional;
	correction = proportional * K_PROPORTIONAL + derivative * K_DERIVATIVE + integral * K_INTEGRAL;
	std_speed = MAX_SPEED - (K_LINEAR * constrain(abs(correction), 0, 200));

	set_motors(constrain(std_speed + correction, -MAX_SPEED, MAX_SPEED),
			   constrain(std_speed - correction, -MAX_SPEED, MAX_SPEED));
}

// SENSORS

void read_sensors() {
	//Analogic (range: -2500; +2500)
	last_proportional = proportional;
	proportional = qtrrc.readLine(sensorValues) - 2500;
}

// MOTORS

void motors_test() {
	//0.5 second motors test
	set_motors(MAX_SPEED, MAX_SPEED);
	delay(500);
	set_motors(0, 0);
}

void motors_go() {
	//Exit from Standby mode
	digitalWrite(MOTOR_STBY, HIGH);
}

void motors_stop() {
	//Standby mode
	digitalWrite(MOTOR_STBY, LOW);
}

void set_motors(int sp_l, int sp_r) {
	//Set right and left motors speeds (-255, 255)
	int dir = int(sp_l >= 0);
	digitalWrite(MOTOR_LEFT_IN1, dir);
	digitalWrite(MOTOR_LEFT_IN2, !dir);
	analogWrite(MOTOR_LEFT_PWM, abs(sp_l));
	dir = int(sp_r >= 0);
	digitalWrite(MOTOR_RIGHT_IN1, dir);
	digitalWrite(MOTOR_RIGHT_IN2, !dir);
	analogWrite(MOTOR_RIGHT_PWM, abs(sp_r));
}
