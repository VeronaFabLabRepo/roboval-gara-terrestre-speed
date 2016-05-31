
/*
 * @scuola:	ITT. PANELLA - VALLAURI | REGGIO CALABRIA (RC)
 * @autori:	Pelleggrino,Rotaru,Orzumati,Condemi,Geniale
 * Squadra: Non ci siamo persi
 * Robot:   Roccoval
 * @data:   Maggio 2016
 * Versione: - 
 * @evento: Roboval 2016
 * 
 */


// INCLUDO LA LIBRERIA QTRSensors
#include <QTRSensors.h>

// PIN-OUT MOTORI
// A: MOTORE SINISTRO
// B: MOTORE DESTRO
#define MOTOR_STBY        7
#define MOTOR_A_PWM       5
#define MOTOR_A_IN2       6
#define MOTOR_A_IN1       4
#define MOTOR_B_PWM       10
#define MOTOR_B_IN1       8
#define MOTOR_B_IN2       9

// IDENTIFICAZIONE MOTORI
#define MOTOR_LEFT        0
#define MOTOR_RIGHT       1

// IDENTIFICAZIONE SENSORI
#define QTRLFar           0
#define QTRLCenter        1
#define QTRLNear          2
#define QTRRNear          3
#define QTRRCenter        4
#define QTRRFar           5

// IMPOSTAZIONI QTRSensors
#define QTR_NUM_SENSORS   6
#define QTR_TIMEOUT       2500
#define QTR_EMITTER_PIN   QTR_NO_EMITTER_PIN
#define QTR_SENSOR_LED    13 // LED ARDUINO

// IMPOSTAZIONI GLOBALI
#define VELOCITA          90 // 1 - 99
#define GIRO_VELOCITA     70 // 1 - 99
#define SOGLIA            255 // 0 - 2500
int CORR_VELOCITA       = 95; // 0 - 100

// DEFINE FUNCTIONS
#define MAGG_SOGLIA(x)( (x) > SOGLIA )

// VALORI SENSORI
int QTRLFarRead     = 0;
int QTRLCenterRead  = 0;
int QTRLNearRead    = 0;
int QTRRNearRead    = 0;
int QTRRCenterRead  = 0;
int QTRRFarRead     = 0;

// CHIAMO LA LIBRERIA QTRSensorsRC
QTRSensorsRC QTRRC((unsigned char[]){A0, A1, A2, A3, A4, A5}, QTR_NUM_SENSORS, QTR_TIMEOUT, QTR_EMITTER_PIN);
// VALORE DI TUTTI I SENSORI
unsigned int QTRSensorsValue[QTR_NUM_SENSORS];

// SETUP
void setup(){

    // SERIAL BEGIN (PER DEBUG)
    Serial.begin(9600);

    // MOTORI - OUTPUT
    setPinMode();

    // CALIBRAZIONE SENSORI
    QTRSensorsCalibration();

    // ABILITAZIONE LED
    QTRSensorsCalibLED(true);

    // ATTIVO I MOTORI
    MOTORStandby(false);

}

// LOOP
void loop(){

    QTRSensorsRead();

    if(MAGG_SOGLIA(QTRLFarRead)){

        TURNLeft();

    }else if(MAGG_SOGLIA(QTRRFarRead)){

        TURNRight();

    }else if(MAGG_SOGLIA(QTRLNearRead) || MAGG_SOGLIA(QTRRNearRead)){

        STRAIGHT();

    }else if(!MAGG_SOGLIA(QTRLFarRead) && !MAGG_SOGLIA(QTRRFarRead) &&
            !MAGG_SOGLIA(QTRLCenterRead) && !MAGG_SOGLIA(QTRRCenterRead) &&
            !MAGG_SOGLIA(QTRLNearRead) && !MAGG_SOGLIA(QTRRNearRead)){

        TURNAround();

    }

}

// GIRO A SINISTRA
void TURNLeft(){

    MOTORRun(MOTOR_LEFT, -GIRO_VELOCITA);
    MOTORRun(MOTOR_RIGHT, GIRO_VELOCITA);

    /*while(MAGG_SOGLIA(QTRRNearRead)){

        QTRSensorsRead();

    }*/

    TURNDelay();

}

// GIRO A DESTRA
void TURNRight(){

    /*STRAIGHT();
    delay(50);
    QTRSensorsRead();*/

    if(!MAGG_SOGLIA(QTRLNearRead) || !MAGG_SOGLIA(QTRRNearRead)){

        MOTORRun(MOTOR_LEFT, GIRO_VELOCITA);
        MOTORRun(MOTOR_RIGHT, -GIRO_VELOCITA);

       /* while(MAGG_SOGLIA(QTRLNearRead)){

            QTRSensorsRead();

        }*/

        TURNDelay();

    }

}

// GIRO SU ME STESSO
void TURNAround(){

    MOTORRun(MOTOR_LEFT, -GIRO_VELOCITA);
    MOTORRun(MOTOR_RIGHT, GIRO_VELOCITA);

    TURNDelay();

}

// ATTESA PER IL GIRO
void TURNDelay(){

    while(!MAGG_SOGLIA(QTRRNearRead) || !MAGG_SOGLIA(QTRLNearRead)){

        QTRSensorsRead();

    }

}

// VADO AVANTI
void STRAIGHT(){

    CORR_VELOCITA=CORR_VELOCITA/2;
    
    if(!MAGG_SOGLIA(QTRLNearRead)){

        MOTORRun(MOTOR_LEFT, VELOCITA);
        MOTORRun(MOTOR_RIGHT, CORR_VELOCITA);
        return;

    }else if(!MAGG_SOGLIA(QTRRNearRead)){

        MOTORRun(MOTOR_LEFT, CORR_VELOCITA);
        MOTORRun(MOTOR_RIGHT, VELOCITA);
        return;

    }else{

        MOTORRun(MOTOR_LEFT, VELOCITA);
        MOTORRun(MOTOR_RIGHT, VELOCITA);

    }

}

// IMPOSTO LO STANDBY AI MOTORI
void MOTORStandby(boolean state){

    digitalWrite(MOTOR_STBY, state ? LOW : HIGH);

}

// IMPOSTO IL LED DEI SENSORI
void QTRSensorsCalibLED(boolean state){

    digitalWrite(QTR_SENSOR_LED, state ? HIGH : LOW);

}

// IMPOSTO I MOTORI
void MOTORRun(boolean motor, char speed){

    MOTORDirection(motor, speed > 0 ? 1 : 0);
    MOTORSpeed(motor, speed);

}

// IMPOSTO LA VELOCITA' DEL MOTORE
void MOTORSpeed(boolean motor, char speed){

    byte PWMValue = map(abs(speed), 0, 100, 0, 255);

    if(motor == MOTOR_LEFT){

        analogWrite(MOTOR_A_PWM, PWMValue);

    }else if(motor == MOTOR_RIGHT){

        analogWrite(MOTOR_B_PWM, PWMValue);

    }

}

// IMPOSTO LA DIREZIONE DEL MOTORE
void MOTORDirection(boolean motor, boolean direction){

    if(motor == MOTOR_LEFT){

        if(direction == 1){

            digitalWrite(MOTOR_A_IN1, HIGH);
            digitalWrite(MOTOR_A_IN2, LOW);

        }else{

            digitalWrite(MOTOR_A_IN1, LOW);
            digitalWrite(MOTOR_A_IN2, HIGH);

        }

    }else if(motor == MOTOR_RIGHT){

        if(direction == 1){

            digitalWrite(MOTOR_B_IN1, HIGH);
            digitalWrite(MOTOR_B_IN2, LOW);

        }else{

            digitalWrite(MOTOR_B_IN1, LOW);
            digitalWrite(MOTOR_B_IN2, HIGH);
      
        }

    }

}

// LEGGO I VALORI DEI SENSORI
void QTRSensorsRead(){

    unsigned int position = QTRRC.readLine(QTRSensorsValue);

    QTRLFarRead      = QTRSensorsValue[QTRLFar];
    QTRLCenterRead   = QTRSensorsValue[QTRLCenter];
    QTRLNearRead     = QTRSensorsValue[QTRLNear];
    QTRRNearRead     = QTRSensorsValue[QTRRNear];
    QTRRCenterRead   = QTRSensorsValue[QTRRCenter];
    QTRRFarRead      = QTRSensorsValue[QTRRFar];

}

// CALIBRAZIONE SENSORI
// NOTA: EMITTER ON
void QTRSensorsCalibration(){

    QTRRC.calibrate();

    for(int i = 0; i < QTR_NUM_SENSORS; i++){

        if(QTRRC.calibratedMinimumOn[i] > SOGLIA || QTRRC.calibratedMaximumOn[i] < SOGLIA){

            QTRSensorsCalibration();
            break;

        }

    }

}

// IMPOSTO IL PIN-MODE
void setPinMode(){

    pinMode(MOTOR_STBY, OUTPUT);
    pinMode(MOTOR_A_PWM, OUTPUT);
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_B_PWM, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(QTR_SENSOR_LED, OUTPUT);

}
