/*
Rusu Cosmin, Dogaru Paul, Zem 2023 LineFollower

Pini Senzori : D4,D5,D6,D7,D8,D9, Emiter Pin
Pini Led : D2, D15
Pini Motor(A si B) : D10,D11
Pini Senzori laterali : A6,A7 I2C
Pini Liberi : A0->A5,  
*/
#include <QTRSensors.h>
#define SPEED 100
#define KP 0
#define KI 0
#define KD 0

#define SensorCount 6
#define LedA null
#define LedB null

#define MotorA null
#define MotorB null

#define SensorStart 4
#define SensorEmitter 3

const bool DEBUG_MODE = false;
QTRSensors qtr;

uint16_t sensorValues[SensorCount];

void setup()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){SensorStart, SensorStart + 1, SensorStart + 2, SensorStart + 3, SensorStart + 4, SensorStart+5}, SensorCount);
  qtr.setEmitterPin(SensorEmitter);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //calibrare, ledul de pe arduino e pornit
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  //oprire calibrare, led oprit
  Serial.begin(9600);
  if(DEBUG_MODE){
    for (uint8_t i = 0; i < SensorCount; i++){
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
      }
    Serial.println();
    // print the calibration maximum values measured when emitters were on
    for (uint8_t i = 0; i < SensorCount; i++){
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(1000);
  }
}
int PID(int targetPosition, int currentPosition){
  static int INTEGRAL = 0;
  static int delta_vechi = 0;
  int delta = currentPosition - targetPosition;
  INTEGRAL += delta;
  int P = KP * delta;
  int I = KI * INTEGRAL;

  
  int D = KD * (delta-delta_vechi);
  delta_vechi = delta;
  return P+I+D;
}
void loop()
{
  uint16_t currentPosition = qtr.readLineBlack(sensorValues);
  int vPID = PID(2500,currentPosition); //valoare PID output
  if(DEBUG_MODE){
    for (uint8_t i = 0; i < SensorCount; i++){
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.print("| |Pozitie: ")
    Serial.print(currentPosition),Serial.print("| Valoare PID: "),Serial.println(vPid);
  }
  vPid=map(vPID, -2500, 2500, 0,255);
  analogWrite(motorA,speed-vPID);
  analogWrite(motorB,speed+vPID);
  delay(100);
}
