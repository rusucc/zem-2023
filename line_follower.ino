/*
Rusu Cosmin, Dogaru Paul, Zem 2023 LineFollower

Pini Senzori : D4,D5,D6,D7,D8,D9, Emiter Pin
Pini Led : D2, D15
Pini Motor(A si B) : D10,D11
Pini Senzori laterali : A6,A7 I2C, A0,A1 XSHUT
Pini Liberi : A2->A5,  
*/
#include <QTRSensors.h>
#include <Adafruit_VL53L0X.h>
//constante PID
#define SPEED 100
#define KP 0
#define KI 0
#define KD 0
//constante senzori laterali
#define L1_ADDRESS 0x30
#define L2_ADDRESS 0x31
#define SHT_LOX1 A0
#define SHT_LOX2 A1
//constante senzori pololu
#define SensorCount 6
#define SensorStart 4
#define SensorEmitter 3
//leduri
#define LedA null
#define LedB null
//motoare
#define In1A null
#define In2A null
#define In1B null
#define In2B null
#define motorA 10
#define motorB 11
//debug
const bool DEBUG_MODE = false;
//senzori pololu
QTRSensors qtr;
uint16_t sensorValues[SensorCount];
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
//senzori laterali
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

void setup();
void setID();
void read_dual_sensors();
int PID();
void loop()
{
  uint16_t currentPosition = qtr.readLineBlack(sensorValues);
  int valPID = PID(2500,currentPosition); //valoare PID output
  valPID=map(valPID, -2500, 2500, 0,255);
  analogWrite(motorA,SPEED-valPID);
  analogWrite(motorB,SPEED+valPID);
  delay(100);
  if(DEBUG_MODE){
    for (uint8_t i = 0; i < SensorCount; i++){
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.print("| |Pozitie: ");
    Serial.print(currentPosition),Serial.print("| Valoare PID: "),Serial.println(valPID);
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
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  if(DEBUG_MODE) Serial.println(F("Shutdown pins inited..."));
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  if(DEBUG_MODE) Serial.println(F("Both in reset mode...(pins are low)"));
  if(DEBUG_MODE) Serial.println(F("Starting..."));
  setID();

}
void setID() {
  // resetarea pinurilor pentru i2c
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(L1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(L2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}
void read_dual_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  if(DEBUG_MODE){
    Serial.print(F("1: "));
    if(measure1.RangeStatus != 4) {     // if not out of range
      Serial.print(measure1.RangeMilliMeter);
    } else {
      Serial.print(F("Out of range"));
    }
  
    Serial.print(F(" "));
    // print sensor two reading
    Serial.print(F("2: "));
    if(measure2.RangeStatus != 4) {
      Serial.print(measure2.RangeMilliMeter);
    } else {
      Serial.print(F("Out of range"));
    }
    Serial.println();
  }
}
