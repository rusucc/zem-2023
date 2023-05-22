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
//debug
const bool DEBUG_MODE = false ;

#define SPEED 178 //190
#define KP 0.01 //0.01 
#define KI 0.00 //0.003
#define KD 0.01 //0.03
#define NOISE 40
#define BOOST 30
#define DELTASPEED 8
//constante senzori laterali
#define L1_ADDRESS 0x30
#define L2_ADDRESS 0x31
#define SHT_LOX1 A0
#define SHT_LOX2 A1
//constante senzori pololu
#define SensorCount 6
#define SensorStart 4
#define SensorEmitter 3 //initial D3!!!!
#define TCALIBRARE 100
#define THRESHOLD 900
//leduri
#define LedA 2
#define LedB 12
//motoare
#define motorA 10
#define motorB 11

//senzori pololu
QTRSensors qtr;
uint16_t sensorValues[SensorCount];
bool lineValues[SensorCount];
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
//senzori laterali
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

void setup();
void setID();
void read_dual_sensors();
int lineCase();
int PID(int targetPosition, int currentPosition){
  //Serial.println("POZ:"+String(currentPosition));
  static int INTEGRAL = 0;
  static int delta_vechi = 0;
  double delta = currentPosition - targetPosition;
  if(abs(delta)<NOISE) delta = 0;
  INTEGRAL += delta;
  double P = KP * delta;
  double I = KI * INTEGRAL;
  double D = KD * (delta-delta_vechi);
  delta_vechi = delta;
  int SUM = P+I+D;
  Serial.println("P: "+String(P)+" | I:"+String(I) + " | D:"+String(D)+ " | " + String(SUM));
  Serial.print("SUM: "),Serial.println(SUM);
  return SUM;
  
}
void loop()
{
  uint16_t currentPosition = qtr.readLineBlack(sensorValues);
  for(int i = 0; i < SensorCount; i++) lineValues[i] = (sensorValues[i]>THRESHOLD)? true:false;
  int valPID;
  int lCase = lineCase();
  valPID = PID(2500,currentPosition); //valoare PID output
  //Serial.println(valPID);
  int adder = 0;
  if(abs(valPID)<DELTASPEED) adder = BOOST;
  else adder = 0;
  int valA = SPEED + valPID + adder;
  int valB = SPEED - valPID + adder;
  if(valA<128) valA=128;
  if(valB<128) valB=128;
  if(valA>255) valA = 255;
  if(valB>255) valB = 255;
  //Serial.println(String(valA)+" "+String(valB));
  analogWrite(motorA,valA);
  analogWrite(motorB,valB);
  /*
  bool kill = true;
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    //Serial.print(sensorValues[i]);
    //Serial.print('\t');
    kill = kill and (sensorValues[i]>750);
  }
  //Serial.println();
  while(kill){
    analogWrite(motorA, 127);
    analogWrite(motorB, 127);
    Serial.println("STOP");
  }
  //kill */
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

void setup()
{
  // configure the sensors
  analogWrite(motorA, 127);
  analogWrite(motorB, 127);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){SensorStart, SensorStart + 1, SensorStart + 2, SensorStart + 3, SensorStart + 4, SensorStart+5}, SensorCount);
  qtr.setEmitterPin(SensorEmitter);
  delay(500);
  pinMode(LedA, OUTPUT);
  digitalWrite(LedA, HIGH);
  //calibrare, ledul de pe arduino e pornit
  for (uint16_t i = 0; i < TCALIBRARE; i++){
    qtr.calibrate();
  }
  
  digitalWrite(LedA, LOW);
  delay(2000);
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
  /*
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  if(DEBUG_MODE) Serial.println(F("Shutdown pins inited..."));
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  if(DEBUG_MODE) Serial.println(F("Both in reset mode...(pins are low)"));
  if(DEBUG_MODE) Serial.println(F("Starting..."));
  setID();
  */

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
int lineCase(){
  /*
   * Comentarii ca sa nu ma dau cu capul de pereti
   * 1. Linie dreapta
   * 2. Cruce
   * 3. Senzor lateral stanga 
   * 4. Senzor lateral dreapta
   * 5. Senzor perete stanga
   * 6. Senzor perete dreapta
   * 
   * 3 sau 4: iesire din giratoriu/linie punctata -> mers spre linie
   * 5 sau 6:
   */
  bool pol = false;
  for(int i=0; i<SensorCount; i++) pol = (pol || lineValues[i]);
  bool wallL = false, wallR = false;
  bool lineR = false, lineL = false;
  if(pol){ // giratoriu, cruce, linie normala
    if(lineR or lineL){
      if(lineR and lineL) return 2; // cruce
      else if(lineL) return 3; //posibil line punctata
      else return 4; //giratoriu sau lineR true;
    }
    else return 1; //linie normala
  }
  else{
    if(wallL) return 5;
    if(wallR) return 6;
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
