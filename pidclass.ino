#define dt 100
/*
 * Un pic de explicatii:
 *  Nu pot pune attachinterrupt pe o metoda a clasei, deci fac o alta functie care salveaza in variabila globala pozitia encoderului,
 *  pe care o iau folosind un pointer
 */
int pa=0;
int pb=0;

class MotorPID {
  private:
    float KP, KI, KD;
    bool dirRight;
    volatile int vi, vt, pt, pa, basePWM;
    int* p=nullptr;
    int IN1, IN2, ENABLE,ENC;
    float r = 0.015;//m
    int ppr = 3;
    int integral = 0, delta = 0;
  public:
    MotorPID(int IN1, int IN2, int ENABLE, int KP, int KI,int KD,int ENC, int basePWM, bool dirRight,int* p) {
      this->IN1 = IN1;
      this->IN2 = IN2;
      this->ENABLE = ENABLE;
      this->ENC=ENC;
      this->KP = KP;
      this->KI = KI;
      this->KD = KD;
      this->dirRight = dirRight;
      this->p = p;
      pt = 0, pa = 0, vi = 0, vt = 0;
    }
    void run() {
      int ev = vt - vi;
      integral += ev;
      int out = KP * ev + KI * integral + KD * (ev - delta);
      delta = ev;
      if (dirRight) {
        analogWrite(IN1, basePWM + out);
        analogWrite(IN2, LOW);
      }
      else {
        analogWrite(IN1, LOW);
        analogWrite(IN2, basePWM + out);
      }
    }
    void setTargetSpeed(int vt = 0) {
      vt = this->vt;
    }
    int getENC(){
      return ENC;
    }
    void calculateSpeed() {
      vi = (*p) / ppr / dt * 1000 * 2 * PI * r;
      *p = 0;
    }//m/s;
};
void setup(){
  MotorPID stanga(A0,A1,A2,0,0,0,A3,127,true,&pa);
  MotorPID dreapta(A4,A5,A6,0,0,0,A7,127,false,&pb);
  //int IN1, int IN2, int ENABLE, int KP, int KI,int KD,int ENC, int basePWM, bool dirRight,int& p
}
void loop(){
  
}
