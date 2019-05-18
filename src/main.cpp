/* Pin Map
    |DC Motor Unit  | Arduino |
    --------------------------
    |     ENC       |   PIN 3 | INTERRUPT FALLING
    |     STCNT     |   N.C   |
    |     DIR       |   PIN 4 |
    |     SEL       |   N.C   |
    |     PULSE     |   PIN 5 |
    |     SYNC      |   PIN 2 | INTERRUPT RISE
    |     END CNT   |   N.C   |
    |     A3        |   N.C   |
    |     A2        |   N.C   |
    |     A1        |   N.C   |
    |     A0        |   N.C   |
    |     0V        |   GND   |
    |     15V       |   N.C   | This pin must be connected up to +12V P.S.

*/

#include <Arduino.h>

#define ENC 3
#define DIR 4
#define PULSE 5
#define SYNC 2

volatile bool SYNC_enable_flag = false;   //SYNC rising Interrupt Flag
volatile bool SerialRecivedFlag = false;  //SerialEvent call Flag
volatile bool ENC_enable_flag = false;    //encrimental encoder Flag
String SerialReciveData = "";             //Recived Data via SerialEvent
long setpoint = 0;                        //Setpoint for System
volatile unsigned long counterValue = 0, FeedBack = 0;   //Counter for ENC signal
volatile unsigned int Op_time =0, sampleTime =0;
unsigned int FeedBack_RPM;
int Drive_Signal;
double Ierror=0.0, Error=0.0,PrevError=0.0,Derror=0.0,Ki=1.5,Kp=1.0,Kd=0.05;               //PID parameter
//void NewSample(void);

//This ISR of external interrupt attached to pin 2 comes from SYNC 
void NewSample(){
  sampleTime++;
  if(sampleTime >= 10){
    //Serial.println(Feedback);
    SYNC_enable_flag = true;
    FeedBack = counterValue;
    FeedBack_RPM =FeedBack*6; //Calculate Feedback as RPM (Round per Minute)
    counterValue=0;
    sampleTime = 0;
  } 
}

//This ISR of external interrupt attached to pin 3 comes from ENC 
void CountPulses(){
  counterValue++;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(DIR,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(SYNC),NewSample,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC),CountPulses,FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(SYNC_enable_flag){
    SYNC_enable_flag = false;
    Error = setpoint - FeedBack_RPM;
    Ierror += Error*0.1;
    Derror = (Error - PrevError)/0.1;
    Drive_Signal = Error*Kp + Ierror*Ki + Derror*Kd;
    // if(setpoint == 0){
    //   Error=0;
    //   Ierror=0;
    //   Derror=0;
    //   Drive_Signal =0;
    // }
  }
  if (setpoint >= 0) {
    digitalWrite(DIR, LOW);
  } 
  else {
    digitalWrite(DIR, HIGH);
  }

  while (SerialRecivedFlag) {
    SerialRecivedFlag = false;
    //Serial.println(SerialReciveData);
    setpoint = SerialReciveData.toInt();   // convert recived data to integer number
    //if (setpoint > 255) setpoint = 255;    // to a void over load
    SerialReciveData = "";
    //Serial.println(setpoint);
  }

  //if(Error < 0) Error = 0;  // Error < 0 that mean feedback > setpoint .. Action = motor stop
  if (Drive_Signal > 255){
    Drive_Signal = 255;    // to a void over load
  }
  if(Drive_Signal <= 0){
    Drive_Signal = 0;
  }
  
  analogWrite(PULSE, 255 - Drive_Signal); //PULSE pin -> PIN 5
  if(FeedBack != 0){
      //Serial.println(FeedBack);
        Serial.print("Setpoint= ");
        Serial.print(setpoint);
        //Serial.print("\tFeedBack= ");
        //Serial.print(FeedBack);
        Serial.print("\tFeedBack_RPM= ");
        Serial.print(FeedBack_RPM);
        Serial.print("\tError= ");
        Serial.print(Error);
        Serial.print("\tIerror= ");
        Serial.print(Ierror);
        Serial.print("\tDerror= ");
        Serial.print(Derror);
        Serial.print("\tDrive_Signal= ");
        Serial.println(Drive_Signal);
    }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    SerialReciveData += inChar;
    if (inChar == '\n') {
    SerialRecivedFlag = true;
    }
  }
}