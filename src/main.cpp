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
String Sys_mode = "Close Loop";                     //to define the close loop or openloop system
long setpoint = 0;                        //Setpoint for System
volatile unsigned long counterValue = 0, FeedBack = 0;   //Counter for ENC signal
volatile unsigned int Op_time =0, sampleTime =0;  //time in millisecond
volatile unsigned int FeedBack_RPM;
bool FeedbackEnable = true;               //to force system between open and close loop operation
volatile float Drive_Signal;
volatile float Ierror=0.0, Error=0.0,PrevError=0.0,Derror=0.0,Ki=0,Kp=1,Kd=0;     //PID parameter
bool startFlag= false;
//void NewSample(void);

//This ISR of external interrupt attached to pin 2 comes from SYNC 
void NewSample(){
  sampleTime++;
  if(sampleTime >= 5){
    //Serial.println(Feedback);
    noInterrupts();
    SYNC_enable_flag = true;
    FeedBack = counterValue;
    FeedBack_RPM = FeedBack * 12; //Calculate Feedback as RPM (Round per Minute)
    Error = setpoint - FeedBack_RPM*FeedbackEnable;
    Ierror += Error*0.1;
    Derror = (Error - PrevError)/0.1;
    PrevError = Error;
    Drive_Signal = Error*Kp + Ierror*Ki + Derror*Kd;
    counterValue=0;
    sampleTime = 0;
    interrupts();
  } 
}

//This ISR of external interrupt attached to pin 3 comes from ENC 
void CountPulses(){
  counterValue++;
}

void setup() {
  Serial.begin(115200);   //configure bauderate 115200
  pinMode(DIR,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(SYNC),NewSample,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC),CountPulses,FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (SerialRecivedFlag) {
    SerialRecivedFlag = false;
    //Serial.println(SerialReciveData);
    if(SerialReciveData.startsWith("set")){         //set new setpoint 
      SerialReciveData.remove(0,3);
      setpoint = SerialReciveData.toInt();
    } else if(SerialReciveData.startsWith("kp")){ //set new Kp
      SerialReciveData.remove(0,2);
      Kp=SerialReciveData.toFloat();
    }else if(SerialReciveData.startsWith("ki")){  //set new Ki
      SerialReciveData.remove(0,2);
      Ki=SerialReciveData.toFloat();
    }else if(SerialReciveData.startsWith("kd")){ //set new Kd
      SerialReciveData.remove(0,2);
      Kd=SerialReciveData.toFloat();
    }else if(SerialReciveData.startsWith("c")){ //Display current configuration
      Serial.print("Setpoint= ");
      Serial.print(setpoint);
      Serial.print("\tKp= ");
      Serial.print(Kp,4);
      Serial.print("\tKi= ");
      Serial.print(Ki,4);
      Serial.print("\tKd= ");
      Serial.print(Kd,4);
      Serial.print("\tMode= ");
      Serial.println(Sys_mode);
    }else if(SerialReciveData.startsWith("m")){ // toggle operation mode Close/Open loop
      if(FeedbackEnable){
        FeedbackEnable=false;
        Sys_mode="Open Loop";
      }else{
        FeedbackEnable=true;
        Sys_mode="Close Loop";
      }
    }else if(SerialReciveData.startsWith("on")){  // Run the system
      startFlag = true;
    }else if(SerialReciveData.startsWith("off")){ // Stop the system
      startFlag = false;
    }else{
      Serial.println("Wrong Input!!");  //Wrong input 
    }
    //Serial.println(SerialReciveData);
    //if (setpoint > 255) setpoint = 255;    // to a void over load
    SerialReciveData = "";    //Reset to recive new date
    //Serial.println(setpoint);
  }
  if (setpoint >= 0) {
    digitalWrite(DIR, LOW);
  } 
  else {
    digitalWrite(DIR, HIGH);
  }
  //if(Error < 0) Error = 0;  // Error < 0 that mean feedback > setpoint .. Action = motor stop
  if (Drive_Signal > 255){
    Drive_Signal = 255;    // to a void over load
  }
  if(Drive_Signal <= 0){
    Drive_Signal = 0;
  }
  analogWrite(PULSE, 255 - Drive_Signal); //PULSE pin -> PIN 5
  while(SYNC_enable_flag){
    SYNC_enable_flag = false;
    if(startFlag){
      //Serial.println(FeedBack);
        //Serial.print("Setpoint= ");
        Serial.print(Op_time++/20.0);
        Serial.print(",");
        Serial.print(setpoint);
        //Serial.print("\tFeedBack= ");
        //Serial.print(FeedBack);
        Serial.print(",");
        Serial.print(FeedBack_RPM);
        if(FeedbackEnable){
          Serial.print(",");
          Serial.print(Error);
          Serial.print(",");
          Serial.print(Ierror);
          Serial.print(",");
          Serial.print(Derror);
          Serial.print(",");
          Serial.print(Drive_Signal);
        }
        Serial.println("");
        
    }else{
      Drive_Signal = 0;
      Error=0;
      Ierror=0;
      Derror=0;
      counterValue = 0;
      Op_time = 0;
    }
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