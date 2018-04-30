/*

*/

#define PWM_RESOLUTION 1024

#define encoderPinA_M1 2
#define encoderPinB_M1 3
#define DIRpin_M1 4
#define PWMpin_M1 5
#define SLPpin_M1 6
#define CSpin_M1 A0

#define encoderPinA_M2 11
#define encoderPinB_M2 12
#define DIRpin_M2 8
#define PWMpin_M2 7 
#define SLPpin_M2 9
#define CSpin_M2 A1

volatile int position_M1 = 0;
volatile int position_M2 = 0;

#include "vfPWM.h"
#include "useful_functions.h"
#include "Encoders.h"
#include <Filters.h>

const float bit2Newton = 0.1463; // 0.1463N/bit of current sensor reading
const float bitoffset = 788.5;

const float pos_Pgain = 0.02; // pos:unit(0-590), output:force(Newton) 
const float pos_Dgain = 0.005;
const float F_Pgain = 1000/52;
const float F_Igain = 1000/52;

int stiff_mode = 0;
 // filters out changes faster that 2 Hz.
float filterFrequency = 1;  

// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilter( LOWPASS, filterFrequency );  

void InitMotorPos_2D(){
  setMotorPower_M1(-200);
  setMotorPower_M2(-200);
  delay(1000);
  position_M1 = 0;
  position_M2 = 0;
}

void setup() {
  
  pinMode(DIRpin_M1,OUTPUT);
  pinMode(PWMpin_M1,OUTPUT);
  pinMode(SLPpin_M1,OUTPUT);
  pinMode(CSpin_M1,INPUT);
  
  pinMode(DIRpin_M2,OUTPUT);
  pinMode(PWMpin_M2,OUTPUT);
  pinMode(SLPpin_M2,OUTPUT);
  pinMode(CSpin_M2,INPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoderPinA_M1), doEncoderA_M1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB_M1), doEncoderB_M1, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(encoderPinA_M2), doEncoderA_M2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB_M2), doEncoderB_M2, CHANGE);
  
  digitalWrite(SLPpin_M1, HIGH);
  digitalWrite(DIRpin_M1, LOW);
  
  digitalWrite(SLPpin_M2, HIGH);
  digitalWrite(DIRpin_M2, LOW);
  
  PWM_setup();
  
  InitMotorPos_2D(); 

  Serial.begin(9600);
  
}


void loop() {
  //Pos_gainScheduling(300, 300, 20);
  //PIDpos(0.02,0.5,0.1, 300, 300, 10000);
  //PDposPIforce(300, 300, 0, 0, pos_Pgain, pos_Dgain, F_Igain, F_Pgain, 10000);
  //testMotor();
  //setPWMDuty_M1(30);
  //delay(500);
  //PDpos(0.02,0.5, 300, 300, 10000);

  /*
  setMotorPower_M1(0);
  setMotorPower_M2(0);
  int current1 = lowpassFilter.input(analogRead(CSpin_M1));
  int current2 = lowpassFilter.input(analogRead(CSpin_M2));// - bitoffset;
  Serial.print(current1,DEC);
  Serial.print("\t");
  Serial.println(current2,DEC);
  */
  //int current1 = lowpassFilter.input(analogRead(CSpin_M1));
  //Serial.println(current1,DEC);
  
  //PIDpos(0.01, 0.003, 0.01, 300, 300, 10000);
  //PIDposPIDforce(300, 300, 0, 0,52*0.02, 52*0.005, 52*0.01, 0,0,0);

  PIDposPIDforce(300, 300, 0, 0, 52*0.03, 52*0.005, 52*0.01, -400, -30, -200);
}

void Pos_gainScheduling(int r1, int r2, int thr){
  float Pgain_low = 0.02;
  float Dgain_low = 0.005;
  float Pgain_high = 0.09;
  float Dgain_high = 1;
  float Igain = 0.01;
  unsigned long old_time = 0;
  unsigned long new_time = 0;
  unsigned long elapsed_time = 0;
  int err_M1, pre_err_M1, err_M2, pre_err_M2;
  int oldpos_M1 = 0;
  int oldpos_M2 = 0;
  float force_M1 = 0;
  float force_M2 = 0;
  float Pterm_M1 = 0;
  float Dterm_M1 = 0;
  float Pterm_M2 = 0;
  float Dterm_M2 = 0;
  float Iterm_M1 = 0;
  float Iterm_M2 = 0;
  int count = 0;
  
  unsigned long start_time = micros();
  new_time = micros();
  
  float Pgain1 = Pgain_high;
  float Dgain1 = Dgain_high;
  float Pgain2 = Pgain_high;
  float Dgain2 = Dgain_high;
  int setpos_M1 = r1;
  int setpos_M2 = r2;
  
  while(1){
    // PD position control
    new_time = micros();
    elapsed_time = new_time - old_time;
    old_time = new_time;
    
    err_M1 = setpos_M1 - position_M1;
    err_M2 = setpos_M2 - position_M2;
    
    if(count > 10){
    if(fabs(err_M1) > thr){
      Pgain1 = Pgain_low;
      Dgain1 = Dgain_low;
      setpos_M1 = position_M1;
    }
    else{
      Pgain1 = Pgain_high;
      Dgain1 = Dgain_high;
      setpos_M1 = r1;
    }
    
    if(fabs(err_M2) > thr){
      Pgain1 = Pgain_low;
      Dgain1 = Dgain_low;
      setpos_M2 = position_M2;
    }
    else{
      Pgain1 = Pgain_high;
      Dgain1 = Dgain_high;
      setpos_M2 = r2;
    }
    count = 0;
    }
    Pterm_M1 = Pgain1*err_M1;
    Dterm_M1 = Dgain1*(err_M1 - pre_err_M1)/elapsed_time;
    Iterm_M1 = Iterm_M1 + Igain*elapsed_time*1e-6*err_M1;
    if(err_M1*pre_err_M1 < 0 || Iterm_M1 > 1)
    {
      Iterm_M1 = 0;
    }
    force_M1 = Pterm_M1 + Dterm_M1 + Iterm_M1;
    
    Pterm_M2 = Pgain2*err_M2;
    Dterm_M2 = Dgain2*(err_M2 - pre_err_M2)/elapsed_time;
    Iterm_M2 = Iterm_M2 + Igain*elapsed_time*1e-6*err_M2;
    if(err_M2*pre_err_M2 < 0 || Iterm_M2 > 1)
    {
      Iterm_M2 = 0;
    }
    force_M2 = Pterm_M2 + Dterm_M2 + Iterm_M2;
    
    pre_err_M1 = err_M1;
    pre_err_M2 = err_M2;
    
    setMotorPower_M1(52*force_M1);
    setMotorPower_M2(52*force_M2);
    
    // PI force/current control
    //PIforceControl_2D(force_Igain, force_Pgain, force_M1, force_M2, 10);
    //Serial.println(elapsed_time);

    /*
    if (oldpos_M1!=position_M1 || oldpos_M2!=position_M2){
      oldpos_M1=position_M1;
      oldpos_M2 = position_M2;
      int current1 = analogRead(CSpin_M1);
      int current2 = analogRead(CSpin_M2);// - bitoffset;
      Serial.print(current1,DEC);
      Serial.print("\t");
      Serial.println(current2,DEC);
    
    }
    */
    
    count++;
  }
}

void testEncoder(){
  Serial.print("Motor1: ");
  Serial.println(position_M1,DEC);
  Serial.print("Motor2: ");
  Serial.println(position_M2,DEC);
  Serial.println("\t");
  delay(20);
}

void testMotor(){
  // setMotorPower_M1(0);
  setMotorPower_M2(150);
  delay(500);
  setMotorPower_M2(-150);
  delay(500);
  setMotorPower_M2(0);
  delay(5);
  setMotorPower_M1(150);
  delay(500);
  setMotorPower_M1(-150);
  delay(500);
  setMotorPower_M1(0);
  delay(5);
}

void PIDpos(float Pgain, float Dgain, float Igain, int setpos_M1, int setpos_M2, int time){
  
  unsigned long old_time = 0;
  unsigned long new_time = 0;
  unsigned long elapsed_time = 0;
  int err_M1, pre_err_M1, err_M2, pre_err_M2;
  int oldpos_M1 = 0;
  int oldpos_M2 = 0;
  float force_M1 = 0;
  float force_M2 = 0;
  float Pterm_M1 = 0;
  float Dterm_M1 = 0;
  float Pterm_M2 = 0;
  float Dterm_M2 = 0;
  float Iterm_M1 = 0;
  float Iterm_M2 = 0;
  
  unsigned long start_time = micros();
  new_time = micros();
  while((new_time - start_time)< time*1e6 ){
    // PD position control
    new_time = micros();
    elapsed_time = new_time - old_time;
    old_time = new_time;
    
    err_M1 = setpos_M1 - position_M1;
    err_M2 = setpos_M2 - position_M2;
    
    Pterm_M1 = Pgain*err_M1;
    Dterm_M1 = Dgain*(err_M1 - pre_err_M1)/elapsed_time;
    Iterm_M1 = Iterm_M1 + Igain*elapsed_time*1e-6*err_M1;
    if(err_M1*pre_err_M1 < 0)
    {
      Iterm_M1 = 0;
    }
    force_M1 = Pterm_M1 + Dterm_M1 + Iterm_M1;
    
    Pterm_M2 = Pgain*err_M2;
    Dterm_M2 = Dgain*(err_M2 - pre_err_M2)/elapsed_time;
    Iterm_M2 = Iterm_M2 + Igain*elapsed_time*1e-6*err_M2;
    if(err_M2*pre_err_M2 < 0)
    {
      Iterm_M2 = 0;
    }
    force_M2 = Pterm_M2 + Dterm_M2 + Iterm_M2;
    
    pre_err_M1 = err_M1;
    pre_err_M2 = err_M2;
    
    setMotorPower_M1(52*force_M1);
    setMotorPower_M2(52*force_M2);
    
    // PI force/current control
    //PIforceControl_2D(force_Igain, force_Pgain, force_M1, force_M2, 10);
    //Serial.println(elapsed_time);
    
    if (oldpos_M1!=position_M1 || oldpos_M2!=position_M2){
      oldpos_M1=position_M1;
      oldpos_M2 = position_M2;
      Serial.print(oldpos_M1,DEC);
      Serial.print("\t");
      Serial.println(oldpos_M2,DEC);
    }
    
  }
}

void setMotorPower_M1(int output)
{
  // set motor direction
  if (output > 0) 
  {
    if (output < 5) output = 0;
    else if (output < 15) output = 15;
    if (output > 330) output = 330;
    digitalWrite(DIRpin_M1, LOW); 
    setPWMDuty_M1(output);
    
  }
  else 
  {
    if (output > -5) output = 0;
    else if (output < -330) output = -330;
    if (output > -15) output = -15;
    digitalWrite(DIRpin_M1, HIGH);
    setPWMDuty_M1(-output);
  }
}

void setMotorPower_M2(int output)
{
  // set motor direction
  if (output > 0) 
  {
    if (output < 5) output = 0;
    else if (output < 15) output = 15;
    if (output > 330) output = 330;
    digitalWrite(DIRpin_M2, LOW); 
    setPWMDuty_M2(output);
    
  }
  else 
  {
    if (output > -5) output = 0;
    else if (output < -330) output = -330;
    if (output > -15) output = -15;
    digitalWrite(DIRpin_M2, HIGH);
    setPWMDuty_M2(-output);
  }
}


void PDposPIforce(int setpos_M1, int setpos_M2, float setforce1, float setforce2, float Pgain, float Dgain, float force_Igain, float force_Pgain){
  
  unsigned long old_time = 0;
  unsigned long new_time = 0;
  unsigned long elapsed_time = 0;
  int err_M1, pre_err_M1, err_M2, pre_err_M2;
  int oldpos_M1 = 0;
  int oldpos_M2 = 0;
  float p1_Pterm = 0;
  float p1_Dterm = 0;
  float p2_Pterm = 0;
  float p2_Dterm = 0;
  
  float err1, err2;
  float f1_Iterm = 0;
  float f1_Pterm = 0;
  float f2_Iterm = 0;
  float f2_Pterm = 0;
  int current1 = 0;
  int current2 = 0;
  
  float output1 = 0;
  float output2 = 0;
  
  unsigned long start_time = micros();
  new_time = micros();
  while(1){
    // PD position control
    new_time = micros();
    elapsed_time = new_time - old_time;
    old_time = new_time;
    
    err_M1 = setpos_M1 - position_M1;
    err_M2 = setpos_M2 - position_M2;
    
    p1_Pterm = Pgain*err_M1;
    p1_Dterm = Dgain*(err_M1 - pre_err_M1)/elapsed_time;
    
    p2_Pterm = Pgain*err_M2;
    p2_Dterm = Dgain*(err_M2 - pre_err_M2)/elapsed_time;
    
    pre_err_M1 = err_M1;
    pre_err_M2 = err_M2;
    
    // PI force 
    //current1 = lowpassFilter.input(analogRead(CSpin_M1)) - bitoffset;
    //current2 = lowpassFilter.input(analogRead(CSpin_M2)) - bitoffset;

    current1 = analogRead(CSpin_M1) - bitoffset;
    current2 = analogRead(CSpin_M2) - bitoffset;
    
    err1= setforce1 - current1*bit2Newton;
    err2= setforce2 - current2*bit2Newton;
    
    f1_Iterm = f1_Iterm + F_Igain*(new_time - old_time)*1e-6*err1;
    f1_Pterm = F_Pgain*err1;
    output1 = p1_Pterm + p1_Dterm + f1_Iterm + f1_Pterm + setforce1; 
    
    f2_Iterm = f2_Iterm + F_Igain*(new_time - old_time)*1e-6*err2;
    f2_Pterm = F_Pgain*err2;
    output2 = p2_Pterm + p2_Dterm + f2_Iterm + f2_Pterm + setforce2;  
  
    setMotorPower_M1((int)output1);
    setMotorPower_M2((int)output2);
    
    if (oldpos_M1!=position_M1 || oldpos_M2!=position_M2){
      oldpos_M1=position_M1;
      oldpos_M2 = position_M2;
      Serial.print(oldpos_M1,DEC);
      Serial.print("\t");
      Serial.println(oldpos_M2,DEC);
    
    }
  }
}

void PIforce(float setforce1, float setforce2, float force_Igain, float force_Pgain){
  
  unsigned long old_time = 0;
  unsigned long new_time = 0;
  unsigned long elapsed_time = 0;
  int err_M1, pre_err_M1, err_M2, pre_err_M2;
  int oldpos_M1 = 0;
  int oldpos_M2 = 0;
  float p1_Pterm = 0;
  float p1_Dterm = 0;
  float p2_Pterm = 0;
  float p2_Dterm = 0;
  
  float err1, err2;
  float f1_Iterm = 0;
  float f1_Pterm = 0;
  float f2_Iterm = 0;
  float f2_Pterm = 0;
  int current1 = 0;
  int current2 = 0;
  
  float output1 = 0;
  float output2 = 0;
  
  unsigned long start_time = micros();
  new_time = micros();
  while(1){
    // PD position control
    new_time = micros();
    elapsed_time = new_time - old_time;
    old_time = new_time;
    
    // PI force 
    //current1 = lowpassFilter.input(analogRead(CSpin_M1)) - bitoffset;
    //current2 = lowpassFilter.input(analogRead(CSpin_M2)) - bitoffset;

    current1 = lowpassFilter.input(analogRead(CSpin_M1)) - bitoffset;
    current2 = analogRead(CSpin_M2) - bitoffset;
    
    err1= -setforce1 + current1*bit2Newton;
    err2= -setforce2 + current2*bit2Newton;
    
    f1_Iterm = f1_Iterm + F_Igain*(new_time - old_time)*1e-6*err1;
    f1_Pterm = F_Pgain*err1;
    output1 = p1_Pterm + p1_Dterm + f1_Iterm + f1_Pterm + setforce1; 
    
    f2_Iterm = f2_Iterm + F_Igain*(new_time - old_time)*1e-6*err2;
    f2_Pterm = F_Pgain*err2;
    output2 = p2_Pterm + p2_Dterm + f2_Iterm + f2_Pterm + setforce2;  
  
    setMotorPower_M1((int)output1);
    setMotorPower_M2((int)output2);
    /*
    if (oldpos_M1!=position_M1 || oldpos_M2!=position_M2){
      oldpos_M1=position_M1;
      oldpos_M2 = position_M2;
     */
      Serial.print(current1,DEC);
      Serial.print("\t");
      Serial.println(current2,DEC);
    
    //}
  }
}

void PIDposPIDforce(int setpos_M1, int setpos_M2, float setforce1, float setforce2, float Pgain, float Dgain, float Igain, float force_Pgain, float force_Igain, float force_Dgain){
  
  unsigned long old_time = 0;
  unsigned long new_time = 0;
  unsigned long elapsed_time = 0;
  int ep1, ep1_pre, ep2, ep2_pre;
  int oldpos_M1 = 0;
  int oldpos_M2 = 0;
  float p1_Pterm = 0;
  float p1_Dterm = 0;
  float p2_Pterm = 0;
  float p2_Dterm = 0;
  float p1_Iterm = 0;
  float p2_Iterm = 0;
  
  float ef1, ef1_pre, ef2, ef2_pre;
  float f1_Iterm = 0;
  float f1_Pterm = 0;
  float f2_Iterm = 0;
  float f2_Pterm = 0;
  float f1_Dterm = 0;
  float f2_Dterm = 0;
  int current1 = 0;
  int current2 = 0;
  
  float output1 = 0;
  float output2 = 0;
  
  unsigned long start_time = micros();
  old_time = micros();
  while(1){
    new_time = micros();
    elapsed_time = new_time - old_time;
    old_time = new_time;
    
    ep1 = setpos_M1 - position_M1;
    ep2 = setpos_M2 - position_M2;
    
    p1_Pterm = Pgain*ep1;
    p1_Dterm = Dgain*(ep1 - ep1_pre)/elapsed_time;
    p1_Iterm = p1_Iterm + Igain*elapsed_time*1e-6*ep1;
    if(ep1*ep1_pre < 0)
    {
      p1_Iterm = 0;
    }
    
    p2_Pterm = Pgain*ep2;
    p2_Dterm = Dgain*(ep2 - ep2_pre)/elapsed_time;
    p2_Iterm = p2_Iterm + Igain*elapsed_time*1e-6*ep2;
    if(ep2*ep2_pre < 0)
    {
      p2_Iterm = 0;
    }
      
    ep1_pre = ep1;
    ep2_pre = ep2;
    
    // PI force 
    current1 = lowpassFilter.input(analogRead(CSpin_M1)) - bitoffset + 12;
    current2 = lowpassFilter.input(analogRead(CSpin_M2)) - bitoffset + 12;

    //current1 = analogRead(CSpin_M1) - bitoffset;
    //current2 = analogRead(CSpin_M2) - bitoffset;
    
    ef1= -setforce1 + current1*bit2Newton;
    ef2= -setforce2 + current2*bit2Newton;
    ef2 = 0;
    
    f1_Iterm = f1_Iterm + force_Igain*(new_time - old_time)*1e-6*ef1;
    f1_Pterm = force_Pgain*ef1;
    f1_Dterm = force_Dgain*(ef1 - ef1_pre)/elapsed_time;
    if(ef1*ef1_pre < 0)
    {
      f1_Iterm = 0;
    }
    output1 = p1_Pterm + p1_Dterm + p1_Iterm + f1_Iterm + f1_Pterm + setforce1; 
    
    f2_Iterm = f2_Iterm + force_Igain*(new_time - old_time)*1e-6*ef2;
    f2_Pterm = force_Pgain*ef2;
    f2_Dterm = force_Dgain*(ef2 - ef2_pre)/elapsed_time;
    if(ef2*ef2_pre < 0)
    {
      f2_Iterm = 0;
    }
    output2 = p2_Pterm + p2_Dterm + p2_Iterm + f2_Iterm + f2_Pterm + setforce2; 

    ef1_pre = ef1;
    ef2_pre = ef2;
    
    setMotorPower_M1((int)output1);
    setMotorPower_M2((int)output2);

  
    //if (oldpos_M1!=position_M1 || oldpos_M2!=position_M2){
      oldpos_M1=position_M1;
      oldpos_M2 = position_M2;
      
      Serial.print(current1,DEC);
      Serial.print("\t");
      /*
      Serial.print(current2,DEC);
      Serial.println("\t");
      */
      Serial.print(oldpos_M1,DEC);
      Serial.print("\t");
      Serial.println(oldpos_M2,DEC);
    //}
    
    //Serial.print(analogRead(CSpin_M1),DEC);
    //Serial.print("\t");
    //Serial.println(lowpassFilter.input(analogRead(CSpin_M1)),DEC);
   
  }
}






