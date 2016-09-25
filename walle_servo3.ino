/*
 *
 *Write Commands:
 * Serial Communication Protocol
 * Motor Forward Write- f###
 * Motor Reverse Write - r###
 * Servo Write- s###
*
 *Read Commands:
*Servo Read - s###o
*Encoder Read - e###r
 */

#define encoder_slave 8    //I2C address of encoder slave
#include <Servo.h>
#include <Wire.h>
//#include "TimerOne.h"

//Wire 20(SDA) 21(SCL)
//String encoder_read; // variable to store the encoder values recd

//Motor Control Pins
int throttle = 6;
int motor_sw1 = 4;
int motor_sw2 = 3;
int enable = 5;
char space[2];//to store the character value received from master
//Servo Control on pin 9
Servo myservo;
int write_angle;
int read_angle;
int t=0;
int timer=0;
boolean flag=false;
char selection;   //selection character for serial protocol
int serial_buffer;  
int encoder_ticks[2];
int count,comp=0;
void setup() 
{
Serial.begin(9600);//Initialize serial communication
//Serial.println("Initializing Arduino");
pinMode(throttle,OUTPUT);
pinMode(enable,OUTPUT);
pinMode(motor_sw1,OUTPUT);
pinMode(motor_sw2,OUTPUT);
Serial.setTimeout(100);
myservo.attach(9);//Initialize Servo Control
Wire.begin();//Initialize wire communication
 noInterrupts();           // disable all interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;

  OCR2A = (195);       // compare match register 16MHz/1024/80Hz
  TCCR2B |= (1 << WGM22);   // CTC mode
  TCCR2B |= (1 << CS20);//
  TCCR2B |= (1 << CS21);//
  TCCR2B |= (1 << CS22);// 1024 prescaler 
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
  interrupts(); 
t = millis();
}
ISR(TIMER2_COMPA_vect) 
{
  count++;
  if(count==5)
  {
    count=0;
  }
}
void loop() {
if(Serial.available())
{
 
  char selection =Serial.read();
  //Serial.println(selection);
  serial_buffer=Serial.parseInt();
   if (selection=='f' || selection =='F')
    motorForwardControl(serial_buffer);
  else if (selection=='r' || selection =='R')
    motorReverseControl(serial_buffer);
  else if (selection=='s' || selection =='S')
    servoControl(serial_buffer);
   t=millis();
}
else
{
  //t=millis();
  if(millis()-t>=500)
  {
     motorForwardControl(0);
      motorReverseControl(0);
  }
}
if(count!=comp)
  {
   Wire.beginTransmission(9);
   int ask1 = Wire.requestFrom(9,(uint8_t)15);
  if(ask1 == 15)
  {
    for(int i=0;i<5;i++)
    {
    char c=char(Wire.read());
    int received = Wire.read()<< 8 ;
    received|= Wire.read();
    Serial.print(received);
    Serial.print(c);
    }
    Serial.println(); 
  }
  Wire.endTransmission();
  comp=count;
  }
Wire.beginTransmission(encoder_slave);
int ask2=Wire.requestFrom(encoder_slave,(uint8_t)6);
if(ask2==6)
{
  for(int i=0;i<2;i++)
  { 
    space[i]=(char)(Wire.read()); 
    //c[i]=(char)(Wire.read());
    encoder_ticks[i] = Wire.read()<<8;
    encoder_ticks[i] |= Wire.read();
    if(i==0)
    {
      Serial.print('E');
      Serial.print(space[i]);
      Serial.print(encoder_ticks[i]);
    }
    else
    {
      Serial.print(space[i]);
      Serial.print(encoder_ticks[i]);
      Serial.print(space[i]);
      Serial.print('R');
    }
   }
  }
  Serial.print('\n');
Wire.endTransmission();
read_angle=myservo.read();
if(read_angle<=(write_angle-10))
{
  if(flag==true)
    {
      Serial.print("E1");
      Serial.print('\n');
      flag=false;
    }
  if((millis()-timer)>=1000)
  {
    flag=true;
  }
}
else
 {
  Serial.print("S ");
  Serial.print(read_angle);
  Serial.print(" O");
  Serial.print('\n');
 }
}

void motorForwardControl(int motor_speed)
{
  digitalWrite (enable,HIGH);
 digitalWrite (motor_sw1,LOW);
digitalWrite (motor_sw2,HIGH);
analogWrite(throttle,motor_speed);
digitalWrite (enable,LOW);
}


void motorReverseControl(int motor_speed)
{
digitalWrite (enable,HIGH);
digitalWrite (motor_sw1,HIGH);
digitalWrite (motor_sw2,LOW);
analogWrite(throttle,motor_speed);
digitalWrite(enable,LOW);
}

void servoControl(int write_angle)
{
myservo.write(write_angle);
timer=millis();
}


