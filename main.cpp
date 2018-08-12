/*
  main.cpp - Main loop for Arduino sketches
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#define F_CPU 16000000UL
#include "Arduino.h"
#include "Wire.h"
#include "Timer1.h"
#include "PID_v1.h"

//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=2, aggKi=0, aggKd=0;
double consKp=1, consKi=0.05, consKd=0.25;
int consA = 0;
String kp="",ki="",kd="";

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//定时器
Timer1 tc1;
void report_encode();
void increase();

//上电获知自己的地址
int slave_address = -1;
int getSLA_ADR();


int getSLA_ADR()
{
	//8引脚公共端——GND
	pinMode(8,OUTPUT);
	digitalWrite(8,LOW);
	
	//9、10、11、12地址识别引脚——上拉电阻
	for (int i=9;i<=12;i++)
	{
		pinMode(i,INPUT_PULLUP);
		if (digitalRead(i)==LOW)
		{
			slave_address = i-8;
		}
	}
	Serial.print("slave_address:");
	Serial.println(slave_address);
	pinMode(8,INPUT_PULLUP);
	return slave_address;
}

//全局变量区
int MAX_encode_num = 0;
int encode_num = 0;
int ADC_val = 0;
bool received = false;

//i2c 中断函数 
void receiveEvent(int howMany);
void requestEvent();

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

void setupUSB() __attribute__((weak));
void setupUSB() { }


int main(void)
{
	init();

	initVariant();
	
#if defined(USBCON)
	USBDevice.attach();
#endif
	
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
        
	return 0;
}


#define VRX A5
#define VRY A4
#define interruptPin 2
#define motor_A 5
#define motor_B 6

void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);
	//设置中断
	Serial.print("");
	//从外部跳线获取从机地址
	getSLA_ADR();
	
	Wire.begin(slave_address);                // join i2c bus with address #8
	Wire.onReceive(receiveEvent); // register event
	Wire.onRequest(requestEvent); // register event
	 
	pinMode(interruptPin,INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(interruptPin), increase, FALLING);
	//设置定时器
	tc1.setMode("CTC",50);
	tc1.attachInterrupt(report_encode);
	//驱动电机
	pinMode(motor_A,OUTPUT);
	analogWrite(motor_A,127);
	pinMode(motor_B,OUTPUT);
	digitalWrite(motor_B,LOW);
}

void loop() {
	if (consA>0)
	{
		analogWrite(motor_A, consA);
		analogWrite(motor_B,0);
	}
	else
	{
		analogWrite(motor_A,0);
		analogWrite(motor_B,-consA);
	}
	Serial.print(slave_address);
	Serial.print("   ");
	Serial.println(consA);
	
}

void report_encode()
{
	//编码器结账
	MAX_encode_num = encode_num;
	encode_num = 0;
	//Serial.print("A:");
	//Serial.print(Setpoint);
	//Serial.print("B:");
	//Serial.print(MAX_encode_num);
	//Serial.print("kp:");
	//Serial.print(consKp);
	//Serial.print("ki:");
	//Serial.print(consKi);
	//Serial.print("kd:");
	//Serial.println(consKd);
}


void increase()
{
	encode_num++;
}


void receiveEvent(int howMany){
	String str = Wire.readStringUntil('\n');
	str.toLowerCase();
	
	int pos = str.indexOf('\n');
	String str1 = str.substring(0,pos);
	
	int pos_A = str.indexOf("a:");
	
	String str_A = str.substring(pos_A+2);
	
	consA = str_A.toInt();

		received = true;
}

void requestEvent() 
{
	if (received)
	{
		Wire.write(slave_address);
	}
}