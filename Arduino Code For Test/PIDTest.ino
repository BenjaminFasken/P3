#include <PID_v1.h>
#include "Dynamixel_Serial.h"
#include <SoftwareSerial.h>


#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

int pos = 0;

int pos1 = 1500;
int pos2 = 2500;

int maxVal;
int minVal;

int lock = 0;

long posMillis;
long constMillis;

double setPoint1, Input1, Output1;
double setPoint2, Input2, Output2;
double setPoint3, Input3, Output3;
double setPoint4, Input4, Output4;
double setPoint5, Input5, Output5;

int joint5 = A4;
int joint4 = A3;
int joint3 = A2;
int joint2 = A1;
int joint1 = A0;

int gripperButton = 7;
int gripperState = 0;

PID ax1(&Input1, &Output1, &setPoint1, 0.8404, 10, 0, DIRECT);
PID ax2(&Input2, &Output2, &setPoint2, 1.1193, 200, 0, DIRECT);
PID ax3(&Input3, &Output3, &setPoint3, 0.833, 150, 0, DIRECT);
PID ax4(&Input4, &Output4, &setPoint4, 1.0904, 200, 0, DIRECT);
PID ax5(&Input5, &Output5, &setPoint5, 1.5582/2, 2, 0, DIRECT);

SoftwareSerial mySerial(10, 11);    // RX, TX

unsigned long previousMillis = 0;
const long interval = 10000;

void setup() {

  pinMode(gripperButton, INPUT_PULLUP);
  pinMode(joint1, INPUT);
  pinMode(joint2, INPUT);
  pinMode(joint3, INPUT);
  pinMode(joint4, INPUT);
  pinMode(joint5, INPUT);
  
  Serial.flush();                                       // Clear the serial buffer of garbage data before running the code.
  mySerial.begin(SERVO_SET_Baudrate);                   // We now need to set Ardiuno to the new Baudrate speed 115200
  Serial.begin(57600);                                  // Start serial communication on baudrate 57600
  Dynamixel.begin(mySerial);                            // Calling mySerial function which sets 10 pin as the 2nd RX serial pin, and sets pin 11 as the 2nd TX serial pin
  Dynamixel.setDirectionPin(SERVO_ControlPin);

  Dynamixel.setHoldingTorque(0x01, true);
  Dynamixel.setHoldingTorque(0x02, true);
  Dynamixel.setHoldingTorque(0x03, true);
  Dynamixel.setHoldingTorque(0x04, true);
  Dynamixel.setHoldingTorque(0x05, true);
  Dynamixel.setHoldingTorque(0x06, true);

  ax1.SetMode(AUTOMATIC);
  ax1.SetOutputLimits(-200,200);
  ax1.SetSampleTime(1);
  ax2.SetMode(AUTOMATIC);
  ax2.SetOutputLimits(-300,300);
  ax2.SetSampleTime(1);
  ax3.SetMode(AUTOMATIC);
  ax3.SetOutputLimits(-200,200);
  ax3.SetSampleTime(1);
  ax4.SetMode(AUTOMATIC);
  ax4.SetOutputLimits(-200,200);
  ax4.SetSampleTime(1);
  ax5.SetMode(AUTOMATIC);
  ax5.SetOutputLimits(-200,200);
  ax5.SetSampleTime(1);


}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  gripperState = digitalRead(gripperButton);

  if (gripperState == HIGH) {
    Dynamixel.setGoalPosition(0x06, 3505);
  } if (gripperState == LOW) {
    Dynamixel.setGoalPosition(0x06, 3650);
  }

  setPoint5 = map(analogRead(joint5), 0,1024,0,4095);
  setPoint4 = map(analogRead(joint4), 1024,0,512,3584);
  setPoint3 = map(analogRead(joint3), 0,1024,512,3584);
  setPoint2 = map(analogRead(joint2), 1024,0,512,3584);
  setPoint1 = map(analogRead(joint1), 1024,0,512, 5000);

//  if (currentMillis - previousMillis >= interval) {
//    previousMillis = currentMillis; 
//
//    if (pos == 1) {
//      posMillis = currentMillis;
//      //Serial.print(posMillis);
//      setPoint1 = pos1;
//      setPoint2 = 2048;
//      setPoint3 = pos1;
//      setPoint4 = pos1;
//      setPoint5 = pos1;
//      pos = 2;
//    } else {
//      posMillis = currentMillis;
//      //Serial.print(posMillis);
//      setPoint1 = pos1;
//      setPoint2 = 2048;
//      setPoint3 = pos2;
//      setPoint4 = pos1;
//      setPoint5 = pos1;
//      pos = 1;
//    }
//     
//  }
  
  Input1 = Dynamixel.getPosition(0x01);
  Input2 = Dynamixel.getPosition(0x02);
  Input3 = Dynamixel.getPosition(0x03);
  Input4 = Dynamixel.getPosition(0x04);
  Input5 = Dynamixel.getPosition(0x05);

  ax1.Compute();

  if (Output1 > 0) {
    Output1 = map(Output1, 1, 300, 15, 300);
  } else if (Output1 < 0) {
    Output1 = map(Output1, -1, -300, -15, -300);
  }

  ax2.Compute();

  if (Output2 > 0) {
    Output2 = map(Output2, 1, 200, 5, 200);
  } else if (Output2 < 0) {
    Output2 = map(Output2, -200, -1, -200, -5);
  }

  ax3.Compute();

  if (Output3 > 0) {
    Output3 = map(Output3, 1, 300, 13, 300);
  } else if (Output3 < 0) {
    Output3 = map(Output3, -1, -300, -13, -300);
  }

  ax4.Compute();

  if (Output4 > 0) {
    Output4 = map(Output4, 1, 200, 13, 200);
  } else if (Output4 < 0) {
    Output4 = map(Output4, -1, -200, -13, -200);
  }

  ax5.Compute();

  if (Output5 > 0) {
    Output5 = map(Output5, 1, 200, 13, 200);
  } else if (Output5 < 0) {
    Output5 = map(Output5, -1, -200, -13, -200);
  }


  Dynamixel.setGoalPWM(0x01,Output1);
  Dynamixel.setGoalPWM(0x02,Output2);
  Dynamixel.setGoalPWM(0x03,Output3);
  Dynamixel.setGoalPWM(0x04,Output4);
  Dynamixel.setGoalPWM(0x05,Output5);
//
  Serial.print(setPoint1);
  Serial.print("  ");
  Serial.print(setPoint2);
  Serial.print("  ");
  Serial.print(setPoint3);
  Serial.print("  ");
  Serial.print(setPoint4);
  Serial.print("  ");
  Serial.print(setPoint5);
  Serial.print("  ");
  Serial.println(gripperState);
  

}
