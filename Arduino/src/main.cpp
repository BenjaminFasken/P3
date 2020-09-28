/*
* hello
* sfdg
* sdfg
*/

#include <Arduino.h>
#include "Dynamics.h"

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  double joints[4] = {1,1,1,1};
  double x = 1; double y = 1; double z = 1; double pitch = 1;
  auto dyna = Dynamics();
  dyna.getInvKinJoints( joints,  x,  y,  z,  pitch);





  // turn the LED on (HIGH is the voltage level)
  Serial.write("hello");
  digitalWrite(LED_BUILTIN, HIGH);
  // wait for a second
  delay(1000);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
   // wait for a second
  delay(1000);
}
