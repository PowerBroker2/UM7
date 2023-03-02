#include "UM7.h"




UM7 myIMU;




void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);

  myIMU.begin(Serial1);

  while(!Serial);
}




void loop()
{
  if (myIMU.available())
  {
    Serial.print(myIMU.acclT());
    Serial.print(' ');
    Serial.print(myIMU.acclX());
    Serial.print(' ');
    Serial.print(myIMU.acclY());
    Serial.print(' ');
    Serial.print(myIMU.acclZ());
    Serial.println();
    Serial.println();
  }
}