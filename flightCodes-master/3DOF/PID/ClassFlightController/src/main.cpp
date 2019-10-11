#include <Arduino.h>
#include <QuadCopter.hpp>


QuadCopter JoshRocks;

void setup() {
  Serial.begin(115200);
  while(!Serial)
  Serial.println("Finished setup");
}

void loop()
{
  
}