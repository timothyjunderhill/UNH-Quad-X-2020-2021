#include <Arduino.h> 

class QuadCopter
{
    public:
        float pitch, roll, yaw;
    
    void GetActualAttitude()
    {
        pitch = 10;
        roll = 10;
        yaw = 10;
        Serial.println(roll);
        delay(1000);
    }
};