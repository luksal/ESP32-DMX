#include <dmx.h>

int readcycle = 0;

void setup() {
  Serial.begin(115200);
  DMX::Initialize();
}

void loop()
{
  if(millis() - readcycle > 1000)
  {
    readcycle = millis();

    Serial.print(readcycle);
      
    if(DMX_healthy())
    {
      Serial.print(": ok - ");
    }
    else
    {
      Serial.print(": fail - ");
    }
    Serial.print(DMX_read(1));
    Serial.print(" - ");
    Serial.print(DMX_read(110));
    Serial.print(" - ");
    Serial.println(DMX_read(256));
  }
}
