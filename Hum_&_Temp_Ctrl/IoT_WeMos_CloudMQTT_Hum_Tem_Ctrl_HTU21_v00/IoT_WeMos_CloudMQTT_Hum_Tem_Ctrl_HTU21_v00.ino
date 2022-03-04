#include <Wire.h>
#include <HTU21D.h>

#define SDA 0 //GPIO0 <=> D8
#define SCL 2 //GPIO2 <=> D9

HTU21D htu;

float temp_VALUE = 0;
float hum_VALUE = 0;

void setup()
{
  Serial.begin(115200); //Init the serial port
  Wire.begin(SDA, SCL); //Init the wireing library
  htu.begin();
}

void loop() {
  temp_VALUE = htu.readTemperature();
  Serial.print(temp_VALUE);  //Print readings
  Serial.print(" °C \t");  //Print readings
  hum_VALUE = htu.readHumidity();
  Serial.print(hum_VALUE);  //Print readings
  Serial.println(" %");  //Print readings

  delay(5000);
}
