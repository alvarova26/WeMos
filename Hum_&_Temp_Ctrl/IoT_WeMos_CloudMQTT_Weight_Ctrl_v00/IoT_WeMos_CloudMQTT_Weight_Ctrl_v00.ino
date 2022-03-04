#include <HX711.h>

#define DOUT 16
#define SCK 5

HX711 scale;

float calibration_factor = 2150;

void setup() {
  Serial.begin(115200);
  Serial.println("serial ok");
  scale.begin(DOUT, SCK);
  
  scale.set_scale(calibration_factor);// <- set here calibration factor!!!
  scale.tare();
}

void loop() {

  if (scale.is_ready()) {
    long reading = scale.read();
    Serial.print("HX711 read: ");
    Serial.println(reading);
    float weight = scale.get_units(1);
    Serial.print("HX711 get_units(1): ");
    Serial.println(String(weight, 2));
  } else {
    Serial.println("HX711 not found.");
  }
  delay(1000); 
  
  if(Serial.available())
  {
    char temp = Serial.read();
    if(temp == '+' || temp == 'a')
      calibration_factor += 10;
    else if(temp == '-' || temp == 'z')
      calibration_factor -= 10;
  }
  Serial.println("calibration_factor");
  Serial.println(calibration_factor);

}
