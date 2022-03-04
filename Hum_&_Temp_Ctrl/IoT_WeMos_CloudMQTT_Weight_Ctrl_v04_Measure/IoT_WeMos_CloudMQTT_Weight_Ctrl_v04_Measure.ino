#include <HX711.h>

#define DOUT 16 // GPIO16 <=> D2
#define SCK 5 // GPIO5 <=> D3

HX711 LoadCell;

void setup() {
  Serial.begin(115200);
  Serial.println("HX711 Measure code");
  LoadCell.begin(DOUT,SCK); //Initialize LoadCell
  LoadCell.set_gain(128); //Set gain of ADC - It could be 64 or 128 (the last one default value)
  LoadCell.set_scale(378); //Start scale of LoadCell - 378 calibration factor form calibrating code
  LoadCell.tare(); //Tare the scale of LoadCell

  delay(1000);
}

void loop() {
  float current_weight=LoadCell.get_units(10);  //Get average of 20 scale readings
  if (current_weight < 0)
    current_weight = 0;
  Serial.println(current_weight);  //Print readings
  delay(1000);
}
