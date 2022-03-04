#include <HX711.h>

#define DOUT 16
#define SCK 5

HX711 LoadCell;

// ## How to calibrate your load cell
// 1. Call set_scale();
// 2. Call tare();
// 3. Place a known weight on the scale and call get_units(10);
// 4. Divide the result in step 3 to your known weight. You should get about the parameter you need to pass to set_scale(a);
// 5. Adjust the parameter in step 4 until you get an accurate reading

// ## scale_factor with gain @ 64
// 8grs         175
// 182grs       189
// 502grs       189
// 667grs       189
// 1018grs      189
// 1931grs      189

// ## scale_factor with gain @ 128
// 8grs         357
// 182grs       377
// 502grs       377
// 933grs       378       
// 1255grs      377
// 1770grs      378


void setup() {
  Serial.begin(115200);
  Serial.println("HX711 calibration code");
  LoadCell.begin(DOUT,SCK); //Initialize LoadCell
  LoadCell.set_gain(128); //Set gain of ADC - It could be 64 or 128 (the last one default value)
  LoadCell.set_scale(); //Start scale of LoadCell
  LoadCell.tare(); //Tare the scale of LoadCell

  delay(1000);
}

void loop() {
  float current_weight=LoadCell.get_units(20);  //Get average of 20 scale readings
  float scale_factor=(current_weight/1770);  //Divide the result by a known weight (7grs - locker's key, 667grs - bananas, 182grs - cellphone)
  Serial.println(scale_factor);  // Print the scale factor to use in future projects
  delay(1000);
}
