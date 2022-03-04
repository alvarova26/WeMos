#include <HX711.h>

#define DOUT 16
#define SCK 5

HX711 scale;


// ## How to calibrate your load cell
// 1. Call `set_scale()` with no parameter.
// 2. Call `tare()` with no parameter.
// 3. Place a known weight on the scale and call `get_units(10)`.
// 4. Divide the result in step 3 to your known weight. You should
//    get about the parameter you need to pass to `set_scale()`.
// 5. Adjust the parameter in step 4 until you get an accurate reading.

float calibration_factor = 2125; //-7050 worked for my 440lb max scale setup
long zero_factor = 0;
float units;
float ounces;

void setup() {
  Serial.begin(115200);
  Serial.println("HX711 calibration sketch");
  Serial.println("Remove all weight from scale");
  Serial.println("After readings begin, place known weight on scale");
  Serial.println("Press + or a to increase calibration factor");
  Serial.println("Press - or z to decrease calibration factor");
  scale.begin(DOUT,SCK);
  scale.set_scale();
  scale.tare();	//Reset the scale to 0

  zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
}

void loop() {

  scale.set_scale(calibration_factor); //Adjust to this calibration factor

  Serial.print("Reading: ");
  units = scale.get_units(), 10;
  if (units < 0) {
    units = 0.00;
  }
  ounces = units * 0.035274;
  Serial.print(units);
  Serial.print(" grams"); 
  Serial.print(" calibration_factor: ");
  Serial.print(calibration_factor);
  Serial.println();

  if(Serial.available())
  {
    char temp = Serial.read();
    if(temp == '+' || temp == 'a')
      calibration_factor += 10;
    else if(temp == '-' || temp == 'z')
      calibration_factor -= 10;
  }
}