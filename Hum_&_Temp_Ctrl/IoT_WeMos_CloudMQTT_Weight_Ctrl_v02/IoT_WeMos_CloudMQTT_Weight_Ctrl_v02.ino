#include <HX711.h>

#define DOUT 16
#define SCK 5

HX711 scale;

// ############# How to setup the HX711 #############
// y = ax + b => a = (y - b) / x or a = (y - b) / x_avg
// Where:
// y is the actual weight in whatever units you want (g, kg, oz, etc)
// x is the raw value from the HX711 - result from scale.read_average(times);
// a is the slope (multiplier) of the curve
// b is the intersection (offset) with y axis - result from scale.read_average(times); without any weight or the result of scale.tare();
// Then set these values to the HX711:
// scale.set_scale(a);
// scale.set_offset(b);
long y = 7;
long x = 0;
long x_avg = 0;
long b = 0;
long a = 0;


// ## How to calibrate your load cell
// 1. Call set_scale();
// 2. Call tare();
// 3. Place a known weight on the scale and call get_units(10);
// 4. Divide the result in step 3 to your known weight. You should get about the parameter you need to pass to set_scale(a);
// 5. Adjust the parameter in step 4 until you get an accurate reading

long read = 0; //Stores the raw readings form the sensor (waits for the chip to be ready and returns a single measurement)
long read_average = 0; //Stores the average of times readings
double get_value = 0; //Stores read_average() - OFFSET, that is the current value without the tare weight (OFFSET - value from tare() method
float get_units = 0; //Stores get_value() divided by SCALE, that is the raw value divided by a value obtained via calibration
float get_scale = 0; //Stores the result of get_scale();, whereas SCALE is sent as parameter in the funcion set_scale(SCALE);
long get_offset = 0; //Stores the result of get_offset();, actuallty the value of the OFFSET (tare weight)

float calibration_factor = 2125; //-7050 calibration value
long zero_factor = 0; //Determins the baseline of measurements
int times = 10; //Times of readings raw values before retun the measurement

void setup() {
  Serial.begin(115200);
  Serial.println("HX711 calibration code");
  Serial.println("Remove all weight from scale");
  Serial.println("After readings begin, place known weight on scale");
  scale.begin(DOUT,SCK);
  scale.set_scale();
  scale.tare();

  zero_factor = scale.read_average(times); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);

  x = scale.read();
  x_avg = scale.read_average(times);
  b = scale.get_offset();
  a = y - b;
  a = a / x;

  Serial.print("x: ");
  Serial.println(x);
  Serial.print("x_avg: ");
  Serial.println(x_avg);
  Serial.print("b: ");
  Serial.println(b);
  Serial.print("a: ");
  Serial.println(a);



}

void loop() {

  x = scale.read();
  x_avg = scale.read_average(times);
  b = scale.get_offset();
  a = y - b;
  a = a / x;

  Serial.print("x: ");
  Serial.println(x);
  Serial.print("x_avg: ");
  Serial.println(x_avg);
  Serial.print("b: ");
  Serial.println(b);
  Serial.print("a: ");
  Serial.println(a);

  delay(10000);
}