// ############################## IoT Project to Pub/Sub measurements via MQTT Broker CloudMQTT 

/* The device (WeMos D1 R1 chipset with ESP8266 onborad) will report:
- Temperature (°C)
- Humidity (%)
- Weight (grs)
- among other measurements every 10 sec via CloudMQTT Broker (using mqtt protocol)
At the same time, WeMos accepts commands form MQTT Broker (Publish/subscribe)  */

/* ############################## Bill of Materials
WeMos D1 R1     1 => Chipset with ESP8266 onboard
HTU21D          1 => Temperature and Humidity sensor (I2C)
Relay           2 => Relay to control the Refrigerator and Humidifier
LoadCell 5kg    1 => Bar to measure until 5kg at full scale
HX711 (Amp)     1 => Amplifier/filter used with the LoadCell
*/

/* ################################################################################################### */
/* ############################## CHECK NOTE FOR DETAILS => libraries, values, behavior, version, etc. */
/* ################################################################################################### */

/* ################################################################################################### */
/* ################################################################################################### */
/* ################################################################################################### */
/* ############################################# C O D E ############################################# */
/* ################################################################################################### */
/* ################################################################################################### */
/* ################################################################################################### */

// ############################## Include Libraries
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Chrono.h>
#include <HX711.h>
#include <Wire.h>
#include <HTU21D.h>

// ############################## Define PINs equivalences
#define D0    3
#define D1    1
#define D2    16
#define D3    5
#define D4    4
#define D5    14
#define D6    12
#define D7    13
#define D8    0
#define D9    2
#define D10   15

#define DOUT D2
#define SCK D3
#define SDA D8
#define SCL D9

// ############################## WiFi Netwrok Parameters
//const char* ssid = "Vanessa"; //Change this to your network SSID (name).
//const char* pass = "vanessa3012"; //Change this your network password

// const char* ssid = "GVT-26D4"; //Change this to your network SSID (name).
// const char* pass = "0162628689"; //Change this your network password

const char* ssid = "Rede_RF01 2.4"; //Change this to your network SSID (name).
const char* pass = "redeclaro03"; //Change this your network password

// const char* ssid = "Rede_RF02 5G"; //Change this to your network SSID (name).
// const char* pass = "redeclaro04"; //Change this your network password

//const char* ssid = "Rede_Infra02 2.4"; //Change this to your network SSID (name).
//const char* pass = "redeclaro02"; //Change this your network password

// ############################## MQTT Broker Parameters
const char* mqttServer = "postman.cloudmqtt.com"; //Server MQTT
const char* mqttUserName = "gykeasns"; //Username provided by the MQTT Broker
const char* mqttPass = "yj7v4U7Pfe9A"; //Password provided by the MQTT Broker
const char* mqttClientID = "WeMosLabClient"; //ClientID provided by the MQTT Broker (MUST BE UNIQUE for each PUB/SUB - i.e. the app on the cell phone must be different)
const char* topicWeMosLabReadings = "WeMosLabReadings"; //Topic to send WeMosLab readings to MQTT Broker / Subs
const char* topicUpdateWeMosLab = "UpdateWeMosLab"; //Topic to get Update values to WeMosLab from MQTT Broker / Pubs
const int mqttPort = 11094; //Port MQTT
unsigned int length = 0; //Length incoming MQTT msg
// ############################## Refrigerator Parameters
int ref_PIN = D6; //What pin is the refrigerator's relay command connected to
int temp_CORR = 0; //Correction value for temperature read from the HTU11 sensor
int des_temp = 11; //Desired temperature, wich will define the aim temperature of the system
int delta_temp = 1; //Detla tempertaure allowed, which will establish upper and lower thresholds for relay activation/deactivation
int upper_temp = des_temp + delta_temp; //Upper temperature allowed, once reached, turn ON the refrigerator
int lower_temp = des_temp - delta_temp; //Lower temperature desired, once reached, turn OFF the refrigerator
float temp_VALUE = 0; //Will store the temperature value comming from HTU
float temp_VALUE_mem = 0; //Will store the temperature value as an average of last 5 previous measurements
bool ref_stat = 0; //Refrigerator's status
double ref_UP = 0; //Refrigerator's time UP (working). Will be percentage: ref_UP=(time_ref_on.elapsed()*100)/(time_ref_on.elapsed()+time_ref_on.elapsed()
double ref_DOWN = 0; //Refrigerator's time UP (working). Will be percentage: ref_UP=(time_ref_on.elapsed()*100)/(time_ref_on.elapsed()+time_ref_on.elapsed()
Chrono time_ref_on(Chrono::SECONDS);
float mem_temp[5] = {0, 0, 0, 0, 0}; //Array for average calculations (memory of 5 previous values)
bool init_ref = 1; //Flag for init regrigerator (mem_logic)

// ############################## Humidifier Parameters
int hum_PIN = D7; //What pin is the humidifier's relay command connected to
int hum_CORR = 0; //Correction value for humidity read from the HTU11 sensor
int des_hum = 80; //Desired humidity, wich will define the aim humidity of the system
int delta_hum = 5; //Detla humidity allowed, which will establish upper and lower thresholds for relay activation/deactivation
int upper_hum = des_hum + delta_hum; //Upper humidity desired, once reached, turn OFF the humidifier
int lower_hum = des_hum - delta_hum; //Lower humidity allowed, once reached, turn ON the humidifier
float hum_VALUE = 0; //Will store the humidity value comming from HTU
float hum_VALUE_mem = 0; //Will store the humidity value as an average of last 5 previous measurements
bool hum_stat = 0; //Humidifier's status
double hum_UP = 0;
double hum_DOWN = 0;
Chrono time_hum_on(Chrono::SECONDS);
float mem_hum[5] = {0, 0, 0, 0, 0}; //Array for average calculations (memory of 5 previous values)
bool init_hum = 1; //Flag for init humidifier (mem_logic)

// ############################## LoadCell Parameters
float initial_weight = 1800; //Initial value off the weight to be measuresd in grs
int weight_CORR = 0; //Correction value for weight read from the LoadCell in grs
float weight_VALUE = 0; //Will store the weight value comming from LoadCell in grs
float weight_VALUE_mem = 0; //Will store the weight value as an average of last 5 previous measurements in grs
float mem_weight[5] = {0, 0, 0, 0, 0}; //Array for average calculations (memory of 5 previous values) in grs
float weight_perc = 0; //Will store the percentage of current weight compared to initial_weight
bool init_weight = 1; //Flag for init LoadCell (mem_logic)
bool tare = 0; //Flag for tare LoadCell

// ############################## Miscellaneous Parameters
int correction = 0; //Correction value sent by reference to serial_print auxiliar funcion
float value = 0; //Value of the parameter to be printed sent by reference to serial_print auxiliar funcion
float value_mem = 0; //Value_mem of the parameter to be printed sent by reference to serial_print auxiliar funcion
char measure = 'u'; //Code for printing (t=temperarutr | h=humidity | w=weight | a=after | b=before) sent to auxiliar funcion
float rssi_VALUE = 0; //Will store the RSSI measuremente form ESP8266
Chrono abs_time(Chrono::SECONDS);
long running_time = 0; //Global time
bool init_mem = 1; //Flag for init memory logic (fill with the first value the memory array)

// ############################## Sensor's status Parameters
unsigned int ref_sensor_COUNT = 0; //Counter for sensor's measurements error
unsigned int hum_sensor_COUNT = 0; //Counter for sensor's measurements error
unsigned int weight_sensor_COUNT = 0; //Counter for sensor's measurements error
bool ref_sensor_STAT = 0; //Flag for sesnor status (current error status)
bool hum_sensor_STAT = 0; //Flag for sesnor status (current error status)
bool weight_sensor_STAT = 0; //Flag for sesnor status (current error status)
bool ref_sensor_MEM = 0; //Flag for sesnor error memory - if measure fails, this flags turns to 1 and remains until reset
bool hum_sensor_MEM = 0; //Flag for sesnor error memory - if measure fails, this flags turns to 1 and remains until reset
bool weight_sensor_MEM = 0; //Flag for sesnor error memory - if measure fails, this flags turns to 1 and remains until reset

// ############################## System Parameters
bool reset_SYS = 0; //Flag for system reset - if received, calls resetFunc();
bool reset_REF_relay = 0; //Flag for refrigerator's relay reset - if received, calls reset_relay('r');
bool reset_HUM_relay = 0; //Flag for refrigerator's relay reset - if received, calls reset_relay('h');
bool sys_init = 1; // Flag for system initialization - used to report any HW or SW reset => published in WeMosLabReadings

// ############################## Initialize Libraies
WiFiClient client; //Initialize the Wifi client library
PubSubClient mqttClient(client); //Initialize the PuBSubClient library
HTU21D htu; //Initialize HTU21D library
HX711 LoadCell; //Initialize HX711 library

// ############################## Functions Prototypes
void setup_serial();
void setup_pinout();
void setup_wiring();
void setup_htu();
void setup_loadcell();
void setup_wifi();
void reconnect_wifi();
void setup_mqtt();
void reconnect_mqtt();
void check_wifi();
void check_mqtt();
void callback(char* topic, byte* payload, unsigned int length);
void update_WeMosLab (String inputmsg);
void temp_logic();
void hum_logic();
void weight_logic();
void publish_readings();
void print_measurements(float value, float value_mem, int correction, char par);
void setup_chronos();
void get_chronos();
void print_chronos();
void print_parameters (char par);
float mem_logic(float value, bool init_mem, char par);
void print_sensor();
void reset_relay (char par);
void loadcell_tare();


// ############################## Setup main functions, units and variables
void setup()
{
  setup_serial();
  setup_pinout();
  setup_wiring();
  setup_htu();
  setup_loadcell();
  setup_chronos();
  setup_wifi();
  setup_mqtt();
}

// ############################## Loop function - Code runs over and over again
void loop()
{
  check_wifi();
  check_mqtt();
  mqtt_loop();
  temp_logic();
  hum_logic();
  weight_logic();
  print_sensor();
  print_chronos();
  publish_readings();
  delay (10000);
}

// ############################## AUX FUNCTIONS
void setup_serial()
{
  Serial.begin(115200); //Initialize the serial port (USB Commm)
  Serial.println();
  Serial.println("Initializing serial port...");
  delay (500);
  Serial.println("Done!");
}

// ############################## AUX FUNCTIONS
void setup_pinout()
{
  Serial.println("Initializing PINOUT...");
  delay (500);
  pinMode(D2, INPUT); //PIN GPIO16 for HX711 DOUT LoadCell
  pinMode(D3, OUTPUT); //PIN GPIO5 for HX711 SCK LoadCell
  pinMode(D6, OUTPUT); //PIN GPIO12 for Humidifyer's relay
  pinMode(D7, OUTPUT); //PIN GPIO13 for Refrigerator's relay
  pinMode(D8, INPUT); //PIN GPIO0 for HTU21D SDA Temp & Hum Sensor
  pinMode(D9, OUTPUT); //PIN GPIO2 for HTU21D SCL Temp & Hum Sensor
  digitalWrite(D6, HIGH); //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - Humidifier OFF
    hum_stat = 0;
  digitalWrite(D7, HIGH); //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - Refrigerator ON
    ref_stat = 1;
  Serial.println("Done!");
}

// ############################## AUX FUNCTIONS
void setup_wiring()
{
  Serial.println("Initializing wiring...");
  delay (500);
  Wire.begin(SDA, SCL); //Initialize the wiring library
  Serial.println("Done!");

}

// ############################## AUX FUNCTIONS
void setup_htu()
{
  Serial.println("Initializing sensor HTU21D...");
  delay (500);
  htu.begin(); //Initialize HTU21D to send HUM & TEMP readings
  temp_VALUE = htu.readTemperature(); //Read the temperature value form the HTU
  hum_VALUE = htu.readHumidity(); //Read the humidity value form the HTU
  Serial.println("Done!");
}

// ############################## AUX FUNCTIONS
void setup_loadcell()
{
  Serial.println("Initializing LoadCell weight sensor...");
  delay (500);
  LoadCell.begin(DOUT,SCK); //Initialize LoadCell
  LoadCell.set_gain(128); //Set gain of ADC - It could be 64 or 128 (the last one default value)
  LoadCell.set_scale(378); //Start scale of LoadCell - 378 calibration factor form calibrating code
  LoadCell.tare(); //Tare the scale of LoadCell
  weight_VALUE = LoadCell.get_units(5);  //Get average of 5 scale readings
  Serial.println("Done!");
}

// ############################## AUX FUNCTIONS
void setup_wifi()
{
  Serial.print("Connecting to WiFi Network ");
  Serial.println(ssid);

  delay (500);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println(".");
  }
  Serial.println();
  Serial.println("WiFi Network connected!");
}

// ############################## AUX FUNCTIONS
void reconnect_wifi()
{
  Serial.println("Disconnected from WiFi Network!");
  Serial.print("Trying to reconnect to WiFi Network ");
  Serial.println(ssid);

  //Initialize WiFi and try to connect the WiFi Network
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println(".");
  }
  Serial.println();
  Serial.println("Connected again!");

}

// ############################## AUX FUNCTIONS
void setup_mqtt()
{
  //Set the MQTT broker parameters and try to connect the MQTT Broker
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);

  Serial.print("Connecting to MTQQ Broker ");
  Serial.println(mqttServer);

  delay (500);
  while (!mqttClient.connected())
  {
    Serial.println(".");
    mqttClient.connect(mqttClientID, mqttUserName, mqttPass);

    if (!mqttClient.connected())
    {
      // If connection fails, print the ErrorCode in order to know what happened
      // Error Code reference https://pubsubclient.knolleary.net/api.html#state for the failure code explanation
      Serial.println("Connection failed, ErrorCode=");
      Serial.print(mqttClient.state());
      Serial.println("Trying to connect again in 5 seconds...");
      delay(5000);
    }
  }
  Serial.println();
  Serial.println("MQTT Broker connected!");

  mqttClient.publish(topicWeMosLabReadings, "Online");
  mqttClient.subscribe(topicUpdateWeMosLab);
}

// ############################## AUX FUNCTIONS
void reconnect_mqtt()
{
  Serial.print("Disconnected from MQTT Broker!");
  Serial.println("Trying to reconnect to MTQQ Broker... ");
  Serial.println(mqttServer);

  while (!mqttClient.connected())
  {
    Serial.println(".");
    mqttClient.connect(mqttClientID, mqttUserName, mqttPass);

    if (!mqttClient.connected())
    {
      // If connection fails, print the ErrorCode in order to know what happened
      // Error Code reference https://pubsubclient.knolleary.net/api.html#state for the failure code explanation
      Serial.println("Connection failed, ErrorCode=");
      Serial.print(mqttClient.state());
      Serial.println("Trying to connect again in 5 seconds...");
      delay(5000);
    }
  }
  Serial.println();
  Serial.println("MQTT Broker connected again!");

  mqttClient.publish(topicWeMosLabReadings, "Online again");
  mqttClient.subscribe(topicUpdateWeMosLab);
}

// ############################## AUX FUNCTIONS
void setup_chronos()
{
  Serial.println("Initializing Chronos (timers of the system)");
  delay (500);
  abs_time.restart();
  time_ref_on.restart();
  time_hum_on.restart();
  time_hum_on.stop();
  Serial.println("Done!");
}

// ############################## AUX FUNCTIONS
void check_wifi()
{
  if (WiFi.status() != WL_CONNECTED) //Reconnect WiFi if got disconnected
    reconnect_wifi();
}

// ############################## AUX FUNCTIONS
void check_mqtt()
{
  if (!mqttClient.connected()) //Reconnect MQTT client if not connected
    reconnect_mqtt();
}

// ############################## AUX FUNCTIONS
void mqtt_loop()
{
  mqttClient.loop(); //Call the loop continuously to establish connection to the server (keepalive)
}

// ############################## AUX FUNCTIONS
void temp_logic()
{
  temp_VALUE = htu.readTemperature(); //Read the temperature value form the HTU
  if  ((isnan(temp_VALUE)) || (temp_VALUE==998) || (temp_VALUE==999))
  {
    if (isnan(temp_VALUE))
      Serial.println("No measurements comming from sensor. Check your connections!"); //Print the auxiliar text to Serial Comm (USB)
    if (temp_VALUE==998)
      Serial.println("I2C timed out. Check your connections!"); //Print the auxiliar text to Serial Comm (USB)
    if (temp_VALUE==999)
      Serial.println("CRC mismatch. Check your connections!"); //Print the auxiliar text to Serial Comm (USB)

    ref_sensor_COUNT = ref_sensor_COUNT + 1;
    ref_sensor_STAT = 1;
    ref_sensor_MEM = 1;
    digitalWrite(ref_PIN, HIGH); //Write HIGH (5V) on the DIGITAL PIN (realy deactivated) - Refrigerator ON
    if (!ref_stat == 1)
        time_ref_on.resume();
    ref_stat = 1;
    Serial.println("Refrigerator ON due to lack of measurements. Check the sensor."); //Print the auxiliar text to Serial Comm (USB)
  }
  else
  {
    ref_sensor_STAT = 0;
    temp_VALUE_mem = mem_logic(temp_VALUE, init_ref, 't'); //Call mem_logic() => Calculate average of last 5 previous values
    print_measurements(temp_VALUE, temp_VALUE_mem, temp_CORR, 't' ); //Call serial_print () => Print Temperature values
    if ((temp_VALUE_mem - temp_CORR) < lower_temp) //Corrected temperature below lower defined threshold
    {
      digitalWrite(ref_PIN, LOW); //Write LOW (0V) on the DIGITAL PIN (realy activated) - Refrigerator OFF
      if (ref_stat == 1)
        time_ref_on.stop();
      ref_stat = 0;
    }
    if ((temp_VALUE_mem - temp_CORR) >= upper_temp) //Corrected temperature above upper defined threshold
    {
      digitalWrite(ref_PIN, HIGH); //Write HIGH (5V) on the DIGITAL PIN (realy deactivated) - Refrigerator ON
      if (!ref_stat == 1)
        time_ref_on.resume();
      ref_stat = 1;
    }
    if (init_ref == 1)
      init_ref = 0;
  }
}

// ############################## AUX FUNCTIONS
void hum_logic()
{
  hum_VALUE = htu.readHumidity(); //Read the humidity value form the HTU
  if  ((isnan(hum_VALUE)) || (hum_VALUE==998) || (hum_VALUE==999))
  {
    if (isnan(hum_VALUE))
      Serial.println("No measurements comming from sensor. Check your connections!"); //Print the auxiliar text to Serial Comm (USB)
    if (hum_VALUE == 998)
      Serial.println("I2C timed out. Check your connections!"); //Print the auxiliar text to Serial Comm (USB)
    if (hum_VALUE == 999)
      Serial.println("CRC mismatch. Check your connections!"); //Print the auxiliar text to Serial Comm (USB)

    hum_sensor_COUNT = hum_sensor_COUNT + 1;
    hum_sensor_STAT = 1;
    hum_sensor_MEM = 1;
    digitalWrite(hum_PIN, HIGH); //Write HIGH (5V) on the DIGITAL PIN (realy deactivated) - Humidifier OFF
    if (!hum_stat == 0)
      time_hum_on.stop();
    hum_stat = 0;
    Serial.println("Humidifier OFF due to lack of measurements. Check the sensor."); //Print the auxiliar text to Serial Comm (USB)
  }
  else
  {
    hum_sensor_STAT = 0;
    hum_VALUE_mem = mem_logic(hum_VALUE, init_hum, 'h'); //Call mem_logic() => Calculate average of last 5 previous values
    print_measurements(hum_VALUE, hum_VALUE_mem, hum_CORR, 'h' ); //Call serial_print () => Print Humidity values
    if ((hum_VALUE_mem - hum_CORR) < lower_hum) //Corrected Humidity below lower defined threshold
    {
      digitalWrite(hum_PIN, LOW); //Write LOW (0V) on the DIGITAL PIN (realy activated) - Humidifier ON
      if (hum_stat == 0)
        time_hum_on.resume();
      hum_stat = 1;
    }
    if ((hum_VALUE_mem - hum_CORR) >= upper_hum) //Corrected Humidity above upper defined threshold
    {
      digitalWrite(hum_PIN, HIGH); //Write HIGH (5V) on the DIGITAL PIN (realy deactivated) - Humidifier OFF
      if (!hum_stat == 0)
        time_hum_on.stop();
      hum_stat = 0;
    }
    if (init_hum == 1)
      init_hum = 0; 
  }  
}

// ############################## AUX FUNCTIONS
void weight_logic()
{
  weight_VALUE = LoadCell.get_units(5);  //Get average of 5 readings
  if  (isnan(weight_VALUE) || (LoadCell.read() == 0) || (LoadCell.read() == -1)) //LoadCell.read() is 0 when wiring failure, -1 when vcc failure
  {
    Serial.println("Measurements are not comming from the LoadCell. Check the sensor."); //Print the auxiliar text to Serial Comm (USB)
    weight_sensor_COUNT = weight_sensor_COUNT + 1;
    weight_sensor_STAT = 1;
    weight_sensor_MEM = 1;
  }
  else
  {
    if (weight_VALUE < 0)
      weight_VALUE = 0;
    weight_sensor_STAT = 0;
    weight_VALUE_mem = mem_logic(weight_VALUE, init_weight, 'w'); //Call mem_logic() => Calculate average of last 5 previous values
    print_measurements(weight_VALUE, weight_VALUE_mem, weight_CORR, 'w' ); //Call serial_print () => Print Humidity values
    if (init_weight == 1)
      init_weight = 0;
    weight_perc = ((weight_VALUE_mem - weight_CORR) * 100) / initial_weight;
  }  
}

// ############################## AUX FUNCTIONS
float mem_logic(float value, bool init_mem, char par)
{
  int i = 0;
  float aux = 0;

  switch (par)
  {
    case 'h':
      if (init_mem == 1)
      {
        for (i = 0; i < 5; i++)
          mem_hum[i] = value;
        init_mem = 0;
      }
      else
      {
        for (i = 0; i < 4; i++)
          mem_hum[4 - i] = mem_hum[4 - (i + 1)];
        mem_hum[0] = value;
      }

      for (i = 0; i < 5; i++)
        aux += mem_hum[i];
      aux = aux / 5;
      return aux;
    break;

    case 't':
      if (init_mem == 1)
      {
        for (i = 0; i < 5; i++)
          mem_temp[i] = value;
        init_mem = 0;
      }
      else
      {
        for (i = 0; i < 4; i++)
          mem_temp[4 - i] = mem_temp[4 - (i + 1)];
        mem_temp[0] = value;
      }

      for (i = 0; i < 5; i++)
        aux += mem_temp[i];
      aux = aux / 5;
    return aux;
    break;

    case 'w':
      if (init_mem == 1)
      {
        for (i = 0; i < 5; i++)
          mem_weight[i] = value;
        init_mem = 0;
      }
      else
      {
        for (i = 0; i < 4; i++)
          mem_weight[4 - i] = mem_weight[4 - (i + 1)];
        mem_weight[0] = value;
      }

      for (i = 0; i < 5; i++)
        aux += mem_weight[i];
      aux = aux / 5;
    return aux;
    break;

    defualt:
      Serial.println("Something went wrong. Check the program."); //Print the auxiliar text to Serial Comm (USB)
    return -1;
    break;
  }
}

// ############################## AUX FUNCTIONS
void get_chronos()
{
  running_time = abs_time.elapsed();
  ref_UP = time_ref_on.elapsed();
  hum_UP = time_hum_on.elapsed();
  ref_DOWN = running_time - ref_UP;
  hum_DOWN = running_time - hum_UP;
  ref_UP = (ref_UP / running_time) * 100;
  hum_UP = (hum_UP / running_time) * 100;
  ref_DOWN = (ref_DOWN / running_time) * 100;
  hum_DOWN = (hum_DOWN / running_time) * 100;
}

// ############################## AUX FUNCTIONS
void print_chronos()
{
  get_chronos();
  Serial.println("======================================== WeMosLab Chronos of the System | Begin");
  Serial.print("Total time =>\t");
  Serial.print(abs_time.elapsed());
  Serial.println(" sec");
  Serial.println("UP time (sec)\tUP time (%)\tDOWN time (%)");
  Serial.print("Ref \t");
  Serial.print(time_ref_on.elapsed());
  Serial.print("\t");
  Serial.print(String (ref_UP));
  Serial.print("\t\t");
  Serial.println(String (ref_DOWN));
  Serial.print("Hum \t");
  Serial.print(time_hum_on.elapsed());
  Serial.print("\t");
  Serial.print(String (hum_UP));
  Serial.print("\t\t");
  Serial.println(String (hum_DOWN));
  Serial.println("======================================== WeMosLab Chronos of the System | End");
}

// ############################## AUX FUNCTIONS
void publish_readings()
{
  StaticJsonBuffer<1024> JSONbufferRX;
  JsonObject& JSONencoder = JSONbufferRX.createObject();

  Serial.println("======================================== WeMosLab Publish sensor's data (JSON) over MQTT topic | Begin");

  // Temp parameters
  JSONencoder["TEMP"] = String (temp_VALUE);
  JSONencoder["TEMPMEM"] = String (temp_VALUE_mem - temp_CORR);
  JSONencoder["TEMPD"] = String (des_temp);
  JSONencoder["DTEMP"] = String (delta_temp);
  JSONencoder["TEMPCORR"] = String (temp_CORR);
  JSONencoder["REFSTAT"] = String (ref_stat);
  JSONencoder["REFSTATAMP"] = String (ref_stat * 5);
  JSONencoder["REFUP"] = String (ref_UP);
  JSONencoder["REFDOWN"] = String (ref_DOWN);
  // Hum parameters
  JSONencoder["HUM"] = String (hum_VALUE);
  JSONencoder["HUMMEM"] = String (hum_VALUE_mem - hum_CORR);
  JSONencoder["HUMD"] = String (des_hum);
  JSONencoder["DHUM"] = String (delta_hum);
  JSONencoder["HUMCORR"] = String (hum_CORR);
  JSONencoder["HUMSTAT"] = String (hum_stat);
  JSONencoder["HUMSTATAMP"] = String (hum_stat * 10);
  JSONencoder["HUMUP"] = String (hum_UP);
  JSONencoder["HUMDOWN"] = String (hum_DOWN);
  // Weight parameters
  JSONencoder["WEIGHT"] = String (weight_VALUE);
  JSONencoder["WEIGHTMEM"] = String (weight_VALUE_mem - weight_CORR);
  JSONencoder["WEIGHTCORR"] = String (weight_CORR);
  JSONencoder["WEIGHTINIT"] = String (initial_weight);
  JSONencoder["WEIGHTPERC"] = String (weight_perc);
  // Misc parameters
  JSONencoder["RSSI"] = String (WiFi.RSSI());
  JSONencoder["UPTIME"] = String (running_time / 3600);
  // Sensor parameters
  JSONencoder["REFSENSSTAT"] = String (ref_sensor_STAT);
  JSONencoder["REFSENSMEM"] = String (ref_sensor_MEM);
  JSONencoder["REFERRCOUNT"] = String (ref_sensor_COUNT);
  JSONencoder["HUMSENSSTAT"] = String (hum_sensor_STAT);
  JSONencoder["HUMSENSMEM"] = String (hum_sensor_MEM);
  JSONencoder["HUMERRCOUNT"] = String (hum_sensor_COUNT);
  JSONencoder["WEIGHTSENSSTAT"] = String (weight_sensor_STAT);
  JSONencoder["WEIGHTSENSMEM"] = String (weight_sensor_MEM);
  JSONencoder["WEIGHTERRCOUNT"] = String (weight_sensor_COUNT);
  // System reset status
  JSONencoder["SYSINIT"] = String (sys_init);
  JSONencoder["TARE"] = String (tare);

  char JSONmessageBuffer[1024];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.print("Sending message to MQTT topic...");
  Serial.println(topicWeMosLabReadings);
  Serial.println(JSONmessageBuffer);

  if (mqttClient.publish(topicWeMosLabReadings, JSONmessageBuffer) == true)
  {
    Serial.println("======================================== WeMosLab Publish sensor's data (JSON) over MQTT topic | End");
    sys_init = 0;
  }  
  else
  {
    Serial.println("Error sending message!");
  }
}

// ############################## AUX FUNCTIONS
void callback(char* topic, byte* payload, unsigned int length)
{
  String msg;
  char c = 'n';

  Serial.println("======================================== WeMosLab Update received from MQTT Broker | Begin");
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  Serial.print("Message content: ");
  for (int i = 0; i < length; i++)
  {
    c = (char)payload[i];
    msg += c;
  }
  Serial.println(msg);
  update_WeMosLab (msg);
  Serial.println("======================================== WeMosLab Update received from MQTT Broker | End");

}

// ############################## AUX FUNCTIONS
void update_WeMosLab (String inputmsg)
{
  print_parameters('b');

  StaticJsonBuffer<1024> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(inputmsg);
  if (!root.success())
  {
    Serial.println("parseObject() failed");
  }
  else
  {
    if (inputmsg.indexOf("RESET", 1) >= 0)
      reset_SYS = root["RESET"];
    if (inputmsg.indexOf("REFRELRESET", 1) >= 0)
      reset_REF_relay = root["REFRELRESET"];
    if (inputmsg.indexOf("HUMRELRESET", 1) >= 0)
      reset_HUM_relay = root["HUMRELRESET"];
    if (inputmsg.indexOf("HUMD", 1) >= 0)
      des_hum = root["HUMD"];
    if (inputmsg.indexOf("DHUM", 1) >= 0)
      delta_hum = root["DHUM"];
    if (inputmsg.indexOf("TEMPD", 1) >= 0)
      des_temp = root["TEMPD"];
    if (inputmsg.indexOf("DTEMP", 1) >= 0)
      delta_temp = root["DTEMP"];
    if (inputmsg.indexOf("TEMPCORR", 1) >= 0)
      temp_CORR = root["TEMPCORR"];
    if (inputmsg.indexOf("HUMCORR", 1) >= 0)
      hum_CORR = root["HUMCORR"];
    if (inputmsg.indexOf("WEIGHTINIT", 1) >= 0)
      initial_weight = root["WEIGHTINIT"];
    if (inputmsg.indexOf("WEIGHTCORR", 1) >= 0)
      weight_CORR = root["WEIGHTCORR"];
    if (inputmsg.indexOf("TARE", 1) >= 0)
      tare = root["TARE"];
    
    if (reset_SYS)
    {
      Serial.println("RESET request received - Shooting-down and resetting WeMosLab SW/HW in 9sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMosLab SW/HW in 8sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMosLab SW/HW in 7sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMosLab SW/HW in 6sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMosLab SW/HW in 5sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMosLab SW/HW in 4sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMosLab SW/HW in 3sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMosLab SW/HW in 2sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMosLab SW/HW in 1sec");
      delay(1000);
      ESP.restart(); //Tells the SDK to reboot, so its a more clean reboot, use this one if possible 
    }

    if (reset_REF_relay)
      reset_relay('r');

    if (reset_HUM_relay)
      reset_relay('h');

    if (tare)
      loadcell_tare ();

    upper_hum = des_hum + delta_hum;
    lower_hum = des_hum - delta_hum;
    upper_temp = des_temp + delta_temp;
    lower_temp = des_temp - delta_temp;

    print_parameters('a');
  }
}

// ############################## AUX FUNCTIONS
void print_parameters(char par)
{
  switch (par)
  {
    case 'b':
      Serial.println("======================================== WeMosLab parameters (current) to be updated | Begin");
      Serial.print("Upper limit for relative humidity :\t");
      Serial.print(upper_hum);
      Serial.println(" %");
      Serial.print("Desired relative humidity: \t\t");
      Serial.print(des_hum);
      Serial.println(" %");
      Serial.print("Lower limit for relative humidity :\t");
      Serial.print(lower_hum);
      Serial.println(" %");
      Serial.print("Delta relative humidity: +/- \t\t");
      Serial.print(delta_hum);
      Serial.println(" %");
      Serial.print("Correction to relative humidity: \t");
      Serial.print(hum_CORR);
      Serial.println(" %");

      Serial.print("Upper limit for temperature: \t\t");
      Serial.print(upper_temp);
      Serial.println(" °C");
      Serial.print("Desired temperature: \t\t\t");
      Serial.print(des_temp);
      Serial.println(" °C");
      Serial.print("Lower limit for temperature: \t\t");
      Serial.print(lower_temp);
      Serial.println(" °C");
      Serial.print("Delta temperature: +/- \t\t\t");
      Serial.print(delta_temp);
      Serial.println(" °C");
      Serial.print("Correction to temperature: \t\t");
      Serial.print(temp_CORR);
      Serial.println(" °C");
      Serial.println("======================================== WeMosLab parameters (current) to be updated | End");
    break;
    
    case 'a':
      Serial.println("======================================== WeMosLab parameters (new) successfully updated | Begin");
      Serial.print("Upper limit for relative humidity :\t");
      Serial.print(upper_hum);
      Serial.println(" %");
      Serial.print("Desired relative humidity: \t\t");
      Serial.print(des_hum);
      Serial.println(" %");
      Serial.print("Lower limit for relative humidity :\t");
      Serial.print(lower_hum);
      Serial.println(" %");
      Serial.print("Delta relative humidity: +/- \t\t");
      Serial.print(delta_hum);
      Serial.println(" %");
      Serial.print("Correction to relative humidity: \t");
      Serial.print(hum_CORR);
      Serial.println(" %");

      Serial.print("Upper limit for temperature: \t\t");
      Serial.print(upper_temp);
      Serial.println(" °C");
      Serial.print("Desired temperature: \t\t\t");
      Serial.print(des_temp);
      Serial.println(" °C");
      Serial.print("Lower limit for temperature: \t\t");
      Serial.print(lower_temp);
      Serial.println(" °C");
      Serial.print("Delta temperature: +/- \t\t\t");
      Serial.print(delta_temp);
      Serial.println(" °C");
      Serial.print("Correction to temperature: \t\t");
      Serial.print(temp_CORR);
      Serial.println(" °C");
      Serial.println("======================================== WeMosLab parameters (new) successfully updated | End");
    break;
    
    defualt:
      Serial.println("Something went wrong. Check the program."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

// ############################## AUX FUNCTIONS
void print_measurements(float value, float value_mem, int correction, char par)
{
  switch (par)
  {
    case 'h':
      Serial.println("======================================== WeMosLab Humidity measurements | Begin");
      Serial.println("Humidity (actual)\tHumidity (mem)\t\tCorrection"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value); //Print the current value of the hum_VALUE variable to Serial Comm (USB)
      Serial.print(" % \t\t"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_mem); //Print the corrected value of humidity to Serial Comm (USB)
      Serial.print(" %\t\t\t"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(correction); //Print the corrected value of humidity to Serial Comm (USB)
      Serial.println(" %"); //Print the auxiliar text to Serial Comm (USB)
      Serial.println("======================================== WeMosLab Humidity measurements | End");
    break;
    
    case 't':
      Serial.println("======================================== WeMosLab Temperature measurements | Begin");
      Serial.println("Temperature (actual)\tTemperature (mem)\tCorrection"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value); //Print the current value of the hum_VALUE variable to Serial Comm (USB)
      Serial.print(" °C\t\t"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_mem); //Print the corrected value of humidity to Serial Comm (USB)
      Serial.print(" °C\t\t"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(correction); //Print the corrected value of humidity to Serial Comm (USB)
      Serial.println(" °C"); //Print the auxiliar text to Serial Comm (USB)
      Serial.println("======================================== WeMosLab Temperature measurements | End");
    break;

    case 'w':
      Serial.println("======================================== WeMosLab Weight measurements | Begin");
      Serial.println("Weight (actual)\t\tWeight (mem)\t\tCorrection"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value); //Print the current value of the hum_VALUE variable to Serial Comm (USB)
      Serial.print(" grs\t\t"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_mem); //Print the corrected value of humidity to Serial Comm (USB)
      Serial.print(" grs\t\t"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(correction); //Print the corrected value of humidity to Serial Comm (USB)
      Serial.println(" grs"); //Print the auxiliar text to Serial Comm (USB)
      Serial.println("======================================== WeMosLab Weight measurements | End");
     break;

    defualt:
      Serial.println("Something went wrong. Check the program."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

// ############################## AUX FUNCTIONS
void print_sensor()
{
  Serial.println("======================================== WeMosLab Sensor's status | Begin");
  Serial.println("Sensor Status\tError Flag (mem)\tError Count (+inc)"); //Print the auxiliar text to Serial Comm (USB)

  if  (!ref_sensor_STAT)
    Serial.print("Temp   OK \t"); //Print the auxiliar text to Serial Comm (USB)
  else
    Serial.print("Temp   FAIL \t"); //Print the auxiliar text to Serial Comm (USB)

  if (!ref_sensor_MEM)
    Serial.print("- \t\t\t"); //Print the auxiliar text to Serial Comm (USB)
  else
    Serial.print("Alarm \t\t\t"); //Print the auxiliar text to Serial Comm (USB)
  
  Serial.print(ref_sensor_COUNT); //Print the auxiliar text to Serial Comm (USB)
  Serial.println("\t\t"); //Print the auxiliar text to Serial Comm (USB)

  if  (!hum_sensor_STAT)
    Serial.print("Hum    OK \t"); //Print the auxiliar text to Serial Comm (USB)
  else
    Serial.print("Hum    FAIL \t"); //Print the auxiliar text to Serial Comm (USB)

  if (!hum_sensor_MEM)
    Serial.print("- \t\t\t"); //Print the auxiliar text to Serial Comm (USB)
  else
    Serial.print("Alarm \t\t\t"); //Print the auxiliar text to Serial Comm (USB)
  
  Serial.print(hum_sensor_COUNT); //Print the auxiliar text to Serial Comm (USB)
  Serial.println("\t\t"); //Print the auxiliar text to Serial Comm (USB)

  if  (!weight_sensor_STAT)
    Serial.print("Weight OK \t"); //Print the auxiliar text to Serial Comm (USB)
  else
    Serial.print("Weight FAIL \t"); //Print the auxiliar text to Serial Comm (USB)

  if (!weight_sensor_MEM)
    Serial.print("- \t\t\t"); //Print the auxiliar text to Serial Comm (USB)
  else
    Serial.print("Alarm \t\t\t"); //Print the auxiliar text to Serial Comm (USB)
  
  Serial.println(weight_sensor_COUNT); //Print the auxiliar text to Serial Comm (USB)

  Serial.println("======================================== WeMosLab Sensor's status | End");
}

// ############################## AUX FUNCTIONS
void reset_relay (char par)
{
  switch (par)
  {
    case 'r':
      Serial.println("======================================== WeMosLab Refrigerator's relay reset | Begin");
      if (ref_stat == 1)
      {
        digitalWrite(ref_PIN, LOW); //Write LOW (0V) on the DIGITAL PIN (realy activated) - Refrigerator OFF
        time_ref_on.stop();
        ref_stat = 0;
        delay(10000);
        digitalWrite(ref_PIN, HIGH); //Write HIGH (5V) on the DIGITAL PIN (realy deactivated) - Refrigerator ON
        time_ref_on.resume();
        ref_stat = 1;
      }

      if (!ref_stat == 1)
      {
        digitalWrite(ref_PIN, HIGH); //Write HIGH (5V) on the DIGITAL PIN (realy deactivated) - Refrigerator ON
        time_ref_on.resume();
        ref_stat = 1;
        delay(10000);
        digitalWrite(ref_PIN, LOW); //Write LOW (0V) on the DIGITAL PIN (realy activated) - Refrigerator OFF
        time_ref_on.stop();
        ref_stat = 0;
      }
      Serial.println("======================================== WeMosLab Refrigerator's relay reset | End");
    break;
    
    case 'h':
      Serial.println("======================================== WeMosLab Humidifier's relay reset | Begin");
      if (hum_stat == 0)
      {
        digitalWrite(hum_PIN, LOW); //Write LOW (0V) on the DIGITAL PIN (realy activated) - Humidifier ON
        time_hum_on.resume();
        hum_stat = 1;
        delay(10000);
        digitalWrite(hum_PIN, HIGH); //Write HIGH (5V) on the DIGITAL PIN (realy deactivated) - Humidifier OFF
        time_hum_on.stop();
        hum_stat = 0;
      }

      if (!hum_stat == 0)
      {
        digitalWrite(hum_PIN, HIGH); //Write HIGH (5V) on the DIGITAL PIN (realy deactivated) - Humidifier OFF
        time_hum_on.stop();
        hum_stat = 0;
        delay(10000);
        digitalWrite(hum_PIN, LOW); //Write LOW (0V) on the DIGITAL PIN (realy activated) - Humidifier ON
        time_hum_on.resume();
        hum_stat = 1;
      }
      Serial.println("======================================== WeMosLab Humidifier's relay reset | Begin");
    break;
    
    defualt:
      Serial.println("Something went wrong. Check relay's code."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

// ############################## AUX FUNCTIONS
void loadcell_tare()
{
  LoadCell.tare(); //Tare the scale of LoadCell
}
