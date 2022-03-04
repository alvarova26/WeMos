//Project to Pub/Sub to the MQTT Broker CloudMQTT
//The device WeMos D1 (ESP8266) will report the RSSI and TEMP/HUM measurment every 10 sec from WeMos and DHT21

// Important!!!!!!!!!!!!!!!!!!!!
// Follow the link below to install the right libraries - WeMos doesn't works without those libraries
// https://randomnerdtutorials.com/esp8266-dht11dht22-temperature-and-humidity-web-server-with-arduino-ide/

// Important!!!!!!!!!!!!!!!!!!!!
// MQTT_MAX_PACKET_SIZE defined in the library header MUST BE INCREASED to allow larger payloads in client.publish() method

// Installing the DHT Sensor library
// 1) Click here to download the DHT sensor library. You should have a .zip folder in your Downloads
// 2) Unzip the .zip folder and you should get DHT-sensor-library-master folder
// 3) Rename your folder from DHT-sensor-library-master to DHT
// 4) Move the DHT folder to your Arduino IDE installation libraries folder
// 5) Then re-open your Arduino IDE

// Installing the Adafruit Unified Sensor library
// 1) Click here to download the Adafruit Unified Sensor library. You should have a .zip folder in your Downloads folder
// 2) Unzip the .zip folder and you should get Adafruit_sensor-master folder
// 3) Rename your folder from Adafruit_sensor-master to Adafruit_sensor
// 4) Move the Adafruit_sensor folder to your Arduino IDE installation libraries folder
// 5) Finally, re-open your Arduino IDE

//Note: The humidifier is connected to the relay's NO (normal opened) pins
// - If the relay is deactivated, then the humidifier is turned OFF
// - If the relay is activated, then the humidifier is turned ON
//Note: The refrigerator is connected to the relay's NC (normal closed) pins
// - If the relay is deactivated, then the refrigerator is turned ON
// - If the relay is activated, then the refrigerator is turned OFF

//Realy behavior
//If digitalWrite(ref_PIN, HIGH) => Writes 5V to Relay => Deactivated => Relay's LED off.
//If digitalWrite(ref_PIN, LOW) => Writes 0V to Relay => Activated => Relay's LED on.

// Version notes
//
// IoT_WeMos_CloudMQTT_Hum_Tem_Ctrl_v03 (20190628)
// Included the possibility of resetting the system
// Added values for check the sensors status (example: if some failure occurs in the measurements, it is informed in the mqtt JSON)
//
// IoT_WeMos_CloudMQTT_Hum_Tem_Ctrl_v04 (20190705)
// Included the possibility of change the temp_CORR and hum_CORR parameter from smartphone
// Included the possibility of resetting the refrigerator's relay and the humidifier's relay. New function reset_relay()
// Altered the type of variable from int to bool of ref_stat and hum_stat.
// Sent to mqtt broker the TEMPMEM as temp_VALUE_mem - temp_CORR to consider any correction. Same for HUMMEM
//
// IoT_WeMos_CloudMQTT_Hum_Tem_Ctrl_v05 (20190709)
// Included SYSINIT field on the mqtt published message to inform that the system has just been restarted
// Renamed variable flag to init_mem to avoid confussion
// Variables init_ref and init_hum set to 0 after used in the logic_mem funcion



// ****************************** Include Libraries
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Chrono.h>
#include <DHT.h>


// ****************************** Define PINOUT correspondence for ESP8266
//Pin         Function                                             ESP-8266 Pin
//TX          TXD                                                  TXD
//RX          RXD                                                  RXD
//A0          Analog input, max 3.3V input                         A0
//D0          IO                                                   GPIO16
//D1          IO, SCL                                              GPIO5
//D2          IO, SDA                                              GPIO4
//D3          IO, 10k Pull-up                                      GPIO0
//D4          IO, 10k Pull-up, BUILTIN_LED                         GPIO2
//D5          IO, SCK                                              GPIO14
//D6          IO, MISO                                             GPIO12
//D7          IO, MOSI                                             GPIO13
//D8          IO, 10k Pull-down, SS                                GPIO15
//G           Ground                                               GND
//5V          5V                                                   -
//3V3         3.3V                                                 3.3V
//RST         Reset                                                RST

#define D0    16
#define D1    5
#define D2    4
#define D3    0
#define D4    2
#define D5    14
#define D6    12
#define D7    13
#define D8    15
#define D9    3
#define D10   1


// ****************************** Define DHTPIN and DHTTYPE
#define DHTPIN D2
#define DHTTYPE DHT21


// ****************************** WiFi Netwrok Parameters
const char* ssid = "NET_2G0995C8"; //Change this to your network SSID (name).
const char* pass = "E20995C8"; //Change this your network password

//const char* ssid = "Vanessa"; //Change this to your network SSID (name).
//const char* pass = "vanessa3012"; //Change this your network password

// const char* ssid = "GVT-26D4"; //Change this to your network SSID (name).
// const char* pass = "0162628689"; //Change this your network password

// const char* ssid = "Rede_RF01 2.4"; //Change this to your network SSID (name).
// const char* pass = "redeclaro03"; //Change this your network password

// const char* ssid = "Rede_RF02 5G"; //Change this to your network SSID (name).
// const char* pass = "redeclaro04"; //Change this your network password

//const char* ssid = "Rede_Infra02 2.4"; //Change this to your network SSID (name).
//const char* pass = "redeclaro02"; //Change this your network password


// ****************************** MQTT Broker Parameters
const char* mqttServer = "m16.cloudmqtt.com"; //Server MQTT
const char* mqttUserName = "oepdtofo"; //Username provided by the MQTT Broker
const char* mqttPass = "hcJjL0FbT40U"; //Password provided by the MQTT Broker
const char* mqttClientID = "WeMosClient"; //ClientID provided by the MQTT Broker (MUST BE UNIQUE for each PUB/SUB - i.e. the app on the cell phone must be different)
const char* topicWeMosReadings = "WeMosReadings"; //Topic to send WeMos readings to MQTT Broker / Subs
const char* topicUpdateWeMos = "UpdateWeMos"; //Topic to get Update values to WeMos from MQTT Broker / Pubs
const int mqttPort = 17879; //Port MQTT


// ****************************** Refrigerator Parameters
int ref_PIN = D6; //What pin is the refrigerator's relay command connected to
int temp_CORR = 0; //Correction value for temperature read from the DHT11 sensor
int des_temp = 11; //Desired temperature, wich will define the aim temperature of the system
int delta_temp = 1; //Detla tempertaure allowed, which will establish upper and lower thresholds for relay activation/deactivation
int upper_temp = des_temp + delta_temp; //Upper temperature allowed, once reached, turn ON the refrigerator
int lower_temp = des_temp - delta_temp; //Lower temperature desired, once reached, turn OFF the refrigerator
float temp_VALUE = 0; //Will store the temperature value comming from DHT
float temp_VALUE_mem = 0; //Will store the temperature value as an average of last 5 previous measurements
bool ref_stat = 0; //Refrigerator's status
double ref_UP = 0; //Refrigerator's time UP (working). Will be percentage: ref_UP=(time_ref_on.elapsed()*100)/(time_ref_on.elapsed()+time_ref_on.elapsed()
double ref_DOWN = 0; //Refrigerator's time UP (working). Will be percentage: ref_UP=(time_ref_on.elapsed()*100)/(time_ref_on.elapsed()+time_ref_on.elapsed()
Chrono time_ref_on(Chrono::SECONDS);
float mem_temp[5] = {0, 0, 0, 0, 0}; //Array for average calculations (memory of 5 previous states)
bool init_ref = 1; //Flag for init regrigerator (mem_logic)


// ****************************** Humidifier Parameters
int hum_PIN = D7; //What pin is the humidifier's relay command connected to
int hum_CORR = 0; //Correction value for humidity read from the DHT11 sensor
int des_hum = 80; //Desired humidity, wich will define the aim humidity of the system
int delta_hum = 5; //Detla humidity allowed, which will establish upper and lower thresholds for relay activation/deactivation
int upper_hum = des_hum + delta_hum; //Upper humidity desired, once reached, turn OFF the humidifier
int lower_hum = des_hum - delta_hum; //Lower humidity allowed, once reached, turn ON the humidifier
float hum_VALUE = 0; //Will store the humidity value comming from DHT
float hum_VALUE_mem = 0; //Will store the humidity value as an average of last 5 previous measurements
bool hum_stat = 0; //Humidifier's status
double hum_UP = 0;
double hum_DOWN = 0;
Chrono time_hum_on(Chrono::SECONDS);
float mem_hum[5] = {0, 0, 0, 0, 0}; //Array for average calculations (memory of 5 previous states)
bool init_hum = 1; //Flag for init humidifier (mem_logic)


// ****************************** Miscellaneous Parameters
int correction = 0; //Correction value sent by reference to serial_print auxiliar funcion
float value = 0; //Value of the parameter to be printed sent by reference to serial_print auxiliar funcion
char measure = 'u'; //Code for printing (t=temperarutr or h=humidity) sent to serial_print auxiliar funcion
char par = 'u'; //Code for printing (b=before or a=after) or (r=refrigerator or h=humidifier)
float rssi_VALUE = 0; //Will store the RSSI measuremente form ESP8266
Chrono abs_time(Chrono::SECONDS);
long running_time = 0; //Global time
bool init_mem = 1; //Flag for init memory logic (fill with the first value the memory array)


// ****************************** Sensor's status Parameters
unsigned int ref_error_COUNT = 0; //Counter for sensor's measurements error
unsigned int hum_error_COUNT = 0; //Counter for sensor's measurements error
bool ref_sensor_STAT = 0; //Flag for sesnor status (current error status)
bool hum_sensor_STAT = 0; //Flag for sesnor status (current error status)
bool ref_sensor_MEM = 0; //Flag for sesnor error memory - if measure fails, this flags turns to 1 and remains until reset
bool hum_sensor_MEM = 0; //Flag for sesnor error memory - if measure fails, this flags turns to 1 and remains until reset


// ****************************** System Parameters
bool reset_SYS = 0; //Flag for system reset - if received, calls resetFunc();
bool reset_REF_relay = 0; //Flag for refrigerator's relay reset - if received, calls reset_relay('r');
bool reset_HUM_relay = 0; //Flag for refrigerator's relay reset - if received, calls reset_relay('h');
bool sys_init = 1; // Flag for system initialization - used to report any HW or SW reset => published in WeMosReadings

// ****************************** Initialize Libraies
WiFiClient client; //Initialize the Wifi client library
PubSubClient mqttClient(client); //Initialize the PuBSubClient library
DHT dht(DHTPIN, DHTTYPE); //Initialize DHT library


// ****************************** Functions Prototypes
void setup_serial();
void setup_pinout();
void setup_dht();
void setup_wifi();
void reconnect_wifi();
void setup_mqtt();
void reconnect_mqtt();
void callback(char* topic, byte* payload, unsigned int length);
void update_WeMos (String inputmsg);
void temp_logic();
void hum_logic();
void publish_readings();
void print_measurements(float value, int correction, char measure);
void setup_chronos();
void get_chronos();
void print_chronos();
void print_parameters (char par);
float mem_logic(float value, bool init_mem, char measure);
void print_sensor();
void reset_relay (char par);


// ****************************** Setup main functions, units and variables
void setup()
{
  setup_serial();
  setup_pinout();
  setup_dht();
  setup_wifi();
  setup_mqtt();
  setup_chronos();
}


// ****************************** Loop function - Code runs over and over again
void loop()
{
  if (WiFi.status() != WL_CONNECTED) //Reconnect WiFi if got disconnected
    reconnect_wifi();
  if (!mqttClient.connected()) //Reconnect MQTT client if not connected
    reconnect_mqtt();

  mqttClient.loop(); //Call the loop continuously to establish connection to the server (keepalive)
  temp_logic();
  hum_logic();
  print_sensor();
  get_chronos();
  print_chronos();
  publish_readings();
  delay (10000);
}


// ****************************** AUX FUNCTIONS
void setup_serial()
{
  Serial.begin(115200); //Initialize the serial port (USB Commm)
}


// ****************************** AUX FUNCTIONS
void setup_pinout()
{
  //Initialize DIGITAL PIN as I/O and set an initial level
  pinMode(D2, INPUT); //PIN for DHT Sensor
  pinMode(D5, OUTPUT); //PIN for onboard LED
  pinMode(D6, OUTPUT); //PIN for Humidifyer's relay
  pinMode(D7, OUTPUT); //PIN for Refrigerator's relay
  digitalWrite(D5, LOW); //Write LOW (0V) on the DIGITAL PIN (turn the LED off)
  digitalWrite(D6, HIGH); //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - Humidifier OFF
  hum_stat = 0;
  digitalWrite(D7, HIGH); //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - Refrigerator ON
  ref_stat = 1;
}


// ****************************** AUX FUNCTIONS
void setup_dht()
{
  dht.begin(); //Initialize DHT to send HUM&TEMP readings
}


// ****************************** AUX FUNCTIONS
void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to WiFi Network... ");
  Serial.println(ssid);

  //Initialize WiFi and try to connect the WiFi Network
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Network connected");
  Serial.println("");
}


// ****************************** AUX FUNCTIONS
void reconnect_wifi()
{
  Serial.print("Disconnected from WiFi Network... ");
  delay(10);
  Serial.println();
  Serial.println();
  Serial.print("Trying to reconnect to WiFi Network... ");
  Serial.println(ssid);

  //Initialize WiFi and try to connect the WiFi Network
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Network connected");
  Serial.println("");
}


// ****************************** AUX FUNCTIONS
void setup_mqtt()
{
  //Set the MQTT broker parameters and try to connect the MQTT Broker
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);

  delay(10);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to MTQQ Broker... ");
  Serial.println(mqttServer);

  while (!mqttClient.connected())
  {
    delay(500);
    Serial.print(".");
    mqttClient.connect(mqttClientID, mqttUserName, mqttPass);

    if (!mqttClient.connected())
    {
      // If connection fails, print the ErrorCode in order to know what happened
      // Error Code reference https://pubsubclient.knolleary.net/api.html#state for the failure code explanation
      Serial.println("Connection failed, ErrorCode=");
      Serial.print(mqttClient.state());
      Serial.println("Trying to connect again in 5 seconds...");
      delay(4500);
    }
  }
  Serial.println("");
  Serial.println("MQTT Broker connected");
  Serial.println("");

  mqttClient.publish(topicWeMosReadings, "Online");
  mqttClient.subscribe(topicUpdateWeMos);
}


// ****************************** AUX FUNCTIONS
void reconnect_mqtt()
{
  Serial.print("Disconnected from MQTT Broker... ");
  delay(10);
  Serial.println();
  Serial.println();
  Serial.println("Trying to reconnect to MTQQ Broker... ");
  Serial.println(mqttServer);

  while (!mqttClient.connected())
  {
    delay(500);
    Serial.print(".");
    mqttClient.connect(mqttClientID, mqttUserName, mqttPass);

    if (!mqttClient.connected())
    {
      // If connection fails, print the ErrorCode in order to know what happened
      // Error Code reference https://pubsubclient.knolleary.net/api.html#state for the failure code explanation
      Serial.println("Connection failed, ErrorCode=");
      Serial.println(mqttClient.state());
      Serial.println("Trying to connect again in 5 seconds...");
      delay(500);
    }
  }
  Serial.println("");
  Serial.println("MQTT Broker connected");
  Serial.println("");

  mqttClient.publish(topicWeMosReadings, "Online");
  mqttClient.subscribe(topicUpdateWeMos);
}


// ****************************** AUX FUNCTIONS
void setup_chronos()
{
  abs_time.restart();
  time_ref_on.restart();
  time_hum_on.restart();
  time_hum_on.stop();
}

// ****************************** AUX FUNCTIONS
void temp_logic()
{
  temp_VALUE = dht.readTemperature(); //Read the temperature value form the DHT
  if  (isnan(temp_VALUE))
  {
    ref_error_COUNT = ref_error_COUNT + 1;
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
    print_measurements(temp_VALUE, temp_CORR, 't' ); //Call serial_print () => Print Temperature values
    print_measurements(temp_VALUE_mem, temp_CORR, 't' ); //Call serial_print () => Print Temperature values
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


// ****************************** AUX FUNCTIONS
void hum_logic()
{
  hum_VALUE = dht.readHumidity(); //Read the humidity value form the DHT
  if  (isnan(hum_VALUE))
  {
    hum_error_COUNT = hum_error_COUNT + 1;
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
    print_measurements(hum_VALUE, hum_CORR, 'h' ); //Call serial_print () => Print Humidity values
    print_measurements(hum_VALUE_mem, hum_CORR, 'h' ); //Call serial_print () => Print Humidity values
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


// ****************************** AUX FUNCTIONS
float mem_logic(float value, bool init_mem, char measure)
{
  int i = 0;
  float aux = 0;

  switch (measure)
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

    defualt:
      Serial.println("Something went wrong. Check the program."); //Print the auxiliar text to Serial Comm (USB)
    return -1;
    break;
  }
}


// ****************************** AUX FUNCTIONS
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


// ****************************** AUX FUNCTIONS
void print_chronos()
{
  Serial.println("==================== WeMos Chronos of the System ====================");
  Serial.print("Total running time: ");
  Serial.print(abs_time.elapsed());
  Serial.println(" sec");
  Serial.print("Total refrigerator's time UP: ");
  Serial.print(time_ref_on.elapsed());
  Serial.println(" sec");
  Serial.print("Total humidifier's time UP: ");
  Serial.print(time_hum_on.elapsed());
  Serial.println(" sec");
  Serial.print("Percentage of the total running time that the refrigerator was UP: ");
  Serial.print(String (ref_UP));
  Serial.println(" %");
  Serial.print("Percentage of the total running time that the refrigerator was DOWN: ");
  Serial.print(String (ref_DOWN));
  Serial.println(" %");
  Serial.print("Percentage of the total running time that the humidifier was UP: ");
  Serial.print(String (hum_UP));
  Serial.println(" %");
  Serial.print("Percentage of the total running time that the humidifier was DOWN: ");
  Serial.print(String (hum_DOWN));
  Serial.println(" %");
}


// ****************************** AUX FUNCTIONS
void publish_readings()
{
  StaticJsonBuffer<1024> JSONbufferRX;
  JsonObject& JSONencoder = JSONbufferRX.createObject();

  Serial.println("==================== WeMos Publish sensor's data with JSON over MQTT topic ====================");

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
  // Misc parameters
  JSONencoder["RSSI"] = String (WiFi.RSSI());
  JSONencoder["UPTIME"] = String (running_time / 3600);
  // Sensor parameters
  JSONencoder["REFSENSSTAT"] = String (ref_sensor_STAT);
  JSONencoder["REFSENSMEM"] = String (ref_sensor_MEM);
  JSONencoder["REFERRCOUNT"] = String (ref_error_COUNT);
  JSONencoder["HUMSENSSTAT"] = String (hum_sensor_STAT);
  JSONencoder["HUMSENSMEM"] = String (hum_sensor_MEM);
  JSONencoder["HUMERRCOUNT"] = String (hum_error_COUNT);
  // System reset status
  JSONencoder["SYSINIT"] = String (sys_init);

  char JSONmessageBuffer[1024];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.print("Sending message to MQTT topic...");
  Serial.println(topicWeMosReadings);
  Serial.println(JSONmessageBuffer);

  if (mqttClient.publish(topicWeMosReadings, JSONmessageBuffer) == true)
  {
    Serial.println("==================== WeMos Publish successfully sent ====================");
    sys_init = 0;
  }  
  else
  {
    Serial.println("Error sending message");
  }
}


// ****************************** AUX FUNCTIONS
void callback(char* topic, byte* payload, unsigned int length)
{
  String msg;
  char c = 'n';

  Serial.println("==================== WeMos Update received from MQTT Broker ====================");
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message content: ");
  for (int i = 0; i < length; i++)
  {
    c = (char)payload[i];
    msg += c;
  }
  Serial.println(msg);
  update_WeMos (msg);
}


// ****************************** AUX FUNCTIONS
void update_WeMos (String inputmsg)
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
    
    if (reset_SYS)
    {
      Serial.println("RESET request received - Shooting-down and resetting WeMos SW/HW in 9sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMos SW/HW in 8sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMos SW/HW in 7sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMos SW/HW in 6sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMos SW/HW in 5sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMos SW/HW in 4sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMos SW/HW in 3sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMos SW/HW in 2sec");
      delay(1000);
      Serial.println("RESET request received - Shooting-down and resetting WeMos SW/HW in 1sec");
      delay(1000);
      ESP.restart(); //Tells the SDK to reboot, so its a more clean reboot, use this one if possible 
    }

    if (reset_REF_relay)
      reset_relay('r');

    if (reset_HUM_relay)
      reset_relay('h');

    upper_hum = des_hum + delta_hum;
    lower_hum = des_hum - delta_hum;
    upper_temp = des_temp + delta_temp;
    lower_temp = des_temp - delta_temp;

    print_parameters('a');
  }
}


// ****************************** AUX FUNCTIONS
void print_parameters(char par)
{
  switch (par)
  {
    case 'b':
      Serial.println("==================== WeMos parameters (current) to be updated! ====================");
      Serial.print("Desired relative humidity: ");
      Serial.print(des_hum);
      Serial.println(" %");
      Serial.print("Delta relative humidity: +/- ");
      Serial.print(delta_hum);
      Serial.println(" %");
      Serial.print("Correction to relative humidity: ");
      Serial.print(hum_CORR);
      Serial.println(" %");
      Serial.print("Desired temperature: ");
      Serial.print(des_temp);
      Serial.println(" °C");
      Serial.print("Delta temperature: +/- ");
      Serial.print(delta_temp);
      Serial.println(" °C");
      Serial.print("Correction to temperature: ");
      Serial.print(temp_CORR);
      Serial.println(" °C");
      Serial.print("Upper limit for relative humidity :");
      Serial.print(upper_hum);
      Serial.println(" %");
      Serial.print("Lower limit for relative humidity :");
      Serial.print(lower_hum);
      Serial.println(" %");
      Serial.print("Upper limit for temperature: ");
      Serial.print(upper_temp);
      Serial.println(" °C");
      Serial.print("Lower limit for temperature: ");
      Serial.print(lower_temp);
      Serial.println(" °C");
    break;
    
    case 'a':
      Serial.println("==================== WeMos parameters (new) successfully updated! ====================");
      Serial.print("Desired relative humidity: ");
      Serial.print(des_hum);
      Serial.println(" %");
      Serial.print("Delta relative humidity: +/- ");
      Serial.print(delta_hum);
      Serial.println(" %");
      Serial.print("Correction to relative humidity: ");
      Serial.print(hum_CORR);
      Serial.println(" %");
      Serial.print("Desired temperature: ");
      Serial.print(des_temp);
      Serial.println(" °C");
      Serial.print("Delta temperature: +/- ");
      Serial.print(delta_temp);
      Serial.println(" °C");
      Serial.print("Correction to temperature: ");
      Serial.print(temp_CORR);
      Serial.println(" °C");
      Serial.print("Upper limit for relative humidity :");
      Serial.print(upper_hum);
      Serial.println(" %");
      Serial.print("Lower limit for relative humidity :");
      Serial.print(lower_hum);
      Serial.println(" %");
      Serial.print("Upper limit for temperature: ");
      Serial.print(upper_temp);
      Serial.println(" °C");
      Serial.print("Lower limit for temperature: ");
      Serial.print(lower_temp);
      Serial.println(" °C");
    break;
    
    defualt:
      Serial.println("Something went wrong. Check the program."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}


  // ****************************** AUX FUNCTIONS
void print_measurements(float value, int correction, char measure)
{
  switch (measure)
  {
    case 'h':
      Serial.println("==================== WeMos Humidity measurements ====================");
      Serial.print("Humidity (actual/measured): "); //Print the auxiliar text: "Humidity (actual): " to Serial Comm (USB)
      Serial.print(value); //Print the current value of the hum_VALUE variable to Serial Comm (USB)
      Serial.println(" %"); //Print the auxiliar text: " %" to Serial Comm (USB)
      Serial.print("Humidity (corrected): "); //Print the auxiliar text: "Humidity (actual): " to Serial Comm (USB)
      Serial.print(value - correction); //Print the corrected value of humidity to Serial Comm (USB)
      Serial.println(" %"); //Print the auxiliar text: " %" to Serial Comm (USB)
    break;
    
    case 't':
      Serial.println("==================== WeMos Temperature measurements ====================");
      Serial.print("Temperature (actual/measured): "); //Print the auxiliar text: "Temperature (actual): " to Serial Comm (USB)
      Serial.print(value); //Print the current value of the temp_VALUE variable to Serial Comm (USB)
      Serial.println(" °C"); //Print the auxiliar text: " C" to Serial Comm (USB)
      Serial.print("Temperature (corrected): "); //Print the auxiliar text: "Temperature (actual): " to Serial Comm (USB)
      Serial.print(value - correction); //Print the corrected value of Temperature to Serial Comm (USB)
      Serial.println(" °C"); //Print the auxiliar text: " C" to Serial Comm (USB)
    break;
    
    defualt:
      Serial.println("Something went wrong. Check the program."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

  // ****************************** AUX FUNCTIONS
void print_sensor()
{
  Serial.println("==================== WeMos Sensor's status ====================");
  Serial.print("Current temperature sensor's status (actual/measured): "); //Print the auxiliar text to Serial Comm (USB)
  if  (!ref_sensor_STAT)
    Serial.println("Temperature sensor OK"); //Print the auxiliar text to Serial Comm (USB)
  else
    Serial.println("Temperature sensor FAILED - No measurements comming from sensor!!!!!!!!!!!!!!"); //Print the auxiliar text to Serial Comm (USB)

  Serial.print("Memory temperature sensor's status (history of failures): "); //Print the auxiliar text to Serial Comm (USB)
  if (!ref_sensor_MEM)
    Serial.println("Temperature sensor without failures since last reset"); //Print the auxiliar text to Serial Comm (USB)
  else
    Serial.println("Temperature sensor has FAILED since last reset!!!!!!!!!!!!!!"); //Print the auxiliar text to Serial Comm (USB)
  
  Serial.print("Temperature sensor's error counter since last reset: "); //Print the auxiliar text to Serial Comm (USB)
  Serial.println(ref_error_COUNT); //Print the auxiliar text to Serial Comm (USB)


  Serial.print("Current humidity sensor's status (actual/measured): "); //Print the auxiliar text to Serial Comm (USB)
  if  (!hum_sensor_STAT)
    Serial.println("Humidity sensor OK"); //Print the auxiliar text to Serial Comm (USB)
  else
    Serial.println("Humidity sensor FAILED - No measurements comming from sensor!!!!!!!!!!!!!!"); //Print the auxiliar text to Serial Comm (USB)

  Serial.print("Memory humidity sensor's status (history of failures): "); //Print the auxiliar text to Serial Comm (USB)
  if (!hum_sensor_MEM)
    Serial.println("Humidity sensor without failures since last reset"); //Print the auxiliar text to Serial Comm (USB)
  else
    Serial.println("Humidity sensor has FAILED since last reset!!!!!!!!!!!!!!"); //Print the auxiliar text to Serial Comm (USB)
  
  Serial.print("Humidity sensor's error counter since last reset: "); //Print the auxiliar text to Serial Comm (USB)
  Serial.println(hum_error_COUNT); //Print the auxiliar text to Serial Comm (USB)
}

void reset_relay (char par)
{
  switch (par)
  {
    case 'r':
      Serial.println("==================== WeMos Refrigerator's relay reset ====================");
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

    break;
    
    case 'h':
      Serial.println("==================== WeMos Humidifier's relay reset ====================");
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

    break;
    
    defualt:
      Serial.println("Something went wrong. Check relay's code."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}
