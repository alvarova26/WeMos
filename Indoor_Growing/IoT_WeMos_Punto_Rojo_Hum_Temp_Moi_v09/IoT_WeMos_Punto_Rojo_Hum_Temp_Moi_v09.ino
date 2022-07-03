/* ############################# IoT Project to Pub/Sub measurements via MQTT Broker CloudMQTT ###### */

/* ################################################################################################### */
/* ############################# CHECK Notes v01.h FOR DETAILS => libraries, values, behavior, etc. ## */
/* ################################################################################################### */

/* ############################## Project's Description
The device (WeMos D1 R1 chipset with ESP8266 onborad) will report:
- Temperature (°C)
- Humidity (%)
- Moisture (%)
- PH of Soil (#) - Future Use
- Open/Close Door (On/Off) - Future Use
Along other measurements, every 10 sec via CloudMQTT Broker (using mqtt protocol)
At the same time, WeMos will accept commands form MQTT Broker (Publish/subscribe).
*/

/* ################################################################################################### */
/* ################################################################################################### */
/* ################################################################################################### */
/* ############################################# C O D E ############################################# */
/* ################################################################################################### */
/* ################################################################################################### */
/* ################################################################################################### */

// ############################## Include Libraries
#include <ESP8266WiFi.h>                          //<filename> searchs in the IDE directory
#include <PubSubClient.h>                         //"filename" searchs in the current directory, if not found, IDE directory
#include <ArduinoJson.h>
#include <DHT.h>
#include <Chrono.h>

// ############################## Define PINs equivalences
#define D0    3     //GPIO03                      => Not used in this project
#define D1    1     //GPIO01                      => Not used in this project
#define D2    16    //GPIO16                      => NOT WORKING
#define D3    5     //GPIO05                      => Not used in this project (future use)
#define D4    4     //GPIO04                      => OUTPUT to Relay: LED_A control
#define D5    14    //GPIO14                      => OUTPUT to Relay: LED_B control
#define D6    12    //GPIO12                      => OUTPUT to Relay: LED_C control
#define D7    13    //GPIO13                      => OUTPUT to Relay: FAN_A control
#define D8    0     //GPIO00                      => INPUT from DHT22 sensor (Temperature & Humidity)
#define D9    2     //GPIO02                      => INPUT from PH sensor (future use)
#define D10   15    //GPIO15                      => INPUT from DOOR sensor (future use)
#define A0    A0    //ANALOG                      => INPUT from MOISTURE sensor (Moisture of Soil)

// ############################## WiFi Netwrok Parameters
const char* ssid = "VIVOFIBRA-B178";              //Change this to your network SSID (name).
const char* pass = "D4A4D07B83";                  //Change this your network password

// ############################## MQTT Broker Parameters
const char* mqttServer = "m16.cloudmqtt.com";     //Server MQTT
const char* mqttUserName = "oepdtofo";            //Username provided by the MQTT Broker
const char* mqttPass = "hcJjL0FbT40U";            //Password provided by the MQTT Broker
const char* mqttClientID = "WeMosClient";         //ClientID provided by the MQTT Broker (MUST BE UNIQUE for each PUB/SUB - i.e. the app on the cell phone must be different)
const char* topicWeMosReadings = "WeMosReadings"; //Topic to send WeMos readings to MQTT Broker / Subs
const char* topicUpdateWeMos = "UpdateWeMos";     //Topic to get Update values to WeMos from MQTT Broker / Pubs
const int mqttPort = 17879;                       //Port MQTT
unsigned int msglength = 0;                       //Length of the incoming MQTT msg

// ############################## DHT22 Parameters
#define DHTPIN D8                                 //PIN connected to the DHT222 sensor
#define DHTTYPE DHT22                             //Type of DHT's family (AM2302)

// ############################## MOI Parameters
const int MOISPIN = A0;                           //PIN connected to the Moisture sensor
const int AirValue = 680;                         //Value with the sensor measuring just air
const int WaterValue = 310;                       //Value with the sensor measuring just water

// ############################## RELAYs Parameters
int LED_A_PIN = D4;                               //PIN connected to Relay to command LED_A (central LED)
int LED_B_PIN = D5;                               //PIN connected to Relay to command LED_B (LEDs \)
int LED_C_PIN = D6;                               //PIN connected to Relay to command LED_C (LEDs /)
int FAN_A_PIN = D7;                               //PIN connected to Relay to command FAN_A (air flow)

// ############################## LEDs Parameters
bool LED_A_stat = 0;                              //LED_A's relay status (0=OFF | 1=ON - LEDs ON)
bool LED_B_stat = 0;                              //LED_B's relay status (0=OFF | 1=ON - LEDs ON)
bool LED_C_stat = 0;                              //LED_C's relay status (0=OFF | 1=ON - LEDs ON)

// ############################## FANs Parameters
bool FAN_A_stat = 0;                              //FAN_A's relay status (0=OFF | 1=ON - FAN ON)

// ############################## TEM Parameters
float tem_raw = 0;                                //Will store the temperature value comming from sensor
float tem_mem = 0;                                //Will store the memorized temperature value for averaging
float tem_vec_mem[5] = {0, 0, 0, 0, 0};           //Array for average calculations (memory of 5 previous values)
int tem_cor = 0;                                  //Will store the correction temperature value if needed
int tem_des = 27;                                 //Desired temperature, wich will define the aim temperature of the system
int tem_del = 1;                                  //Detla tempertaure allowed, which will establish upper and lower thresholds
int tem_upl = tem_des + tem_del;                  //Upper temperature limit
int tem_lol = tem_des - tem_del;                  //Lower temperature limit
bool tem_ini_mem = 1;                             //Flag for init memory logic (fill with the first value the memory array)
bool tem_ala_sta = 0;                             //Flag for current temperature alarm status => Active if upper or lower treshold broken
bool tem_ala_mem = 0;                             //Flag for temperature alarm status => Active if upper or lower treshold has been broken
unsigned int tem_ala_cou = 0;                     //Counter for temperature alarm treshold broken => Increases every time it was broken

// ############################## HUM Parameters
float hum_raw = 0;                                //Will store the humidity value comming from sensor
float hum_mem = 0;                                //Will store the memorized humidity value for averaging
float hum_vec_mem[5] = {0, 0, 0, 0, 0};           //Array for average calculations (memory of 5 previous values)
int hum_cor = 0;                                  //Will store the correction humidity value if needed
int hum_des = 75;                                 //Desired humidity, wich will define the aim humidity of the system
int hum_del = 3;                                  //Detla humidity allowed, which will establish upper and lower thresholds
int hum_upl = hum_des + hum_del;                  //Upper humidity limit
int hum_lol = hum_des - hum_del;                  //Lower humidity limit
bool hum_ini_mem = 1;                             //Flag for init memory logic (fill with the first value the memory array)
bool hum_ala_sta = 0;                             //Flag for current humidity alarm status => Active if upper or lower treshold broken
bool hum_ala_mem = 0;                             //Flag for humidity alarm status => Active if upper or lower treshold has been broken
unsigned int hum_ala_cou = 0;                     //Counter for humidity alarm treshold broken => Increases every time it was broken

// ############################## MOI Parameters
float moi_unr = 0;                                //Will store the raw moisture value comming from sensor
float moi_raw = 0;                                //Will store the final value of moisture (%) after formula mapping
float moi_mem = 0;                                //Will store the memorized moisture value for averaging
float moi_vec_mem[5] = {0, 0, 0, 0, 0};           //Array for average calculations (memory of 5 previous values)
int moi_cor = 0;                                  //Will store the correction moisture value if needed
int moi_des = 70;                                 //Desired moisture, wich will define the aim moisture of the system
int moi_del = 3;                                  //Detla moisture allowed, which will establish upper and lower thresholds
int moi_upl = moi_des + moi_del;                  //Upper moisture limit
int moi_lol = moi_des - moi_del;                  //Lower moisture limit
bool moi_ini_mem = 1;                             //Flag for init memory logic (fill with the first value the memory array)
bool moi_ala_sta = 0;                             //Flag for current moisture alarm status => Active if upper or lower treshold broken
bool moi_ala_mem = 0;                             //Flag for moisture alarm status => Active if upper or lower treshold has been broken
unsigned int moi_ala_cou = 0;                     //Counter for moisture alarm treshold broken => Increases every time it was broken

// ############################## Sensor's status Parameters
unsigned int tem_sen_cou = 0;                     //Counter for sensor's measurements error
unsigned int hum_sen_cou = 0;                     //Counter for sensor's measurements error
unsigned int moi_sen_cou = 0;                     //Counter for sensor's measurements error
bool tem_sen_sta = 0;                             //Flag for sesnor status (current error status)
bool hum_sen_sta = 0;                             //Flag for sesnor status (current error status)
bool moi_sen_sta = 0;                             //Flag for sesnor status (current error status)
bool tem_sen_mem = 0;                             //Flag for sesnor error memory - if measure fails, this flags turns to 1 and remains until reset
bool hum_sen_mem = 0;                             //Flag for sesnor error memory - if measure fails, this flags turns to 1 and remains until reset
bool moi_sen_mem = 0;                             //Flag for sesnor error memory - if measure fails, this flags turns to 1 and remains until reset

// ############################## System Parameters
bool reset_SYS = 0;                               //Flag for system reset - if received, calls reset_sys();
bool reset_LED_A_relay = 0;                       //Flag for LED_A's relay reset - if received, calls reset_relay('a');
bool reset_LED_B_relay = 0;                       //Flag for LED_B's relay reset - if received, calls reset_relay('b');
bool reset_LED_C_relay = 0;                       //Flag for LED_C's relay reset - if received, calls reset_relay('c');
bool reset_FAN_A_relay = 0;                       //Flag for FAN_A's relay reset - if received, calls reset_relay('f');
bool init_param = 1;                              //Flag for system initialization - change value after initialization status and varibles, or restar
bool clear_ALM = 0;                               //Flag for clear alarms received
bool clear_SEM = 0;                               //Flag for clear failures received

// ############################## Illumunation & AirFlow Parameters
bool rot_ill_stat = 0;                            //Flag for entering in the rotating illumination first time
bool onoff_ill_stat = 0;                          //Flag for entering in the on/off illumination first time
int airflow_sett = 0;                             //Code for follow up the status of the air flow - if received, calls set_airflow();
int illumin_sett = 0;                             //Code for follow up the setting of the illumination - if received, calls set_illumination();
int rot_ill_mode = 0;                             //Code for the rotating illumination mode (0 = OFF | 1 = ON)
int rot_ill_count = 0;                            //Counter for follow up the status of the rotating illumination
int rot_ill_per = 1;                              //Code for period of time for rotating illumination (1 = 1min | 2 = 5min | and so on)
int rot_ill_cyc = 6;                              //Period of time for rotating illumination (1 cycle = 10 seconds => 6 = 1min | 30 = 5min | and so on)
int onoff_ill_mode = 0;                           //Code for the on/off illumination mode (0 = OFF | 1 = 4hs ON - 4hs OFF | and so on...)
int onoff_ill_count = 0;                          //Counter for follow up the status of the on/off illumination
int onoff_ill_cyc = 0;                            //Period of time for on/off illumination (1 cycle = 10 seconds)
char illc = 'u';                                  //Illumination code (see function for details) sent/received via MQTT to choose the level of illumination in the setup
char airc = 'u';                                  //Air Flow code (0=OFF | 1=ON) sent/received via MQTT to turn ON/OFF the FANs
char rotc = 'u';                                  //Code for rotating illumination settings
char onoffc = 'u';                                //Code for on/off illumination settings
int prev_illum = 0;                               //Variable to store the previous status of illumination
int prev_airfl = 0;                               //Variable to store the previous status of ariflow
bool global_off = 0;                              //Flag for preventing counters to add up during the 'off' time (onoff_ill_mode)

// ############################## Miscellaneous Parameters
float rssi_val_raw = 0;                           //Will store the RSSI measuremente form ESP8266
float value_mea = 0;                              //Value of the measurement to be printed. Sent by reference to print_measurements() auxiliar function
float value_mem = 0;                              //Averaged value of the measurement to be printed. Sent by reference to print_measurements() auxiliar function
int value_cor = 0;                                //Correction value of the measurement to be printed. Sent by reference to print_measurements() auxiliar function
char code = 'u';                                  //Printing code (t=temperarutr | h=humidity | m=moisture | a=after | b=before) sent to auxiliar functions
char clrc = 'u';                                  //Clear code (t=temperarutre | h=humidity | m=moisture) sent to auxiliar functions clear_mem()
bool mea_ini = 1;                                 //Flag for init memory logic (fill with the first value the memory array) sent to mem_logic() auxiliar function
unsigned long chrono_time = 0;                    //Will sotre the global running time (in seconds)
Chrono abs_time(Chrono::SECONDS);                 //Define the Chrono object  abs_time to keep the time

// ############################## Time_keeper Parameters
unsigned long secsNow = 0;
unsigned long secsLast = 0;
int seconds = 0;
int minutes = 0;
int hours = 0;
int days = 0;

// ############################## Initialize Libraies
WiFiClient client;                                //Initialize the Wifi client library
PubSubClient mqttClient(client);                  //Initialize the PuBSubClient library
DHT dht(DHTPIN, DHTTYPE);                         //Initialize DHT sensor

// ############################## Functions Prototypes
void setup_serial();                              //FUNCTION OK
void setup_pinout();                              //FUNCTION OK
void setup_sensors();                             //FUNCTION OK
void initial_setup();                             //FUNCTION OK
void setup_wifi();                                //FUNCTION OK
void setup_mqtt();                                //FUNCTION OK
void check_wifi();                                //FUNCTION OK
void reconnect_wifi();                            //FUNCTION OK
void check_mqtt();                                //FUNCTION OK
void reconnect_mqtt();                            //FUNCTION OK
void callback(char* topic, byte* payload, unsigned int msglength);//FUNCTION OK
void update_WeMos(String inputmsg);               //FUNCTION OK
void tem_logic();                                 //FUNCTION OK
void hum_logic();                                 //FUNCTION OK
void moi_logic();                                 //FUNCTION OK
void set_illumination(char illc);                 //FUNCTION OK
void set_rotation(char rotc);                     //FUNCTION OK
void set_onoff(char onoffc);                      //FUNCTION OK
void set_airflow(char airc);                      //FUNCTION OK
void clear_mem(char clrc);                        //FUNCTION OK
void check_illum();                               //working on it
void check_rot();                                 //working on it
void check_onoff();                               //working on it
void publish_readings();                          //FUNCTION OK
void print_measurements(float value_mea, float value_mem, int value_cor, char code);//FUNCTION OK
void print_parameters(char code);                 //FUNCTION OK                 
float mem_logic(float value_mea, bool init_mem, char code); //FUNCTION OK
void reset_relay(char code);                      //FUNCTION OK                     
void reset_sys();                                 //FUNCTION OK                     
void setup_chrono();                              //FUNCTION OK
void get_chrono();                                //FUNCTION OK
void print_chrono();                              //FUNCTION OK
void time_keeper();                               //FUNCTION OK

/* ################################################################################################### */
/* ################################################################################################### */
/* ############################## SETUP FUNCTION - Setup main functions, units and variables ######### */
/* ################################################################################################### */
/* ################################################################################################### */
void setup()
{
  setup_serial();                                 //Calls setup_serial() function to initialize the serial port
  setup_pinout();                                 //Calls pinout() funciton to set the PINs mode (INPUT, OUTPUT, test, etc.)
  setup_sensors();                                //Calls setup_sensor() to initialize the sensors and print basic readings
  setup_wifi();                                   //Calls setup_wifi() to initialize and setup the WiFi
  setup_mqtt();                                   //Calls setup_mqtt() to initialize and setup the MQTT connection
  initial_setup();                                //Calls initial_setup() to initialize status and variables
  setup_chrono();                                 //Calls setup_chrono() to initialize the timers
}

/* ################################################################################################### */
/* ################################################################################################### */
/* ############################## MAIN FUNCTION - Forever LOOP ####################################### */
/* ################################################################################################### */
/* ################################################################################################### */
void loop()
{
  check_wifi();                                   //Continously check the WiFi status (reconnect if needed)
  check_mqtt();                                   //Continously check the mqtt status (reconnect if needed)
  mqtt_loop();                                    //MQTT keep alive
  tem_logic();                                    //Calls the function to read Temperature from DTH22
  hum_logic();                                    //Calls the function to read Humidity from DTH22
  moi_logic();                                    //Calls the function to read moisture from MOI
  check_illum();                                  //Calls the function to verify the illumination status
  check_rot();                                    //Calls the function to verify the rotation status
  check_onoff();                                  //Calls the function to verify the on/off status
  get_chrono();                                   //Calls the funciton to read the time has passed
  print_chrono();                                 //Prints just for control (line will be deleted)
  time_keeper();                                  //Calls  the function to verify the time has passed
  publish_readings();                             //Publish the WeMos readings to the topic
  delay (10000);                                  //Delay 10 sec
}

/* ################################################################################################### */
/* ################################################################################################### */
/* ############################## AUX FUNCTIONS - Definition of all functions of the code ############ */
/* ################################################################################################### */
/* ################################################################################################### */

// ############################## AUX FUNCTIONS
void setup_serial()
{
  Serial.begin(115200);                           //Initialize the serial port (USB Comm)
  Serial.println();
  Serial.println("Initializing serial port...");
  delay (500);
  Serial.println("Done!");
}

// ############################## AUX FUNCTIONS
void setup_pinout()
{
  Serial.println("Initializing PINOUT Settings...");
  delay (500);
  pinMode(D4, OUTPUT);                            //PIN GPIO04 => LED_A's relay
  pinMode(D5, OUTPUT);                            //PIN GPIO14 => LED_B's relay
  pinMode(D6, OUTPUT);                            //PIN GPIO12 => LED_C's relay
  pinMode(D7, OUTPUT);                            //PIN GPIO13 => FAN_A's relay
  pinMode(D8, INPUT);                             //PIN GPIO00 => DHT22's input PIN
  pinMode(A0, INPUT);                             //PIN ANALOG => MOISTURE's input PIN
  Serial.println("PINOUT Settings Done!");

  Serial.println("Deactivate all relays (no energy condition)!");
  digitalWrite(D4, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
  LED_A_stat = 0;                                 //Update status variable
  digitalWrite(D5, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
  LED_B_stat = 0;                                 //Update status variable
  digitalWrite(D6, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C OFF
  LED_C_stat = 0;                                 //Update status variable
  digitalWrite(D7, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_A OFF
  FAN_A_stat = 0;                                 //Update status variable

/* Disable the one-by-one relay's test to avoid waste time and possible HW failures
  Serial.println("Initializing Relay_1's Quick Test...");
  digitalWrite(D4, LOW);                          //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_A ON
  LED_A_stat = 1;                                 //Update status variable
  delay (5000);                                   //Delay code execution
  digitalWrite(D4, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
  LED_A_stat = 0;                                 //Update status variable
  Serial.println("Relay's Quick Test Done!");

  Serial.println("Initializing Relay_2's Quick Test...");
  digitalWrite(D5, LOW);                          //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_B ON
  LED_B_stat = 1;                                 //Update status variable
  delay (5000);                                   //Delay code execution
  digitalWrite(D5, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
  LED_B_stat = 0;                                 //Update status variable
  Serial.println("Relay's Quick Test Done!");

  Serial.println("Initializing Relay_3's Quick Test...");
  digitalWrite(D6, LOW);                          //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_C ON
  LED_C_stat = 1;                                 //Update status variable
  delay (5000);                                   //Delay code execution
  digitalWrite(D6, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C OFF
  LED_C_stat = 0;                                 //Update status variable
  Serial.println("Relay's Quick Test Done!");

  Serial.println("Initializing Relay_4's Quick Test...");
  digitalWrite(D7, LOW);                          //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_A ON
  FAN_A_stat = 1;                                 //Update status variable
  delay (5000);                                   //Delay code execution
  digitalWrite(D7, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_A OFF
  FAN_A_stat = 0;                                 //Update status variable
  Serial.println("Relay's Quick Test Done!");
*/
}

// ############################## AUX FUNCTIONS
void setup_sensors()
{
  Serial.println("Initializing Temperature Sensor...");
  delay (500);
  dht.begin();                                    //Initialize DHT22 sensor and send HUM & TEMP readings to serial port
  tem_raw = dht.readTemperature();                //Read the temperature value form the DHT22
  Serial.println("Temperature: ");
  Serial.println(tem_raw);                        //Print it to serial port
  Serial.println("°C");
  Serial.println("Temperature Sensor Done!");

  delay (500);
  Serial.println("Initializing Humidity Sensor...");
  hum_raw = dht.readHumidity();                   //Read the humidity value form the DHT22
  Serial.println("Humidity: ");
  Serial.println(hum_raw);                        //Print it to serial port
  Serial.println("%");
  Serial.println("Humidity Sensor Done!");

  Serial.println("Initializing Moisture Sensor...");
  delay (500);
  moi_unr = analogRead(A0);                       //Read the raw value of moisture of the soil 
  moi_raw = map(moi_unr, AirValue, WaterValue, 0, 100); //Map the raw value to percentage of moisture in the soil
  Serial.println("Moisture: ");
  Serial.println(moi_raw);
  Serial.println("%");
  Serial.println("Moisture Sensor Done!");
}

// ############################## AUX FUNCTIONS
void setup_wifi()
{
  Serial.print("Connecting to WiFi Network: ");
  Serial.println(ssid);

  delay (500);
  WiFi.mode(WIFI_STA);                            //Explicitly set the ESP8266 to be a WiFi-client (avoid network conflict)
  WiFi.begin(ssid, pass);                         //Initialize the WiFi
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println(".");
  }
  Serial.println();
  Serial.println("WiFi Network connected!");
}

// ############################## AUX FUNCTIONS
void check_wifi()
{
  if (WiFi.status() != WL_CONNECTED)              //Reconnect WiFi if got disconnected
    reconnect_wifi();
}

// ############################## AUX FUNCTIONS
void reconnect_wifi()
{
  Serial.println("Disconnected from WiFi Network!");
  Serial.print("Trying to reconnect to WiFi Network ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);                            //Explicitly set the ESP8266 to be a WiFi-client (avoid network conflict)
  WiFi.begin(ssid, pass);                         //Initialize again the WiFi
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
  mqttClient.setServer(mqttServer, mqttPort);     //Set the MQTT broker parameters and try to connect the MQTT Broker
  mqttClient.setCallback(callback);               //Define the callback function (will be activated when event arrives)

  Serial.print("Connecting to MTQQ Broker: ");
  Serial.println(mqttServer);

  delay (500);
  while (!mqttClient.connected())
  {
    Serial.println(".");
    mqttClient.connect(mqttClientID, mqttUserName, mqttPass);

    if (!mqttClient.connected())
    {
      /* If connection fails, print the ErrorCode in order to know what happened
      Error Code reference https://pubsubclient.knolleary.net/api.html#state for the failure code explanation */
      Serial.println("Connection failed, ErrorCode=");
      Serial.print(mqttClient.state());
      Serial.println("Trying to connect again in 5 seconds...");
      delay(5000);
    }
  }
  Serial.println();
  Serial.println("MQTT Broker connected!");

  mqttClient.publish(topicWeMosReadings, "Online"); //Publish to the topic the word "Online" for let know the established connection
  mqttClient.subscribe(topicUpdateWeMos);         //Subscribe to the topic to be 'listening' for any event published
}

// ############################## AUX FUNCTIONS
void check_mqtt()
{
  if (!mqttClient.connected())                    //Reconnect MQTT client if not connected
    reconnect_mqtt();
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
      /* If connection fails, print the ErrorCode in order to know what happened
      Error Code reference https://pubsubclient.knolleary.net/api.html#state for the failure code explanation */
      Serial.println("Connection failed, ErrorCode=");
      Serial.print(mqttClient.state());
      Serial.println("Trying to connect again in 5 seconds...");
      delay(5000);
    }
  }
  Serial.println();
  Serial.println("MQTT Broker connected again!");

  mqttClient.publish(topicWeMosReadings, "Online again");
  mqttClient.subscribe(topicUpdateWeMos);
}

// ############################## AUX FUNCTIONS
void initial_setup()
{
  if (init_param)
  {
    Serial.println("======================================== WeMos Initial Illumination Setup to LOW | Begin");
    set_illumination('a');                      //Set the code for initial illumination (VLOW - only central LED ON)
    set_rotation('3');                          //Set the code for initial rotating illumination (3 = 10 min rotating period)
    set_onoff('0');                             //Set the code for initial on/off illumination (0 = OFF)
    Serial.println("======================================== WeMos Initial Illumination Setup to LOW | End");

    Serial.println("======================================== WeMos Initial Air Flow Set to ON | Begin");
    set_airflow('o');                           //Set the code for initial air flow
    Serial.println("======================================== WeMos Initial Air Flow Set to ON | End");
    init_param = 0;
  }
}

// ############################## AUX FUNCTIONS
void mqtt_loop()
{
  mqttClient.loop();                              //Call the loop continuously to establish connection to the server (keepalive)
}

// ############################## AUX FUNCTIONS
void publish_readings()
{
  StaticJsonDocument<1024> JSONBuffer;            //Create the JSON Document (will be the buffer of data)
  JsonObject JSONDocTX = JSONBuffer.to<JsonObject>();//Create the JSON Object that will be TX

  Serial.println("======================================== WeMos Publish sensor's data (JSON) over MQTT topic | Begin");

  // Publish Temperature Values
  JSONDocTX["TEMRAW"] = String (tem_raw);
  JSONDocTX["TEMCOR"] = String (tem_cor);
  JSONDocTX["TEMMEM"] = String (tem_mem - tem_cor);
  JSONDocTX["TEMDES"] = String (tem_des);
  JSONDocTX["TEMDEL"] = String (tem_del);
  JSONDocTX["TEMUPL"] = String (tem_upl);
  JSONDocTX["TEMLOL"] = String (tem_lol);
  JSONDocTX["TEMALS"] = String (tem_ala_sta);
  JSONDocTX["TEMALM"] = String (tem_ala_mem);
  JSONDocTX["TEMALC"] = String (tem_ala_cou);

  // Publish Humidity Values
  JSONDocTX["HUMRAW"] = String (hum_raw);
  JSONDocTX["HUMCOR"] = String (hum_cor);
  JSONDocTX["HUMMEM"] = String (hum_mem - hum_cor);
  JSONDocTX["HUMDES"] = String (hum_des);
  JSONDocTX["HUMDEL"] = String (hum_del);
  JSONDocTX["HUMUPL"] = String (hum_upl);
  JSONDocTX["HUMLOL"] = String (hum_lol);
  JSONDocTX["HUMALS"] = String (hum_ala_sta);
  JSONDocTX["HUMALM"] = String (hum_ala_mem);
  JSONDocTX["HUMALC"] = String (hum_ala_cou);

  // Publish Moisture Values
  JSONDocTX["MOIUNR"] = String (moi_unr);
  JSONDocTX["MOIRAW"] = String (moi_raw);
  JSONDocTX["MOICOR"] = String (moi_cor);
  JSONDocTX["MOIMEM"] = String (moi_mem - moi_cor);
  JSONDocTX["MOIDES"] = String (moi_des);
  JSONDocTX["MOIDEL"] = String (moi_del);
  JSONDocTX["MOIUPL"] = String (moi_upl);
  JSONDocTX["MOILOL"] = String (moi_lol);
  JSONDocTX["MOIALS"] = String (moi_ala_sta);
  JSONDocTX["MOIALM"] = String (moi_ala_mem);
  JSONDocTX["MOIALC"] = String (moi_ala_cou);

  // Publish Illumination Status
  JSONDocTX["ILLSET"] = String (illumin_sett);      //Illumination setting (how many LEDs ON/OFF)
  JSONDocTX["ALILLP"] = String (rot_ill_per);       //Rotating period
  JSONDocTX["ALILLY"] = String (rot_ill_cyc);       //Rotating cycle - used to set up the amount of time
  JSONDocTX["ALILLC"] = String (rot_ill_count);     //Rotating counter - used to follow up how many cycles have been passed
  JSONDocTX["ALTMOD"] = String (rot_ill_mode);      //Rotating mode
  JSONDocTX["ONOFFM"] = String (onoff_ill_mode);    //ON/OFF mode (0 = OFF | 1 = 4hs ON - 4hs OFF | 2 = 8hs ON - 8hs OFF...and so on)
  JSONDocTX["ONOFFY"] = String (onoff_ill_cyc);     //ON/OFF mode (0 = OFF | 1 = 4hs ON - 4hs OFF | 2 = 8hs ON - 8hs OFF...and so on)
  JSONDocTX["ONOFFC"] = String (onoff_ill_count);   //ON/OFF mode (0 = OFF | 1 = 4hs ON - 4hs OFF | 2 = 8hs ON - 8hs OFF...and so on)

  // Publish Air Flow Status
  JSONDocTX["AIRSTA"] = String (airflow_sett);

  // Publish Relay Status
  JSONDocTX["LEDAST"] = String (LED_A_stat);
  JSONDocTX["LEDBST"] = String (LED_B_stat);
  JSONDocTX["LEDCST"] = String (LED_C_stat);
  JSONDocTX["FANAST"] = String (FAN_A_stat);

  // Publish Sensor Status
  JSONDocTX["TEMSES"] = String (tem_sen_sta);
  JSONDocTX["TEMSEM"] = String (tem_sen_mem);
  JSONDocTX["TEMSEC"] = String (tem_sen_cou);
  JSONDocTX["HUMSES"] = String (hum_sen_sta);
  JSONDocTX["HUMSEM"] = String (hum_sen_mem);
  JSONDocTX["HUMSEC"] = String (hum_sen_cou);
  JSONDocTX["MOISES"] = String (moi_sen_sta);
  JSONDocTX["MOISEM"] = String (moi_sen_mem);
  JSONDocTX["MOISEC"] = String (moi_sen_cou);

  // Publish Miscelaneous Values
  JSONDocTX["RSSIWI"] = String (WiFi.RSSI());
  JSONDocTX["RUNCHR"] = String (chrono_time) + String ("sec");
  JSONDocTX["RUNTIM"] = String (days) + String("d ") + String (hours) + String(":") + String (minutes) + String(":") + String (seconds);
  JSONDocTX["SW_VER"] = String ("v09 - 20220703");

  char JSONBufferTX[1024];                        //Define a buffer to Tx the data
  serializeJson(JSONDocTX, JSONBufferTX, sizeof(JSONBufferTX));//Serialize the JSONDocTX into JSONBufferTX

  Serial.print("Sending message to MQTT topic...");
  Serial.println(topicWeMosReadings);             //Print the Topic just for control
  Serial.println(JSONBufferTX);                   //Print the buffer that will be Tx just for control

  if (mqttClient.publish(topicWeMosReadings, JSONBufferTX) == true)//Publish the buffer to the topic
    Serial.println("======================================== WeMos Publish sensor's data (JSON) over MQTT topic | End");
  else
    Serial.println("Error sending message!");
}

// ############################## AUX FUNCTIONS
void callback(char* topic, byte* payload, unsigned int msglength)
{
  String msg;
  char c = 'n';

  Serial.println("======================================== WeMos Update received from MQTT Broker | Begin");
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  Serial.print("Message content: ");
  for (int i = 0; i < msglength; i++)
  {
    c = (char)payload[i];
    msg += c;
  }
  Serial.println(msg);
  update_WeMos(msg);
  Serial.println("======================================== WeMos Update received from MQTT Broker | End");
}

// ############################## AUX FUNCTIONS
void update_WeMos(String inputmsg)
{
  print_parameters('b');
  
  StaticJsonDocument<1024> JSONBufferRX;          //Create the JSON Document (will be the buffer of data)
  DeserializationError error = deserializeJson(JSONBufferRX, inputmsg);//Deserialize the input message into JSONBufferRX

  if (error)
  {
    Serial.print(F("DeserializeJson() failed with code "));
    Serial.println(error.c_str());
    return;
  }
  else
  {
    // SYS & RELAY RESET RECEIVED
    if (inputmsg.indexOf("SYS_RESET", 1) >= 0)      //Check if there is the desired string in the input message
    {
      reset_SYS = JSONBufferRX["SYS_RESET"];        //Extract from JSONBufferRX the value fo the field especified
      if (reset_SYS)
      {
        reset_sys();                                //Call the function reset_sys() for SW/HW reinitialization
      }
    }
    if (inputmsg.indexOf("LDA_RESET", 1) >= 0)
    {
      reset_LED_A_relay = JSONBufferRX["LDA_RESET"];
      if (reset_LED_A_relay)
      {
        reset_relay('a');                           //Call the function reset_relay() to reset the relay passed as argument
      }
    }
    if (inputmsg.indexOf("LDB_RESET", 1) >= 0)
    {
      reset_LED_B_relay = JSONBufferRX["LDB_RESET"];
      if (reset_LED_B_relay)
      {
        reset_relay('b');                           //Call the function reset_relay() to reset the relay passed as argument
      }
    }
    if (inputmsg.indexOf("LDC_RESET", 1) >= 0)
    {
      reset_LED_C_relay = JSONBufferRX["LDC_RESET"];
      if (reset_LED_C_relay)
      {
        reset_relay('c');                           //Call the function reset_relay() to reset the relay passed as argument
      }
    }
    if (inputmsg.indexOf("FNA_RESET", 1) >= 0)
    {
      reset_FAN_A_relay = JSONBufferRX["FNA_RESET"];
      if (reset_FAN_A_relay)
      {
        reset_relay('f');                           //Call the function reset_relay() to reset the relay passed as argument
      }
    }
    
    // CLEAR ALARMS & FAILURES CODE RECEIVED
    if (inputmsg.indexOf("TEMALR", 1) >= 0)
    {
      clear_ALM = JSONBufferRX["TEMALR"];
      Serial.println(clear_ALM);
      if (clear_ALM)
      {
        clear_mem('t');                           //Call the function clear_mem() to reset the memory of the code passed as argument
      }
    }
    if (inputmsg.indexOf("HUMALR", 1) >= 0)
    {
      clear_ALM = JSONBufferRX["HUMALR"];
      if (clear_ALM)
      {
        clear_mem('h');                           //Call the function clear_mem() to reset the memory of the code passed as argument
      }
    }
    if (inputmsg.indexOf("MOIALR", 1) >= 0)
    {
      clear_ALM = JSONBufferRX["MOIALR"];
      if (clear_ALM)
      {
        clear_mem('m');                           //Call the function clear_mem() to reset the memory of the code passed as argument
      }
    }
    if (inputmsg.indexOf("TEMSER", 1) >= 0)
    {
      clear_ALM = JSONBufferRX["TEMSER"];
      if (clear_ALM)
      {
        clear_mem('a');                           //Call the function clear_mem() to reset the memory of the code passed as argument
      }
    }
    if (inputmsg.indexOf("HUMSER", 1) >= 0)
    {
      clear_ALM = JSONBufferRX["HUMSER"];
      if (clear_ALM)
      {
        clear_mem('b');                           //Call the function clear_mem() to reset the memory of the code passed as argument
      }
    }
    if (inputmsg.indexOf("MOISER", 1) >= 0)
    {
      clear_ALM = JSONBufferRX["MOISER"];
      if (clear_ALM)
      {
        clear_mem('c');                           //Call the function clear_mem() to reset the memory of the code passed as argument
      }
    }

    // ILLUMINATION SETTING CODE RECEIVED
    if (inputmsg.indexOf("ILLSET", 1) >= 0)
    {
      illumin_sett = JSONBufferRX["ILLSET"];
      switch (illumin_sett)
      {
        case 0:
          set_illumination('x');
        break;

        case 1:
          set_illumination('a');
        break;

        case 2:
          set_illumination('b');
        break;

        case 3:
          set_illumination('c');
        break;

        case 4:
          set_illumination('d');
        break;

        case 5:
          set_illumination('e');
        break;

        case 6:
          set_illumination('f');
        break;

        case 7:
          set_illumination('g');
        break;

        case 8:
          set_illumination('h');
        break;

        case 9:
          set_illumination('i');
        break;
      }
    }

    // ROTATING CODE RECEIVED
    if (inputmsg.indexOf("ALILLP", 1) >= 0)
    {
      rot_ill_per = JSONBufferRX["ALILLP"];
      switch (rot_ill_per)
      {
        case 1:
          set_rotation('1');                       //Period set to 01 min => 6 cycles of 10 sec each = 60 seconds 
        break;

        case 2:
          set_rotation('2');                      //Period set to 05 min => 30 cycles of 10 sec each = 300 seconds 
        break;

        case 3:
          set_rotation('3');                       //Period set to 10 min => 60 cycles of 10 sec each = 600 seconds 
        break;

        case 4:
          set_rotation('4');                       //Period set to 15 min => 90 cycles of 10 sec each = 900 seconds 
        break;

        case 5:
          set_rotation('5');                       //Period set to 20 min => 120 cycles of 10 sec each = 1200 seconds 
        break;

        case 6:
          set_rotation('6');                       //Period set to 30 min => 180 cycles of 10 sec each = 1800 seconds 
        break;
      }
    }

    // ON/OFF PERIOD CODE RECEIVED
    if (inputmsg.indexOf("ONOFFM", 1) >= 0)
    {
      onoff_ill_mode = JSONBufferRX["ONOFFM"];
      switch (onoff_ill_mode)
      {
        case 0:
          set_onoff('0');                              //ON/OFF mode disabled (set to work permanently) 
        break;

        case 1:
          set_onoff('1');                             //ON/OFF period set to 04 hs => 1440 cycles of 10 sec each = 14400 seconds 
        break;

        case 2:
          set_onoff('2');                              //ON/OFF period set to 08 hs => 2880 cycles of 10 sec each = 28800 seconds 
        break;

        case 3:
          set_onoff('3');                             //ON/OFF period set to 12 hs => 4320 cycles of 10 sec each = 43200 seconds 
        break;
      }
    }

    // AIR FLOW SETUP CODE RECEIVED
    if (inputmsg.indexOf("AIRSTA", 1) >= 0)
    {
      airflow_sett = JSONBufferRX["AIRSTA"];
      switch (airflow_sett)
      {
        case 0:
          set_airflow('x');
        break;

        case 1:
          set_airflow('o');
        break;        
      }
    }

    // TEM | HUM | MOI SETUP CODE RECEIVED
    if (inputmsg.indexOf("TEMDES", 1) >= 0)
    {
      tem_des = JSONBufferRX["TEMDES"];
      tem_upl = tem_des + tem_del;
      tem_lol = tem_des - tem_del;
    }
    if (inputmsg.indexOf("TEMDEL", 1) >= 0)
    {
      tem_del = JSONBufferRX["TEMDEL"];
      tem_upl = tem_des + tem_del;
      tem_lol = tem_des - tem_del;
    }
    if (inputmsg.indexOf("TEMCOR", 1) >= 0)
    {
      tem_cor = JSONBufferRX["TEMCOR"];
    }
  
    if (inputmsg.indexOf("HUMDES", 1) >= 0)
    {
      hum_des = JSONBufferRX["HUMDES"];
      hum_upl = hum_des + hum_del;
      hum_lol = hum_des - hum_del;
    }
    if (inputmsg.indexOf("HUMDEL", 1) >= 0)
    {
      hum_del = JSONBufferRX["HUMDEL"];
      hum_upl = hum_des + hum_del;
      hum_lol = hum_des - hum_del;
    }
    if (inputmsg.indexOf("HUMCOR", 1) >= 0)
    {
      hum_cor = JSONBufferRX["HUMCOR"];
    }

    if (inputmsg.indexOf("MOIDES", 1) >= 0)
    {
      moi_des = JSONBufferRX["MOIDES"];
      moi_upl = moi_des + moi_del;
      moi_lol = moi_des - moi_del;
    }
    if (inputmsg.indexOf("MOIDEL", 1) >= 0)
    {
      moi_del = JSONBufferRX["MOIDEL"];
      moi_upl = moi_des + moi_del;
      moi_lol = moi_des - moi_del;
    }
    if (inputmsg.indexOf("MOICOR", 1) >= 0)
    {
      moi_cor = JSONBufferRX["MOICOR"];
    }
  }
  print_parameters('a');                          
}

// ############################## AUX FUNCTIONS
void print_parameters(char code)
{
  /* Received parameters values:
    code:
      "b" => Print the current parameter's values
      "a" => Print the parameter's values after update
  */
  switch (code)
  {
    case 'b':
      Serial.println("======================================== WeMos parameters (current) to be updated | Begin");

      Serial.println("Temperature Parameters \t");
      Serial.print("Upper limit for temperature: \t\t");
      Serial.print(tem_upl);
      Serial.println(" °C");
      Serial.print("Desired temperature: \t\t\t");
      Serial.print(tem_des);
      Serial.println(" °C");
      Serial.print("Lower limit for temperature: \t\t");
      Serial.print(tem_lol);
      Serial.println(" °C");
      Serial.print("Delta temperature: +/- \t\t\t");
      Serial.print(tem_del);
      Serial.println(" °C");
      Serial.print("Correction to temperature: \t\t");
      Serial.print(tem_cor);
      Serial.println(" °C");

      Serial.println("Humidity Parameters \t");
      Serial.print("Upper limit for Humidity: \t\t");
      Serial.print(hum_upl);
      Serial.println(" %");
      Serial.print("Desired Humidity: \t\t\t");
      Serial.print(hum_des);
      Serial.println(" %");
      Serial.print("Lower limit for Humidity: \t\t");
      Serial.print(hum_lol);
      Serial.println(" %");
      Serial.print("Delta Humidity: +/- \t\t\t");
      Serial.print(hum_del);
      Serial.println(" %");
      Serial.print("Correction to Humidity: \t\t");
      Serial.print(hum_cor);
      Serial.println(" %");

      Serial.println("Moisture Parameters \t");
      Serial.print("Upper limit for Moisture: \t\t");
      Serial.print(moi_upl);
      Serial.println(" %");
      Serial.print("Desired Moisture: \t\t\t");
      Serial.print(moi_des);
      Serial.println(" %");
      Serial.print("Lower limit for Moisture: \t\t");
      Serial.print(moi_lol);
      Serial.println(" %");
      Serial.print("Delta Moisture: +/- \t\t\t");
      Serial.print(moi_del);
      Serial.println(" %");
      Serial.print("Correction to Moisture: \t\t");
      Serial.print(moi_cor);
      Serial.println(" %");

      Serial.println("LEDs & FANs Status \t");
      Serial.print("Status of Central LED: \t\t\t");
      if (LED_A_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of Periferical LEDs: \t\t");
      if (LED_B_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of Periferical LEDs: \t\t");
      if (LED_C_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of IN-OU FANs: \t\t\t");
      if (FAN_A_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");

      Serial.println("======================================== WeMos parameters (current) to be updated | End");
    break;
    
    case 'a':
      Serial.println("======================================== WeMos parameters (new) successfully updated | Begin");

      Serial.println("Temperature Parameters \t");
      Serial.print("Upper limit for temperature: \t\t");
      Serial.print(tem_upl);
      Serial.println(" °C");
      Serial.print("Desired temperature: \t\t\t");
      Serial.print(tem_des);
      Serial.println(" °C");
      Serial.print("Lower limit for temperature: \t\t");
      Serial.print(tem_lol);
      Serial.println(" °C");
      Serial.print("Delta temperature: +/- \t\t\t");
      Serial.print(tem_del);
      Serial.println(" °C");
      Serial.print("Correction to temperature: \t\t");
      Serial.print(tem_cor);
      Serial.println(" °C");

      Serial.println("Humidity Parameters \t");
      Serial.print("Upper limit for Humidity: \t\t");
      Serial.print(hum_upl);
      Serial.println(" %");
      Serial.print("Desired Humidity: \t\t\t");
      Serial.print(hum_des);
      Serial.println(" %");
      Serial.print("Lower limit for Humidity: \t\t");
      Serial.print(hum_lol);
      Serial.println(" %");
      Serial.print("Delta Humidity: +/- \t\t\t");
      Serial.print(hum_del);
      Serial.println(" %");
      Serial.print("Correction to Humidity: \t\t");
      Serial.print(hum_cor);
      Serial.println(" %");

      Serial.println("Moisture Parameters \t");
      Serial.print("Upper limit for Moisture: \t\t");
      Serial.print(moi_upl);
      Serial.println(" %");
      Serial.print("Desired Moisture: \t\t\t");
      Serial.print(moi_des);
      Serial.println(" %");
      Serial.print("Lower limit for Moisture: \t\t");
      Serial.print(moi_lol);
      Serial.println(" %");
      Serial.print("Delta Moisture: +/- \t\t\t");
      Serial.print(moi_del);
      Serial.println(" %");
      Serial.print("Correction to Moisture: \t\t");
      Serial.print(moi_cor);
      Serial.println(" %");

      Serial.println("LEDs & FANs Status \t");
      Serial.print("Status of Central LED: \t\t\t");
      if (LED_A_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of Periferical LEDs: \t\t");
      if (LED_B_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of Periferical LEDs: \t\t");
      if (LED_C_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of IN-OU FANs: \t\t\t");
      if (FAN_A_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");

      Serial.println("======================================== WeMos parameters (new) successfully updated | End");
    break;
    
    defualt:
      Serial.println("Something went wrong. Check the code."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

// ############################## AUX FUNCTIONS
void print_measurements(float value_mea, float value_mem, int value_cor, char code)
{
  /* Received parameters values:
    value_mea:
      "#" => Value of the measurement to be printed
    value_mem:
      "#" => Value (averaged) of the measurement to be printed
    value_cor:
      "#" => Value (correction) of the measurement to be printed
    code:
      "t" => Measurement to be printed: Temperature
      "h" => Measurement to be printed: Humidity
      "m" => Measurement to be printed: Moisture
  */
  switch (code)
  {
    case 't':
      Serial.println("======================================== WeMos Temperature measurements | Begin");
      Serial.println("Temperature (actual)\tTemperature (mem)\tCorrection"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_mea);                    //Print the current value of the tem_raw variable to Serial Comm (USB)
      Serial.print(" °C\t\t");                    //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_mem);                    //Print the corrected (weighted average) value of temperature to Serial Comm (USB)
      Serial.print(" °C\t\t");                    //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_cor);                    //Print the correction factor of temperature to Serial Comm (USB)
      Serial.println(" °C");                      //Print the auxiliar text to Serial Comm (USB)
      Serial.println("======================================== WeMos Temperature measurements | End");
    break;

    case 'h':
      Serial.println("======================================== WeMos Humidity measurements | Begin");
      Serial.println("Humidity (actual)\tHumidity (mem)\t\tCorrection"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_mea);                    //Print the current value of the hum_raw variable to Serial Comm (USB)
      Serial.print(" %\t\t\t");                   //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_mem);                    //Print the corrected (weighted average) value of humidity to Serial Comm (USB)
      Serial.print(" %\t\t\t");                   //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_cor);                    //Print the correction factor of humidity to Serial Comm (USB)
      Serial.println(" %");                       //Print the auxiliar text to Serial Comm (USB)
      Serial.println("======================================== WeMos Humidity measurements | End");
    break;
    
    case 'm':
      Serial.println("======================================== WeMos Moisture measurements | Begin");
      Serial.println("Moisture (actual)\tMoisture (mem)\t\tCorrection"); //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_mea);                    //Print the current value of the moi_raw variable to Serial Comm (USB)
      Serial.print(" %\t\t\t");                   //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_mem);                    //Print the corrected (weighted average) value of moisture to Serial Comm (USB)
      Serial.print(" %\t\t\t");                     //Print the auxiliar text to Serial Comm (USB)
      Serial.print(value_cor);                    //Print the correction factor of moisture to Serial Comm (USB)
      Serial.println(" %");                       //Print the auxiliar text to Serial Comm (USB)
      Serial.println("======================================== WeMos Moisture measurements | End");
     break;

    defualt:
      Serial.println("Something went wrong. Check the program."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

// ############################## AUX FUNCTIONS
float mem_logic(float value_mea, bool mea_ini, char code)
{
  /* Received parameters values:
    value_mea:
      "#" => Value of the measurement to be processed
    mea_ini:
      "#" => Flag for averaging the measurement to be processed (initial condition memory)
             If mea_ini == 1 => System just started - averaging with the first measurement 
             If mea_ini == 0 => System already working - averaging with bubble method 
    code:
      "t" => Measurement to be processed: Temperature
      "h" => Measurement to be processed: Humidity
      "m" => Measurement to be processed: Moisture
  */
  int i = 0;
  float aux = 0;

  switch (code)
  {
    case 't':
      if (mea_ini == 1)
      {
        for (i = 0; i < 5; i++)
          tem_vec_mem[i] = value_mea;
        mea_ini = 0;
      }
      else
      {
        for (i = 0; i < 4; i++)
          tem_vec_mem[4 - i] = tem_vec_mem[4 - (i + 1)];
        tem_vec_mem[0] = value_mea;
      }

      for (i = 0; i < 5; i++)
        aux += tem_vec_mem[i];
      aux = aux / 5;
      return aux;
    break;

    case 'h':
      if (mea_ini == 1)
      {
        for (i = 0; i < 5; i++)
          hum_vec_mem[i] = value_mea;
        mea_ini = 0;
      }
      else
      {
        for (i = 0; i < 4; i++)
          hum_vec_mem[4 - i] = hum_vec_mem[4 - (i + 1)];
        hum_vec_mem[0] = value_mea;
      }

      for (i = 0; i < 5; i++)
        aux += hum_vec_mem[i];
      aux = aux / 5;
      return aux;
    break;

    case 'm':
      if (mea_ini == 1)
      {
        for (i = 0; i < 5; i++)
          moi_vec_mem[i] = value_mea;
        mea_ini = 0;
      }
      else
      {
        for (i = 0; i < 4; i++)
          moi_vec_mem[4 - i] = moi_vec_mem[4 - (i + 1)];
        moi_vec_mem[0] = value_mea;
      }

      for (i = 0; i < 5; i++)
        aux += moi_vec_mem[i];
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
void tem_logic()
{
  tem_raw = dht.readTemperature();            //Read the temperature form the DHT22
  if ((isnan(tem_raw)) || (tem_raw==998) || (tem_raw==999))
  {
    if (isnan(tem_raw))
      Serial.println("No measurements comming from sensor. Check connections!");
    if (tem_raw==998)
      Serial.println("I2C timed out. Check your connections!");
    if (tem_raw==999)
      Serial.println("CRC mismatch. Check your connections!");

    tem_sen_cou = tem_sen_cou + 1;
    tem_sen_sta = 1;
    tem_sen_mem = 1;
  }
  else
  {
    tem_sen_sta = 0;
    tem_mem = mem_logic(tem_raw, tem_ini_mem, 't'); //Call mem_logic() => Calculate average of last 5 previous values
    print_measurements(tem_raw, tem_mem, tem_cor, 't' ); //Call print_measurements()
    if (((tem_mem - tem_cor) >= tem_lol) && ((tem_mem - tem_cor) <= tem_upl)) //Corrected temperature in between defined thresholds
    {
      tem_ala_sta = 0;
    }
    else
    {
      if((tem_mem - tem_cor) < tem_lol)       //Corrected temperature below lower defined threshold
      {
        //Condition prepared for future code (take some action the threshold is broken below or above)
        //Ex.: turn on more LEDs
        tem_ala_sta = 1;
        tem_ala_mem = 1;
        tem_ala_cou = tem_ala_cou + 1;
      }
      if((tem_mem - tem_cor) > tem_upl)       //Corrected temperature above upper defined threshold
      {
        //Condition prepared for future code (take some action the threshold is broken below or above)
        //Ex.: turn on more FANs
        tem_ala_sta = 1;
        tem_ala_mem = 1;
        tem_ala_cou = tem_ala_cou + 1;
      }
    }
  }
}

// ############################## AUX FUNCTIONS
void hum_logic()
{
  hum_raw = dht.readHumidity();               //Read the Humidity form the DHT22
  if ((isnan(hum_raw)) || (hum_raw==998) || (hum_raw==999))
  {
    if (isnan(hum_raw))
      Serial.println("No measurements comming from sensor. Check connections!");
    if (hum_raw==998)
      Serial.println("I2C timed out. Check your connections!");
    if (hum_raw==999)
      Serial.println("CRC mismatch. Check your connections!");

    hum_sen_cou = hum_sen_cou + 1;
    hum_sen_sta = 1;
    hum_sen_mem = 1;
  }
  else
  {
    hum_sen_sta = 0;
    hum_mem = mem_logic(hum_raw, hum_ini_mem, 'h'); //Call mem_logic() => Calculate average of last 5 previous values
    print_measurements(hum_raw, hum_mem, hum_cor, 'h' ); //Call print_measurements()
    if (((hum_mem - hum_cor) >= hum_lol) && ((hum_mem - hum_cor) <= hum_upl)) //Corrected Humidity in between defined thresholds
    {
      hum_ala_sta = 0;
    }
    else
    {
      if((hum_mem - hum_cor) < hum_lol)       //Corrected Humidity below lower defined threshold
      {
        //Condition prepared for future code (take some action the threshold is broken below or above)
        //Ex.: turn on humidifier
        hum_ala_sta = 1;
        hum_ala_mem = 1;
        hum_ala_cou = hum_ala_cou + 1;
      }
      if((hum_mem - hum_cor) > hum_upl)       //Corrected Humidity above upper defined threshold
      {
        //Condition prepared for future code (take some action the threshold is broken below or above)
        //Ex.: turn on more FANs
        hum_ala_sta = 1;
        hum_ala_mem = 1;
        hum_ala_cou = hum_ala_cou + 1;
      }
    }
  }
}

// ############################## AUX FUNCTIONS
void moi_logic()
{
  moi_unr = analogRead(A0);                       //Read the moisture form the MOI
  moi_raw = map(moi_unr, AirValue, WaterValue, 0, 100);
  if ((isnan(moi_raw)) || (moi_raw==998) || (moi_raw==999))
  {
    if (isnan(moi_raw))
      Serial.println("No measurements comming from sensor. Check connections!");
    if (moi_raw==998)
      Serial.println("I2C timed out. Check your connections!");
    if (moi_raw==999)
      Serial.println("CRC mismatch. Check your connections!");

    moi_sen_cou = moi_sen_cou + 1;
    moi_sen_sta = 1;
    moi_sen_mem = 1;
  }
  else
  {
    moi_sen_sta = 0;
    if (moi_raw < 0)
    {
      moi_raw = 0;
      Serial.println("Miosture value below 0. Check sensor/calibration!");
    }
    if (moi_raw > 100)
    {
      moi_raw = 100;
      Serial.println("Miosture value above 100. Check sensor/calibration!");
    }

    moi_mem = mem_logic(moi_raw, moi_ini_mem, 'm'); //Call mem_logic() => Calculate average of last 5 previous values
    print_measurements(moi_raw, moi_mem, moi_cor, 'm' ); //Call print_measurements()
    if (((moi_mem - moi_cor) >= moi_lol) && ((moi_mem - moi_cor) <= moi_upl)) //Corrected moisture in between defined thresholds
    {
      moi_ala_sta = 0;
    }
    else
    {
      if((moi_mem - moi_cor) < moi_lol)       //Corrected moisture below lower defined threshold
      {
        //Condition prepared for future code (take some action the threshold is broken below or above)
        //Ex.: turn on watering system
        moi_ala_sta = 1;
        moi_ala_mem = 1;
        moi_ala_cou = moi_ala_cou + 1;
      }
      if((moi_mem - moi_cor) > moi_upl)       //Corrected moisture above upper defined threshold
      {
        //Condition prepared for future code (take some action the threshold is broken below or above)
        //Ex.: stop watering
        moi_ala_sta = 1;
        moi_ala_mem = 1;
        moi_ala_cou = moi_ala_cou + 1;
      }
    }
  }
}

// ############################## AUX FUNCTIONS
void check_illum()
{
  if (illumin_sett == 0)                        //"x" => OFF Illumination: All LEDs OFF
  {
    if ((LED_A_stat == 0) && (LED_B_stat == 0) && (LED_C_stat == 0))
    {
      Serial.println("Illumination already set to OFF");
    }
    else
    {
      Serial.println("======================================== WeMos Illumination Set to OFF | Begin");
      digitalWrite(LED_A_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
      LED_A_stat = 0;                             //Update the value of the boolean variable to follow up the status
      digitalWrite(LED_B_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
      LED_B_stat = 0;                             //Update the value of the boolean variable to follow up the status
      digitalWrite(LED_C_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C OFF
      LED_C_stat = 0;                             //Update the value of the boolean variable to follow up the status
      Serial.println("======================================== WeMos Illumination Set to OFF | End");
    }
  }
  if (illumin_sett == 1)                        //"a" => VLOW Illumination: Only central LED ON
  {
      if ((LED_A_stat == 1) && (LED_B_stat == 0) && (LED_C_stat == 0))
      {
        Serial.println("Illumination already set to VLOW");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to VLOW | Begin");
        digitalWrite(LED_A_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_A ON
        LED_A_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
        LED_B_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C OFF
        LED_C_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to VLOW | End");
      }
    }
  if (illumin_sett == 2)                        //"b" => LOWA Illumination: Only \ LEDs ON
  {
    if ((LED_A_stat == 0) && (LED_B_stat == 1) && (LED_C_stat == 0))
      {
        Serial.println("Illumination already set to LOWA");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to LOWA | Begin");
        digitalWrite(LED_A_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
        LED_A_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_B ON
        LED_B_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C OFF
        LED_C_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to LOWA | End");
      }
  }
  if (illumin_sett == 3)                        //"c" => LOWB Illumination: Only / LEDs ON
  {
      if ((LED_A_stat == 0) && (LED_B_stat == 0) && (LED_C_stat == 1))
      {
        Serial.println("Illumination already set to LOWB");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to LOWB | Begin");
        digitalWrite(LED_A_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
        LED_A_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
        LED_B_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_C ON
        LED_C_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to LOWB | End");
      }
  }
  if (illumin_sett == 4)                        //"d" => MIDA Illumination: Central and \ LEDs ON
  {
      if ((LED_A_stat == 1) && (LED_B_stat == 1) && (LED_C_stat == 0))
      {
        Serial.println("Illumination already set to MIDA");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to MIDA | Begin");
        digitalWrite(LED_A_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_A ON
        LED_A_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_B ON
        LED_B_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C OFF
        LED_C_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to MIDA | End");
      }
  }
  if (illumin_sett == 5)                        //"e" => MIDB Illumination: Central and / LEDs ON
  {
      if ((LED_A_stat == 1) && (LED_B_stat == 0) && (LED_C_stat == 1))
      {
        Serial.println("Illumination already set to MIDB");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to MIDB | Begin");
        digitalWrite(LED_A_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_A ON
        LED_A_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
        LED_B_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_C ON
        LED_C_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to MIDB | End");
      }
  }
  if (illumin_sett == 6)                        //"f" => HIGH Illumination: Only \ and / LEDs ON
  {
      if ((LED_A_stat == 0) && (LED_B_stat == 1) && (LED_C_stat == 1))
      {
        Serial.println("Illumination already set to HIGH");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to HIGH | Begin");
        digitalWrite(LED_A_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
        LED_A_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_B ON
        LED_B_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_C ON
        LED_C_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to HIGH | End");
      }
  }
  if (illumin_sett == 7)                        //"g" => VHIG Illumination: All LEDs ON
  {
      if ((LED_A_stat == 1) && (LED_B_stat == 1) && (LED_C_stat == 1))
      {
        Serial.println("Illumination already set to VHIG");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to VHIG | Begin");
        digitalWrite(LED_A_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_A ON
        LED_A_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_B ON
        LED_B_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_C ON
        LED_C_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to VHIG | End");
      }
  }
}

// ############################## AUX FUNCTIONS
void check_rot()
{
  if (global_off == 0)
  {
    if (rot_ill_count > 0)
    {
      rot_ill_count = rot_ill_count - 1;
    }
    else
    {
      rot_ill_count = rot_ill_cyc;
      switch (illumin_sett)
      {
        case 0:                                         //No rotation due to Illuminations Levels
          set_illumination('x');                        //Set to 0
          break;

        case 1:                                         //No rotation due to Illuminations Levels
          set_illumination('a');
          break;

        case 2:                                         //Rotating LOW Illumination Levels
          set_illumination('c');
          break;

        case 3:                                         //Rotating LOW Illumination Levels
          set_illumination('b');
          break;

        case 4:                                         //Rotating MID Illumination Levels
          set_illumination('e');
          break;

        case 5:                                         //Rotating MID Illumination Levels
          set_illumination('d');
          break;

        case 6:                                         //No rotation due to Illuminations Levels
          set_illumination('f');
          break;

        case 7:                                         //No rotation due to Illuminations Levels
          set_illumination('g');
          break;

        defualt:
          Serial.println("Something went wrong. Check HW & code."); //Print the auxiliar text to Serial Comm (USB)
        break;
      }
    }
  }
}

// ############################## AUX FUNCTIONS
void check_onoff()
{
  if (onoff_ill_mode != 0)
  {
    if (onoff_ill_count > 0)
    {
      onoff_ill_count = onoff_ill_count - 1;
    }
    else
    {
      onoff_ill_count = onoff_ill_cyc;
      if (onoff_ill_stat == 0)
      {
        prev_illum = illumin_sett;
        prev_airfl = airflow_sett;
        set_illumination('x');
        set_airflow('x');
        onoff_ill_stat = 1;
        global_off = 1;
      }
      else
      {
        switch (prev_illum)
        {
          case 0:
            set_illumination('x');
            break;

          case 1:
            set_illumination('a');
            break;

          case 2:
            set_illumination('b');
            break;

          case 3:
            set_illumination('c');
            break;

          case 4:
            set_illumination('d');
            break;

          case 5:
            set_illumination('e');
            break;

          case 6:
            set_illumination('f');
            break;

          case 7:
            set_illumination('g');
            break;
        }
        switch (prev_airfl)
        {
          case 0:
            set_airflow('x');
            break;

          case 1:
            set_airflow('o');
            break;
        }
        onoff_ill_stat = 0;
        global_off = 0;
      }
    }
  }
}

// ############################## AUX FUNCTIONS
void set_illumination(char illc)
{
  /* Received parameters values:
    code:
      "x" => OFF Illumination: All LEDs OFF                         (5 LEDs OFF)
      "a" => VLOW Illumination: Only central LED ON                 (1 LEDs ON in total)
      "b" => LOWA Illumination: Only \ LEDs ON                      (2 LEDs ON in total)
      "c" => LOWB Illumination: Only / LEDs ON                      (2 LEDs ON in total)
      "d" => MIDA Illumination: Central and \ LEDs ON               (3 LEDs ON in total)
      "e" => MIDB Illumination: Central and / LEDs ON               (3 LEDs ON in total)
      "f" => HIGH Illumination: Only \ and / LEDs ON                (4 LEDs ON in total)
      "g" => VHIG Illumination: All LEDs ON                         (5 LEDs ON in total)
  */
  switch (illc)
  {
    case 'x':
      illumin_sett = 0;                             //Set the code for illumination
      rot_ill_count = rot_ill_cyc;                  //Reset the rotation cyclic counter
      if ((LED_A_stat == 0) && (LED_B_stat == 0) && (LED_C_stat == 0))
      {
        Serial.println("Illumination already set to OFF");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to OFF | Begin");
        digitalWrite(LED_A_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
        LED_A_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
        LED_B_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C OFF
        LED_C_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to OFF | End");
      }
    break;

    case 'a':
      illumin_sett = 1;                             //Set the code for illumination
      rot_ill_count = rot_ill_cyc;                  //Reset the rotation cyclic counter
      if ((LED_A_stat == 1) && (LED_B_stat == 0) && (LED_C_stat == 0))
      {
        Serial.println("Illumination already set to VLOW");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to VLOW | Begin");
        digitalWrite(LED_A_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_A ON
        LED_A_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
        LED_B_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C OFF
        LED_C_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to VLOW | End");
      }
    break;

    case 'b':
      illumin_sett = 2;                             //Set the code for illumination
      rot_ill_count = rot_ill_cyc;                  //Reset the rotation cyclic counter
      if ((LED_A_stat == 0) && (LED_B_stat == 1) && (LED_C_stat == 0))
      {
        Serial.println("Illumination already set to LOWA");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to LOWA | Begin");
        digitalWrite(LED_A_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
        LED_A_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_B ON
        LED_B_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C OFF
        LED_C_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to LOWA | End");
      }
    break;

    case 'c':
      illumin_sett = 3;                             //Set the code for illumination
      rot_ill_count = rot_ill_cyc;                  //Reset the rotation cyclic counter
      if ((LED_A_stat == 0) && (LED_B_stat == 0) && (LED_C_stat == 1))
      {
        Serial.println("Illumination already set to LOWB");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to LOWB | Begin");
        digitalWrite(LED_A_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
        LED_A_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
        LED_B_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_C ON
        LED_C_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to LOWB | End");
      }
    break;

    case 'd':
      illumin_sett = 4;                             //Set the code for illumination
      rot_ill_count = rot_ill_cyc;                  //Reset the rotation cyclic counter
      if ((LED_A_stat == 1) && (LED_B_stat == 1) && (LED_C_stat == 0))
      {
        Serial.println("Illumination already set to MIDA");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to MIDA | Begin");
        digitalWrite(LED_A_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_A ON
        LED_A_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_B ON
        LED_B_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C OFF
        LED_C_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to MIDA | End");
      }
    break;

    case 'e':
      illumin_sett = 5;                             //Set the code for illumination
      rot_ill_count = rot_ill_cyc;                  //Reset the rotation cyclic counter
      if ((LED_A_stat == 1) && (LED_B_stat == 0) && (LED_C_stat == 1))
      {
        Serial.println("Illumination already set to MIDB");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to MIDB | Begin");
        digitalWrite(LED_A_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_A ON
        LED_A_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
        LED_B_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_C ON
        LED_C_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to MIDB | End");
      }
    break;

    case 'f':
      illumin_sett = 6;                             //Set the code for illumination
      rot_ill_count = rot_ill_cyc;                  //Reset the rotation cyclic counter
      if ((LED_A_stat == 0) && (LED_B_stat == 1) && (LED_C_stat == 1))
      {
        Serial.println("Illumination already set to HIGH");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to HIGH | Begin");
        digitalWrite(LED_A_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
        LED_A_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_B ON
        LED_B_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_C ON
        LED_C_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to HIGH | End");
      }
    break;

    case 'g':
      illumin_sett = 7;                             //Set the code for illumination
      rot_ill_count = rot_ill_cyc;                  //Reset the rotation cyclic counter
      if ((LED_A_stat == 1) && (LED_B_stat == 1) && (LED_C_stat == 1))
      {
        Serial.println("Illumination already set to VHIG");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to VHIG | Begin");
        digitalWrite(LED_A_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_A ON
        LED_A_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_B_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_B ON
        LED_B_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_C_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_C ON
        LED_C_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to VHIG | End");
      }
    break;

    defualt:
      Serial.println("Something went wrong. Check HW & code."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

// ############################## AUX FUNCTIONS
void set_rotation(char rotc)
{  
  /* Received parameters values:
    code:
      '1' =>  Rotation period set to 01 min => 6 cycles of 10 sec each = 60 seconds 
      '2' =>  Rotation period set to 05 min => 30 cycles of 10 sec each = 300 seconds 
      '3' =>  Rotation period set to 10 min => 60 cycles of 10 sec each = 600 seconds 
      '4' =>  Rotation period set to 15 min => 90 cycles of 10 sec each = 900 seconds 
      '5' =>  Rotation period set to 20 min => 120 cycles of 10 sec each = 1200 seconds 
      '6' =>  Rotation period set to 30 min => 180 cycles of 10 sec each = 1800 seconds 
  */
  switch (rotc)
  {
    case '1':
      rot_ill_per = 1;
      rot_ill_cyc = 6;                          //Period set to 01 min => 6 cycles of 10 sec each = 60 seconds 
      rot_ill_count =rot_ill_cyc;
    break;

    case '2':
      rot_ill_per = 2;
      rot_ill_cyc = 30;                         //Period set to 05 min => 30 cycles of 10 sec each = 300 seconds 
      rot_ill_count =rot_ill_cyc;
    break;

    case '3':
      rot_ill_per = 3;
      rot_ill_cyc = 60;                         //Period set to 10 min => 60 cycles of 10 sec each = 600 seconds 
      rot_ill_count =rot_ill_cyc;
    break;

    case '4':
      rot_ill_per = 4;
      rot_ill_cyc = 90;                         //Period set to 15 min => 90 cycles of 10 sec each = 900 seconds 
      rot_ill_count =rot_ill_cyc;
    break;

    case '5':
      rot_ill_per = 5;
      rot_ill_cyc = 120;                        //Period set to 20 min => 120 cycles of 10 sec each = 1200 seconds 
      rot_ill_count =rot_ill_cyc;
    break;

    case '6':
      rot_ill_per = 6;
      rot_ill_cyc = 180;                        //Period set to 30 min => 180 cycles of 10 sec each = 1800 seconds 
      rot_ill_count =rot_ill_cyc;
    break;
  }
}

// ############################## AUX FUNCTIONS
void set_onoff(char onoffc)
{  
  /* Received parameters values:
    code:
      '0' =>  ON/OFF mode disabled (set to work permanently) 
      '1' =>  ON/OFF period set to 04 hs => 1440 cycles of 10 sec each = 14400 seconds 
      '2' =>  ON/OFF period set to 08 hs => 2880 cycles of 10 sec each = 28800 seconds 
      '3' =>  ON/OFF period set to 12 hs => 4320 cycles of 10 sec each = 43200 seconds 
  */
  switch (onoffc)
  {
    case '0':
      onoff_ill_mode = 0;                       //ON/OFF mode disabled (set to work permanently) 
    break;

    case '1':
      onoff_ill_mode = 1;
      //onoff_ill_cyc = 1440;                     //ON/OFF period set to 04 hs => 1440 cycles of 10 sec each = 14400 seconds 
      onoff_ill_cyc = 6;     //test
      onoff_ill_count = onoff_ill_cyc;
    break;

    case '2':
      onoff_ill_mode = 2;
      onoff_ill_cyc = 2880;                     //ON/OFF period set to 08 hs => 2880 cycles of 10 sec each = 28800 seconds 
      onoff_ill_count = onoff_ill_cyc;
    break;

    case '3':
      onoff_ill_mode = 3;
      onoff_ill_cyc = 4320;                     //ON/OFF period set to 12 hs => 4320 cycles of 10 sec each = 43200 seconds 
      onoff_ill_count = onoff_ill_cyc;
    break;
  }
}

// ############################## AUX FUNCTIONS
void set_airflow(char airc)
{
  /* Received parameters values:
    code:
      "x" => NO AIR FLOW: Air Flow turned OFF (FANs OFF)
      "o" => AIR FLOW: Air Flow turned ON (FANs ON )
  */
  switch (airc)
  {
    case 'x':
      if (FAN_A_stat == 0)
      {
        Serial.println("Air Flow already Set to OFF");
      }
      else
      {
        Serial.println("======================================== WeMos Air Flow Set to OFF | Begin");
        digitalWrite(FAN_A_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_A OFF
        FAN_A_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Air Flow Set to OFF | End");
      }
      airflow_sett = 0;                             //Set the code for initial air flow
    break;

    case 'o':
      if (FAN_A_stat == 1)
      {
        Serial.println("Air Flow already Set to ON");
      }
      else
      {
        Serial.println("======================================== WeMos Air Flow Set to ON | Begin");
        digitalWrite(FAN_A_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_A ON
        FAN_A_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Air Flow Set to ON | End");
      }
      airflow_sett = 1;                             //Set the code for initial air flow
    break;

    defualt:
      Serial.println("Something went wrong. Check HW & code."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

// ############################## AUX FUNCTIONS
void clear_mem(char clrc)
{
  /* Received parameters values:
    code:
      "a" => Reset the temperature's sensor failure memory
      "b" => Reset the humidity's sensor failure memory
      "c" => Reset the moisture's sensor failure memory
      "t" => Reset the temperature's alarm memory
      "h" => Reset the humidity's alarm memory
      "m" => Reset the moisture's alarm memory
  */
  switch (clrc)
  {
    case 'a':
      Serial.println("Clearing temperature's sensor failure memory & counter...");
      tem_sen_mem = 0;
      tem_sen_cou = 0;
    break;
    
    case 'b':
      Serial.println("Clearing humidity's sensor failure memory & counter...");
      hum_sen_mem = 0;
      hum_sen_cou = 0;
    break;
    
    case 'c':
      Serial.println("Clearing moisture's sensor failure memory & counter...");
      moi_sen_mem = 0;
      moi_sen_cou = 0;
    break;

    case 't':
      Serial.println("Clearing temperature's alarm memory & counter...");
      tem_ala_mem = 0;
      tem_ala_cou = 0;
    break;

    case 'h':
      Serial.println("Clearing humidity's alarm memory & counter...");
      hum_ala_mem = 0;
      hum_ala_cou = 0;
    break;

    case 'm':
      Serial.println("Clearing moisture's alarm memory & counter...");
      moi_ala_mem = 0;
      moi_ala_cou = 0;
    break;

    defualt:
      Serial.println("Something went wrong. Check the code."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

// ############################## AUX FUNCTIONS
void reset_relay(char code)
{
  /* Received parameters values:
    code:
      "a" => Reset the LED_A Relay - Central LED
      "b" => Reset the LED_B Relay - LEDs \
      "c" => Reset the LED_C Relay - LEDs /
      "f" => Reset the FAN_A Relay - IN-OU FANs
  */
  switch (code)
  {
    case 'a':
      Serial.println("CENTRAL LED RESET request received - Resetting Realy in 3sec...");
      delay(1000);
      Serial.println("CENTRAL LED RESET request received - Resetting Realy in 2sec...");
      delay(1000);
      Serial.println("CENTRAL LED RESET request received - Resetting Realy in 1sec...");
      delay(1000);

      if (LED_A_stat == 0)
      {
        digitalWrite(LED_A_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_A ON
        LED_A_stat = 1;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(LED_A_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
        LED_A_stat = 0;                           //Update the value of the boolean variable to follow up the status
      }

      if (LED_A_stat == 1)
      {
        digitalWrite(LED_A_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_A OFF
        LED_A_stat = 0;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(LED_A_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_A ON
        LED_A_stat = 1;                           //Update the value of the boolean variable to follow up the status
      }
    break;
    
    case 'b':
      Serial.println("PERIFERIAL LEDs RESET request received - Resetting Realy in 3sec...");
      delay(1000);
      Serial.println("PERIFERIAL LEDs RESET request received - Resetting Realy in 2sec...");
      delay(1000);
      Serial.println("PERIFERIAL LEDs RESET request received - Resetting Realy in 1sec...");
      delay(1000);

      if (LED_B_stat == 0)
      {
        digitalWrite(LED_B_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_B ON
        LED_B_stat = 1;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(LED_B_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
        LED_B_stat = 0;                           //Update the value of the boolean variable to follow up the status
      }

      if (LED_B_stat == 1)
      {
        digitalWrite(LED_B_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_B OFF
        LED_B_stat = 0;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(LED_B_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_B ON
        LED_B_stat = 1;                           //Update the value of the boolean variable to follow up the status
      }
    break;

    case 'c':
      Serial.println("PERIFERIAL LEDs RESET request received - Resetting Realy in 3sec...");
      delay(1000);
      Serial.println("PERIFERIAL LEDs RESET request received - Resetting Realy in 2sec...");
      delay(1000);
      Serial.println("PERIFERIAL LEDs RESET request received - Resetting Realy in 1sec...");
      delay(1000);

      if (LED_C_stat == 0)
      {
        digitalWrite(LED_C_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_C_PIN ON
        LED_C_stat = 1;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(LED_C_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C_PIN OFF
        LED_C_stat = 0;                           //Update the value of the boolean variable to follow up the status
      }

      if (LED_C_stat == 1)
      {
        digitalWrite(LED_C_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_C_PIN OFF
        LED_C_stat = 0;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(LED_C_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_C_PIN ON
        LED_C_stat = 1;                           //Update the value of the boolean variable to follow up the status
      }
    break;

    case 'f':
      Serial.println("FANs RESET request received - Resetting Realy in 3sec...");
      delay(1000);
      Serial.println("FANs RESET request received - Resetting Realy in 2sec...");
      delay(1000);
      Serial.println("FANs RESET request received - Resetting Realy in 1sec...");
      delay(1000);

      if (FAN_A_stat == 0)
      {
        digitalWrite(FAN_A_PIN, LOW);            //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_A_PIN ON
        FAN_A_stat = 1;                          //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(FAN_A_PIN, HIGH);           //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_A_PIN OFF
        FAN_A_stat = 0;                          //Update the value of the boolean variable to follow up the status
      }

      if (FAN_A_stat == 1)
      {
        digitalWrite(FAN_A_PIN, HIGH);           //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_A_PIN OFF
        FAN_A_stat = 0;                          //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(FAN_A_PIN, LOW);            //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_A_PIN ON
        FAN_A_stat = 1;                          //Update the value of the boolean variable to follow up the status
      }
    break;

    defualt:
      Serial.println("Something went wrong. Check relay's code."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

// ############################## AUX FUNCTIONS
void reset_sys()
{
  Serial.println("SYS RESET request received - Shooting-down and resetting WeMos SW/HW in 9sec...");
  delay(1000);
  Serial.println("SYS RESET request received - Shooting-down and resetting WeMos SW/HW in 8sec...");
  delay(1000);
  Serial.println("SYS RESET request received - Shooting-down and resetting WeMos SW/HW in 7sec...");
  delay(1000);
  Serial.println("SYS RESET request received - Shooting-down and resetting WeMos SW/HW in 6sec...");
  delay(1000);
  Serial.println("SYS RESET request received - Shooting-down and resetting WeMos SW/HW in 5sec...");
  delay(1000);
  Serial.println("SYS RESET request received - Shooting-down and resetting WeMos SW/HW in 4sec...");
  delay(1000);
  Serial.println("SYS RESET request received - Shooting-down and resetting WeMos SW/HW in 3sec...");
  delay(1000);
  Serial.println("SYS RESET request received - Shooting-down and resetting WeMos SW/HW in 2sec...");
  delay(1000);
  Serial.println("SYS RESET request received - Shooting-down and resetting WeMos SW/HW in 1sec...");
  delay(1000);
  ESP.restart();                              //Reboot WeMos's SW/HW.
}

// ############################## AUX FUNCTIONS
void setup_chrono()
{
  abs_time.restart();
}

// ############################## AUX FUNCTIONS
void get_chrono()
{
  chrono_time = abs_time.elapsed();
}

// ############################## AUX FUNCTIONS
void print_chrono()
{
  Serial.println("======================================== WeMos running time | Begin");
  Serial.print("Total running time: ");
  Serial.print(abs_time.elapsed());
  Serial.println(" sec");
  Serial.println("======================================== WeMos running time | End");
}

// ############################## AUX FUNCTIONS
void time_keeper()
{
  secsNow = millis()/1000;
  seconds = secsNow - secsLast;

  if (seconds >= 60)
  {
    seconds = 0;
    secsLast = secsNow;
    minutes = minutes + 1;
  } 

  if (minutes == 60)
  {
    minutes = 0;
    hours = hours + 1;
  }

  if (hours == 24)
  {
    hours = 0;
    days = days + 1;
  }
}
