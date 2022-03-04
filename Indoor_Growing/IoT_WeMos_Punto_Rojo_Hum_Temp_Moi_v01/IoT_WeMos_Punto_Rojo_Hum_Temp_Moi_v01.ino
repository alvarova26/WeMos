// ############################## IoT Project to Pub/Sub measurements via MQTT Broker CloudMQTT 

/* ################################################################################################### */
/* ############################# CHECK Notes v00.h FOR DETAILS => libraries, values, behavior, etc. ## */
/* ################################################################################################### */

/* ############################## Project's Description
The device (WeMos D1 R1 chipset with ESP8266 onborad) will report:
- Temperature (°C)
- Humidity (%)
- Moisture (%)
- PH of Soil (#) - Future Use
- Open/Close Door (On/Off) - Future Use
Along other measurements, every 10 sec via CloudMQTT Broker (using mqtt protocol)
At the same time, WeMos will accept commands form MQTT Broker (Publish/subscribe)  */

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
#include <DHT.h>

// ############################## Define PINs equivalences
#define D0    3     //GPIO03                      => Not used in this project
#define D1    1     //GPIO01                      => Not used in this project
#define D2    16    //GPIO16                      => NOT WORKING
#define D3    5     //GPIO05                      => Not used in this project (future use)
#define D4    4     //GPIO04                      => OUTPUT to Relay: LED_1 control
#define D5    14    //GPIO14                      => OUTPUT to Relay: LED_4 control
#define D6    12    //GPIO12                      => OUTPUT to Relay: FAN_I control
#define D7    13    //GPIO13                      => OUTPUT to Relay: FAN_O control
#define D8    0     //GPIO00                      => INPUT from DHT22 sensor (Temperature & Humidity)
#define D9    2     //GPIO02                      => INPUT from PH sensor (future use)
#define D10   15    //GPIO15                      => INPUT from DOOR sensor (future use)
#define A0    A0    //ANALOG                      => INPUT from MOISTURE sensor (Moisture of Soil)

// ############################## WiFi Netwrok Parameters
const char* ssid = "ALVARO1";                     //Change this to your network SSID (name).
const char* pass = "zurdito2017";                 //Change this your network password

// ############################## MQTT Broker Parameters
const char* mqttServer = "m16.cloudmqtt.com";     //Server MQTT
const char* mqttUserName = "oepdtofo";            //Username provided by the MQTT Broker
const char* mqttPass = "hcJjL0FbT40U";            //Password provided by the MQTT Broker
const char* mqttClientID = "WeMosClient";         //ClientID provided by the MQTT Broker (MUST BE UNIQUE for each PUB/SUB - i.e. the app on the cell phone must be different)
const char* topicWeMosReadings = "WeMosReadings"; //Topic to send WeMos readings to MQTT Broker / Subs
const char* topicUpdateWeMos = "UpdateWeMos";     //Topic to get Update values to WeMos from MQTT Broker / Pubs
const int mqttPort = 17879;                       //Port MQTT
unsigned int msglength = 0;                          //msglength of the incoming MQTT msg

// ############################## DHT22 Parameters
#define DHTPIN D8                                 //PIN connected to the DHT222 sensor
#define DHTTYPE DHT22                             //Type of DHT's family (AM2302)

// ############################## MOI Parameters
const int MOISPIN = A0;                           //PIN connected to the Moisture sensor
const int AirValue = 680;                         //Value with the sensor measuring just air
const int WaterValue = 310;                       //Value with the sensor measuring just water
                                                  //Follow below some approximate moisture levels for the sensor readings:
                                                  //000 to 300 => Dyr Soil    - CHECKKKKK THIS OUT
                                                  //300 to 700 => Humid Soil
                                                  //700 to 950 => Wet Soil

// ############################## RELAYs Parameters
int LED_1_PIN = D4;                               //PIN connected to Relay to command LED_1 (central LED)
int LED_4_PIN = D5;                               //PIN connected to Relay to command LED_4 (periferial LEDs)
int FAN_I_PIN = D6;                               //PIN connected to Relay to command FAN_I (EXT to INT air flow)
int FAN_O_PIN = D7;                               //PIN connected to Relay to command FAN_O (INT to EXT air flow)

// ############################## LEDs Parameters
bool LED_1_stat = 0;                              //LED_1's relay status (0=OFF | 1=ON - LEDs ON)
bool LED_4_stat = 0;                              //LED_4's relay status (0=OFF | 1=ON - LEDs ON)

// ############################## FANs Parameters
bool FAN_I_stat = 0;                              //FAN_I's relay status (0=OFF | 1=ON - FAN ON)
bool FAN_O_stat = 0;                              //FAN_O's relay status (0=OFF | 1=ON - FAN ON)

// ############################## TEM Parameters
float tem_raw = 0;                                //Will store the temperature value comming from sensor
float tem_mem = 0;                                //Will store the memorized temperature value for averaging
float tem_vec_mem[5] = {0, 0, 0, 0, 0};           //Array for average calculations (memory of 5 previous values)
int tem_cor = 0;                                  //Will store the correction temperature value if needed
int tem_des = 26;                                 //Desired temperature, wich will define the aim temperature of the system
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
int hum_des = 70;                                 //Desired humidity, wich will define the aim humidity of the system
int hum_del = 1;                                  //Detla humidity allowed, which will establish upper and lower thresholds
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
int moi_des = 65;                                 //Desired moisture, wich will define the aim moisture of the system
int moi_del = 1;                                  //Detla moisture allowed, which will establish upper and lower thresholds
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
bool reset_LED_1_relay = 0;                       //Flag for LED_1's relay reset - if received, calls reset_relay('l1');
bool reset_LED_4_relay = 0;                       //Flag for LED_4's relay reset - if received, calls reset_relay('l4');
bool reset_FAN_I_relay = 0;                       //Flag for FAN_I's relay reset - if received, calls reset_relay('if');
bool reset_FAN_O_relay = 0;                       //Flag for FAN_O's relay reset - if received, calls reset_relay('of');
bool init_param = 1;                              //Flag for system initialization - change value after initialization status and varibles, or restar
int illumin_stat = 0;                             //Code for follow up the status of the illumination - if received, calls set_illumination();
int airflow_stat = 0;                             //Code for follow up the status of the air flow - if received, calls set_airflow();

// ############################## Miscellaneous Parameters
float rssi_val_raw = 0;                           //Will store the RSSI measuremente form ESP8266
float value_mea = 0;                              //Value of the measurement to be printed. Sent by reference to print_measurements() auxiliar function
float value_mem = 0;                              //Averaged value of the measurement to be printed. Sent by reference to print_measurements() auxiliar function
int value_cor = 0;                                //Correction value of the measurement to be printed. Sent by reference to print_measurements() auxiliar function
char code = 'u';                                  //Printing code (t=temperarutr | h=humidity | m=moisture | a=after | b=before) sent to auxiliar functions
char illc = 'u';                                  //Illumination code (l=LOW | m=MID | h=HIGH | x=OFF) sent/received via MQTT to choose the level of illumination in the setup
char airc = 'u';                                  //Air Flow code (i=INPUT | o=OUTPUT | b=BOTH | x=OFF) sent/received via MQTT to choose the air flow inside the setup
bool mea_ini = 1;                                 //Flag for init memory logic (fill with the first value the memory array) sent to mem_logic() auxiliar function

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
void update_WeMos(String inputmsg);               //FUNCTION OK - IMPROVE PARAMETERS TO UPDATE
void tem_logic();                                 //FUNCTION OK
void hum_logic();                                 //FUNCTION OK
void moi_logic();                                 //FUNCTION OK
void set_illumination(char illc);                 //FUNCTION OK
void set_airflow(char airc);                      //FUNCTION OK
void publish_readings();                          //FUNCTION OK
void print_measurements(float value_mea, float value_mem, int value_cor, char code);//FUNCTION OK
void print_parameters(char code);                 //FUNCTION OK                 
float mem_logic(float value_mea, bool init_mem, char code); //FUNCTION OK
void reset_relay(char code);                      //FUNCTION OK                     
void reset_sys();                                 //FUNCTION OK                     

/* ################################################################################################### */
/* ################################################################################################### */
// ############################## SETUP FUNCTION - Setup main functions, units and variables
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
}

/* ################################################################################################### */
/* ################################################################################################### */
// ############################## MAIN FUNCTION - Forever LOOP
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
  publish_readings();                             //Publish the WeMos readings to the topic
  delay (10000);                                  //Delay 10 sec
}

/* ################################################################################################### */
/* ################################################################################################### */
// ############################## AUX FUNCTIONS - Definition of all functions of the code
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
  pinMode(D4, OUTPUT);                            //PIN GPIO04 => LED_1's relay
  pinMode(D5, OUTPUT);                            //PIN GPIO14 => LED_4's relay
  pinMode(D6, OUTPUT);                            //PIN GPIO12 => FAN_I's relay
  pinMode(D7, OUTPUT);                            //PIN GPIO13 => FAN_O's relay
  pinMode(D8, INPUT);                             //PIN GPIO00 => DHT22's input PIN
  pinMode(A0, INPUT);                             //PIN ANALOG => MOISTURE's input PIN
  Serial.println("PINOUT Settings Done!");

  Serial.println("Initializing Relay_1's Quick Test...");
  digitalWrite(D4, LOW);                          //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_1 ON
  LED_1_stat = 1;                                 //Update status variable
  delay (5000);                                   //Delay code execution
  digitalWrite(D4, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_1 OFF
  LED_1_stat = 0;                                 //Update status variable
  Serial.println("Relay's Quick Test Done!");

  Serial.println("Initializing Relay_2's Quick Test...");
  digitalWrite(D5, LOW);                          //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_4 ON
  LED_4_stat = 1;                                 //Update status variable
  delay (5000);                                   //Delay code execution
  digitalWrite(D5, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_4 OFF
  LED_4_stat = 0;                                 //Update status variable
  Serial.println("Relay's Quick Test Done!");

  Serial.println("Initializing Relay_3's Quick Test...");
  digitalWrite(D6, LOW);                          //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_I ON
  FAN_I_stat = 1;                                 //Update status variable
  delay (5000);                                   //Delay code execution
  digitalWrite(D6, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_I OFF
  FAN_I_stat = 0;                                 //Update status variable
  Serial.println("Relay's Quick Test Done!");

  Serial.println("Initializing Relay_4's Quick Test...");
  digitalWrite(D7, LOW);                          //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_O ON
  FAN_O_stat = 1;                                 //Update status variable
  delay (5000);                                   //Delay code execution
  digitalWrite(D7, HIGH);                         //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_O OFF
  FAN_O_stat = 0;                                 //Update status variable
  Serial.println("Relay's Quick Test Done!");
}

// ############################## AUX FUNCTIONS
void setup_sensors()
{
  Serial.println("Initializing Temperature Sensor...");
  delay (500);
  dht.begin();                                      //Initialize DHT22 sensor and send HUM & TEMP readings to serial port
  tem_raw = dht.readTemperature();              //Read the temperature value form the DHT22
  Serial.println("Temperature: ");
  Serial.println(tem_raw);                      //Print it to serial port
  Serial.println("°C");
  Serial.println("Temperature Sensor Done!");

  delay (500);
  Serial.println("Initializing Humidity Sensor...");
  hum_raw = dht.readHumidity();                 //Read the humidity value form the DHT22
  Serial.println("Humidity: ");
  Serial.println(hum_raw);                      //Print it to serial port
  Serial.println("%");
  Serial.println("Humidity Sensor Done!");

  Serial.println("Initializing Moisture Sensor...");
  delay (500);
  moi_unr = analogRead(A0);                   //Read the raw value of moisture of the soil 
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
  mqttClient.publish(topicWeMosReadings, "SW: IoT_WeMos_Punto_Rojo_Hum_Temp_Ctrl_v01"); //Publish SW Version for version control
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
    digitalWrite(LED_1_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_1 ON
    LED_1_stat = 1;                             //Update the value of the boolean variable to follow up the status
    digitalWrite(LED_4_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_4 OFF
    LED_4_stat = 0;                             //Update the value of the boolean variable to follow up the status
    illumin_stat = 1;                           //Set the code for initial illumination
    Serial.println("======================================== WeMos Initial Illumination Setup to LOW | End");

    Serial.println("======================================== WeMos Initial Air Flow Set to OUTPUT | Begin");
    digitalWrite(FAN_I_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_I OFF
    FAN_I_stat = 0;                             //Update the value of the boolean variable to follow up the status
    digitalWrite(FAN_O_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_O ON
    FAN_O_stat = 1;                             //Update the value of the boolean variable to follow up the status
    airflow_stat = 1;                           //Set the code for initial air flow
    Serial.println("======================================== WeMos Initial Air Flow Set to OUTPUT | End");

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
  StaticJsonBuffer<1024> JSONbufferRX;            //Set the RX JSON Buffer size
  JsonObject& JSONencoder = JSONbufferRX.createObject();

  Serial.println("======================================== WeMos Publish sensor's data (JSON) over MQTT topic | Begin");

  // Publish Temperature Values
  JSONencoder["TEMRAW"] = String (tem_raw);
  JSONencoder["TEMCOR"] = String (tem_cor);
  JSONencoder["TEMMEM"] = String (tem_mem - tem_cor);
  JSONencoder["TEMDES"] = String (tem_des);
  JSONencoder["TEMDEL"] = String (tem_del);
  JSONencoder["TEMUPL"] = String (tem_upl);
  JSONencoder["TEMLOL"] = String (tem_lol);
  JSONencoder["TEMALS"] = String (tem_ala_sta);
  JSONencoder["TEMALM"] = String (tem_ala_mem);
  JSONencoder["TEMALC"] = String (tem_ala_cou);

  // Publish Humidity Values
  JSONencoder["HUMRAW"] = String (hum_raw);
  JSONencoder["HUMCOR"] = String (hum_cor);
  JSONencoder["HUMMEM"] = String (hum_mem - hum_cor);
  JSONencoder["HUMDES"] = String (hum_des);
  JSONencoder["HUMDEL"] = String (hum_del);
  JSONencoder["HUMUPL"] = String (hum_upl);
  JSONencoder["HUMLOL"] = String (hum_lol);
  JSONencoder["HUMALS"] = String (hum_ala_sta);
  JSONencoder["HUMALM"] = String (hum_ala_mem);
  JSONencoder["HUMALC"] = String (hum_ala_cou);

  // Publish Moisture Values
  JSONencoder["MOIUNR"] = String (moi_unr);
  JSONencoder["MOIRAW"] = String (moi_raw);
  JSONencoder["MOICOR"] = String (moi_cor);
  JSONencoder["MOIMEM"] = String (moi_mem - moi_cor);
  JSONencoder["MOIDES"] = String (moi_des);
  JSONencoder["MOIDEL"] = String (moi_del);
  JSONencoder["MOIUPL"] = String (moi_upl);
  JSONencoder["MOILOL"] = String (moi_lol);
  JSONencoder["MOIALS"] = String (moi_ala_sta);
  JSONencoder["MOIALM"] = String (moi_ala_mem);
  JSONencoder["MOIALC"] = String (moi_ala_cou);

  // Publish Illumination Status
  JSONencoder["ILLSTA"] = String (illumin_stat);

  // Publish Air Flow Status
  JSONencoder["AIRSTA"] = String (airflow_stat);

  // Publish Relay Status
  JSONencoder["LED1ST"] = String (LED_1_stat);
  JSONencoder["LED4ST"] = String (LED_4_stat);
  JSONencoder["FANIST"] = String (FAN_I_stat);
  JSONencoder["FANOST"] = String (FAN_O_stat);

  // Publish Sensor Status
  JSONencoder["TEMSES"] = String (tem_sen_sta);
  JSONencoder["TEMSEM"] = String (tem_sen_mem);
  JSONencoder["TEMSEC"] = String (tem_sen_cou);
  JSONencoder["HUMSES"] = String (hum_sen_sta);
  JSONencoder["HUMSEM"] = String (hum_sen_mem);
  JSONencoder["HUMSEC"] = String (hum_sen_cou);
  JSONencoder["MOISES"] = String (moi_sen_sta);
  JSONencoder["MOISEM"] = String (moi_sen_mem);
  JSONencoder["MOISEC"] = String (moi_sen_cou);

  // Publish Miscelaneous Values
  JSONencoder["RSSIWI"] = String (WiFi.RSSI());

  char JSONmessageBuffer[1024];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.print("Sending message to MQTT topic...");
  Serial.println(topicWeMosReadings);
  Serial.println(JSONmessageBuffer);

  if (mqttClient.publish(topicWeMosReadings, JSONmessageBuffer) == true)
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

  StaticJsonBuffer<1024> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(inputmsg);
  if (!root.success())
  {
    Serial.println("parseObject() failed");
  }
  else
  {
    // SYS & RELAY RESET RECEIVED CHECK
    if (inputmsg.indexOf("SYS_RESET", 1) >= 0)
    {
      reset_SYS = root["SYS_RESET"];
      if (reset_SYS)
      {
        reset_sys();                                //Call the function reset_sys() for SW/HW reinitialization
      }
    }
    if (inputmsg.indexOf("LD1_RESET", 1) >= 0)
    {
      reset_LED_1_relay = root["LD1_RESET"];
      if (reset_LED_1_relay)
      {
        reset_relay('1');                           //Call the function reset_relay() to reset the relay passed as argument
      }
    }
    if (inputmsg.indexOf("LD4_RESET", 1) >= 0)
    {
      reset_LED_4_relay = root["LD1_RESET"];
      if (reset_LED_4_relay)
      {
        reset_relay('4');                           //Call the function reset_relay() to reset the relay passed as argument
      }
    }
    if (inputmsg.indexOf("FNI_RESET", 1) >= 0)
    {
      reset_FAN_I_relay = root["FNI_RESET"];
      if (reset_FAN_I_relay)
      {
        reset_relay('i');                           //Call the function reset_relay() to reset the relay passed as argument
      }
    }
    if (inputmsg.indexOf("FNO_RESET", 1) >= 0)
    {
      reset_FAN_O_relay = root["FNO_RESET"];
      if (reset_FAN_O_relay)
      {
        reset_relay('o');                           //Call the function reset_relay() to reset the relay passed as argument
      }
    }
    
    // ILLUMINATION SETUP CODE RECEIVED CHECK
    if (inputmsg.indexOf("ILLSTA", 1) >= 0)
    {
      illumin_stat = root["ILLSTA"];
      if ((illumin_stat == 0) || (illumin_stat == 1) || (illumin_stat == 2) || (illumin_stat == 3))
      {
        switch (illumin_stat)
        {
          case 0:
            set_illumination('x');
          break;

          case 1:
            set_illumination('l');
          break;

          case 2:
            set_illumination('m');
          break;

          case 3:
            set_illumination('h');
          break;
        }
      }
    }

    // AIR FLOW SETUP CODE RECEIVED CHECK
    if (inputmsg.indexOf("AIRSTA", 1) >= 0)
    {
      airflow_stat = root["AIRSTA"];
      if ((airflow_stat == 0) || (airflow_stat == 1) || (airflow_stat == 2) || (airflow_stat == 3))
      {
        switch (airflow_stat)
        {
          case 0:
            set_airflow('x');
          break;

          case 1:
            set_airflow('i');
          break;

          case 2:
            set_airflow('o');
          break;

          case 3:
            set_airflow('b');
          break;
        }
      }
    }

    // TEM | HUM | MOI SETUP CODE RECEIVED CHECK
    if (inputmsg.indexOf("TEMDES", 1) >= 0)
    {
      tem_des = root["TEMDES"];
      tem_upl = tem_des + tem_del;
      tem_lol = tem_des - tem_del;
    }
    if (inputmsg.indexOf("TEMDEL", 1) >= 0)
    {
      tem_del = root["TEMDEL"];
      tem_upl = tem_des + tem_del;
      tem_lol = tem_des - tem_del;
    }
    if (inputmsg.indexOf("TEMCOR", 1) >= 0)
    {
      tem_cor = root["TEMCOR"];
    }
  
    if (inputmsg.indexOf("HUMDES", 1) >= 0)
    {
      hum_des = root["HUMDES"];
      hum_upl = hum_des + hum_del;
      hum_lol = hum_des - hum_del;
    }
    if (inputmsg.indexOf("HUMDEL", 1) >= 0)
    {
      hum_del = root["HUMDEL"];
      hum_upl = hum_des + hum_del;
      hum_lol = hum_des - hum_del;
    }
    if (inputmsg.indexOf("HUMCOR", 1) >= 0)
    {
      hum_cor = root["HUMCOR"];
    }

    if (inputmsg.indexOf("MOIDES", 1) >= 0)
    {
      moi_des = root["MOIDES"];
      moi_upl = moi_des + moi_del;
      moi_lol = moi_des - moi_del;
    }
    if (inputmsg.indexOf("MOIDEL", 1) >= 0)
    {
      moi_del = root["MOIDEL"];
      moi_upl = moi_des + moi_del;
      moi_lol = moi_des - moi_del;
    }
    if (inputmsg.indexOf("MOICOR", 1) >= 0)
    {
      moi_cor = root["MOICOR"];
    }

  print_parameters('a');                          
  }
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
      Serial.print("Status of Central LED: \t\t");
      if (LED_1_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of Periferical LEDs: \t");
      if (LED_4_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of INPUT FAN: \t\t");
      if (FAN_I_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of OUTPUT FAN: \t\t");
      if (FAN_O_stat)
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
      Serial.print("Status of Central LED: \t\t");
      if (LED_1_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of Periferical LEDs: \t");
      if (LED_4_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of INPUT FAN: \t\t");
      if (FAN_I_stat)
        Serial.println("ON");
      else
        Serial.println("OFF");
      Serial.print("Status of OUTPUT FAN: \t\t");
      if (FAN_O_stat)
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
  if  ((isnan(tem_raw)) || (tem_raw==998) || (tem_raw==999))
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
    if ((tem_mem - tem_cor) < tem_lol)          //Corrected temperature below lower defined threshold
    {
      tem_ala_sta = 1;
      tem_ala_mem = 1;
      tem_ala_cou = tem_ala_cou + 1;
    }
    else
    {
      tem_ala_sta = 0;
    }
    
    if ((tem_mem - tem_cor) >= tem_upl)   //Corrected temperature above upper defined threshold
    {
      tem_ala_sta = 1;
      tem_ala_mem = 1;
      tem_ala_cou = tem_ala_cou + 1;
    }
    else
    {
      tem_ala_sta = 0;
    }
  }
}

// ############################## AUX FUNCTIONS
void hum_logic()
{
  hum_raw = dht.readHumidity();               //Read the humidity form the DHT22
  if  ((isnan(hum_raw)) || (hum_raw==998) || (hum_raw==999))
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
    if ((hum_mem - hum_cor) < hum_lol)    //Corrected humidity below lower defined threshold
    {
      hum_ala_sta = 1;
      hum_ala_mem = 1;
      hum_ala_cou = hum_ala_cou + 1;
    }
    else
    {
      hum_ala_sta = 0;
    }
    
    if ((hum_mem - hum_cor) >= hum_upl)   //Corrected temperature above upper defined threshold
    {
      hum_ala_sta = 1;
      hum_ala_mem = 1;
      hum_ala_cou = hum_ala_cou + 1;
    }
    else
    {
      hum_ala_sta = 0;
    }
  }
}

// ############################## AUX FUNCTIONS
void moi_logic()
{
  moi_unr = analogRead(A0);                   //Read the moisture form the MOI
  moi_raw = map(moi_unr, AirValue, WaterValue, 0, 100);
  if  ((isnan(moi_raw)) || (moi_raw==998) || (moi_raw==999))
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
    moi_mem = mem_logic(moi_raw, moi_ini_mem, 'm'); //Call mem_logic() => Calculate average of last 5 previous values
    print_measurements(moi_raw, moi_mem, moi_cor, 'm' ); //Call print_measurements()
    if ((moi_mem - moi_cor) < moi_lol)    //Corrected moisture below lower defined threshold
    {
      moi_ala_sta = 1;
      moi_ala_mem = 1;
      moi_ala_cou = moi_ala_cou + 1;
    }
    else
    {
      moi_ala_sta = 0;
    }
    
    if ((moi_mem - moi_cor) >= moi_upl)   //Corrected moisture above upper defined threshold
    {
      moi_ala_sta = 1;
      moi_ala_mem = 1;
      moi_ala_cou = moi_ala_cou + 1;
    }
    else
    {
      moi_ala_sta = 0;
    }
  }
}

// ############################## AUX FUNCTIONS
void set_illumination(char illc)
{
  /* Received parameters values:
    code:
      "l" => LOW Illumination: Just the Central LED ON (one LED ON in total)
      "m" => MID Illumination: Just the Periferial LEDs ON (four LEDs ON in total)
      "h" => HIG Illumination: All LEDs ON (five LEDs ON in total)
      "x" => OFF Illumination: All LEDs OFF (five LEDs OFF in total)
  */
  switch (illc)
  {
    case 'l':
      if ((LED_1_stat == 1) && (LED_4_stat == 0))
      {
        Serial.println("Illumination already set to LOW");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to LOW | Begin");
        digitalWrite(LED_1_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_1 ON
        LED_1_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_4_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_4 OFF
        LED_4_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to LOW | End");
      }
      illumin_stat = 1;                             //Set the code for initial illumination
    break;

    case 'm':
      if ((LED_1_stat == 0) && (LED_4_stat == 1))
      {
        Serial.println("Illumination already set to MID");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to MID | Begin");
        digitalWrite(LED_1_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_1 OFF
        LED_1_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_4_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_4 ON
        LED_4_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to MID | End");
      }
      illumin_stat = 2;                             //Set the code for initial illumination
    break;

    case 'h':
      if ((LED_1_stat == 1) && (LED_4_stat == 1))
      {
        Serial.println("Illumination already set to HIGH");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to HIGH | Begin");
        digitalWrite(LED_1_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_1 ON
        LED_1_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_4_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_4 ON
        LED_4_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to HIGH | End");
      }
      illumin_stat = 3;                             //Set the code for initial illumination
    break;

    case 'x':
      if ((LED_1_stat == 0) && (LED_4_stat == 0))
      {
        Serial.println("Illumination already set to OFF");
      }
      else
      {
        Serial.println("======================================== WeMos Illumination Set to OFF | Begin");
        digitalWrite(LED_1_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_1 OFF
        LED_1_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(LED_4_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_4 OFF
        LED_4_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Illumination Set to OFF | End");
      }
      illumin_stat = 0;                             //Set the code for initial illumination
    break;

    defualt:
      Serial.println("Something went wrong. Check HW & code."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

// ############################## AUX FUNCTIONS
void set_airflow(char airc)
{
  /* Received parameters values:
    code:
      "i" => IPUT AIR FLOW: Just INPUT Air Flow (one FAN ON in total)
      "o" => OPUT AIR FLOW: Just OUTPUT Air Flow (one FAN ON in total)
      "b" => BOTH AIR FLOW: Both WAYs Air Flow (two FANs ON in total)
      "x" => OFF AIR FLOW: Air Flow turned OFF (all FANs OFF in total)
  */
  switch (airc)
  {
    case 'i':
      if ((FAN_I_stat == 1) && (FAN_O_stat == 0))
      {
        Serial.println("Air Flow already Set to INPUT");
      }
      else
      {
        Serial.println("======================================== WeMos Air Flow Set to INPUT | Begin");
        digitalWrite(FAN_I_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_I ON
        FAN_I_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(FAN_O_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_O OFF
        FAN_O_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Air Flow Set to INPUT | End");
      }
      airflow_stat = 1;                             //Set the code for initial air flow
    break;

    case 'o':
      if ((FAN_I_stat == 0) && (FAN_O_stat == 1))
      {
        Serial.println("Air Flow already Set to OUTPUT");
      }
      else
      {
        Serial.println("======================================== WeMos Air Flow Set to OUTPUT | Begin");
        digitalWrite(FAN_I_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_I OFF
        FAN_I_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(FAN_O_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_O ON
        FAN_O_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Air Flow Set to OUTPUT | End");
      }
      airflow_stat = 2;                             //Set the code for initial air flow
    break;

    case 'b':
      if ((FAN_I_stat == 1) && (FAN_O_stat == 1))
      {
        Serial.println("Air Flow already Set to BOTH (circulation)");
      }
      else
      {
        Serial.println("======================================== WeMos Air Flow Set to BOTH (circulation) | Begin");
        digitalWrite(FAN_I_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_I ON
        FAN_I_stat = 1;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(FAN_O_PIN, LOW);               //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_O ON
        FAN_O_stat = 1;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Air Flow Set to BOTH (circulation) | End");
      }
      airflow_stat = 3;                             //Set the code for initial air flow
    break;

    case 'x':
      if ((FAN_I_stat == 0) && (FAN_O_stat == 0))
      {
        Serial.println("Air Flow already Set to OFF");
      }
      else
      {
        Serial.println("======================================== WeMos Air Flow Set to OFF | Begin");
        digitalWrite(FAN_I_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_I OFF
        FAN_I_stat = 0;                             //Update the value of the boolean variable to follow up the status
        digitalWrite(FAN_O_PIN, HIGH);              //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_O OFF
        FAN_O_stat = 0;                             //Update the value of the boolean variable to follow up the status
        Serial.println("======================================== WeMos Air Flow Set to OFF | End");
      }
      airflow_stat = 0;                             //Set the code for initial air flow
    break;

    defualt:
      Serial.println("Something went wrong. Check HW & code."); //Print the auxiliar text to Serial Comm (USB)
    break;
  }
}

// ############################## AUX FUNCTIONS
void reset_relay(char code)
{
  /* Received parameters values:
    code:
      "1" => Reset the LED_1 Relay - Central LED
      "4" => Reset the LED_4 Relay - Periferial LEDs
      "i" => Reset the FAN_I Relay - INPUT FAN
      "O" => Reset the FAN_O Relay - OUTPUT FAN
  */
  switch (code)
  {
    case '1':
      Serial.println("CENTRAL LED RESET request received - Resetting Realy in 3sec...");
      delay(1000);
      Serial.println("CENTRAL LED RESET request received - Resetting Realy in 2sec...");
      delay(1000);
      Serial.println("CENTRAL LED RESET request received - Resetting Realy in 1sec...");
      delay(1000);

      Serial.println("======================================== WeMos CENTRAL LED's relay reset | Begin");
      if (LED_1_stat == 1)
      {
        digitalWrite(LED_1_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_1 ON
        LED_1_stat = 0;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(LED_1_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_1 OFF
        LED_1_stat = 1;                           //Update the value of the boolean variable to follow up the status
      }

      if (LED_1_stat == 0)
      {
        digitalWrite(LED_1_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_1 OFF
        LED_1_stat = 1;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(LED_1_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_1 ON
        LED_1_stat = 0;                           //Update the value of the boolean variable to follow up the status
      }
      Serial.println("======================================== WeMos CENTRAL LED's relay reset | End");
    break;
    
    case '4':
      Serial.println("PERIFERIAL LEDs RESET request received - Resetting Realy in 3sec...");
      delay(1000);
      Serial.println("PERIFERIAL LEDs RESET request received - Resetting Realy in 2sec...");
      delay(1000);
      Serial.println("PERIFERIAL LEDs RESET request received - Resetting Realy in 1sec...");
      delay(1000);

      Serial.println("======================================== WeMos PERIFERIAL LED's relay reset | Begin");
      if (LED_4_stat == 1)
      {
        digitalWrite(LED_4_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_4 ON
        LED_4_stat = 0;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(LED_4_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_4 OFF
        LED_4_stat = 1;                           //Update the value of the boolean variable to follow up the status
      }

      if (LED_4_stat == 0)
      {
        digitalWrite(LED_4_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - LED_4 OFF
        LED_4_stat = 1;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(LED_4_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - LED_4 ON
        LED_4_stat = 0;                           //Update the value of the boolean variable to follow up the status
      }
      Serial.println("======================================== WeMos PERIFERIAL LED's relay reset | End");
    break;

    case 'i':
      Serial.println("INPUT AIR FLOW RELAY RESET request received - Resetting Realy in 3sec...");
      delay(1000);
      Serial.println("INPUT AIR FLOW RELAY RESET request received - Resetting Realy in 2sec...");
      delay(1000);
      Serial.println("INPUT AIR FLOW RELAY RESET request received - Resetting Realy in 1sec...");
      delay(1000);

      Serial.println("======================================== WeMos INPUT FAN's relay reset | Begin");
      if (FAN_I_stat == 1)
      {
        digitalWrite(FAN_I_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_I_PIN ON
        FAN_I_stat = 0;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(FAN_I_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_I_PIN OFF
        FAN_I_stat = 1;                           //Update the value of the boolean variable to follow up the status
      }

      if (FAN_I_stat == 0)
      {
        digitalWrite(FAN_I_PIN, HIGH);            //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_I_PIN OFF
        FAN_I_stat = 1;                           //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(FAN_I_PIN, LOW);             //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_I_PIN ON
        FAN_I_stat = 0;                           //Update the value of the boolean variable to follow up the status
      }
      Serial.println("======================================== WeMos INPUT FAN LED's relay reset | End");
    break;

    case 'o':
      Serial.println("OUTPUT AIR FLOW RELAY RESET request received - Resetting Realy in 3sec...");
      delay(1000);
      Serial.println("OUTPUT AIR FLOW RELAY RESET request received - Resetting Realy in 2sec...");
      delay(1000);
      Serial.println("OUTPUT AIR FLOW RELAY RESET request received - Resetting Realy in 1sec...");
      delay(1000);

      Serial.println("======================================== WeMos OUTPUT FAN's relay reset | Begin");
      if (FAN_O_stat == 1)
      {
        digitalWrite(FAN_O_PIN, LOW);            //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_O_PIN ON
        FAN_O_stat = 0;                          //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(FAN_O_PIN, HIGH);           //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_O_PIN OFF
        FAN_O_stat = 1;                          //Update the value of the boolean variable to follow up the status
      }

      if (FAN_O_stat == 0)
      {
        digitalWrite(FAN_O_PIN, HIGH);           //Write HIGH (3V) on the DIGITAL PIN (realy deactivated) - FAN_O_PIN OFF
        FAN_O_stat = 1;                          //Update the value of the boolean variable to follow up the status
        delay(10000);                             //Dealy 10 sec
        digitalWrite(FAN_O_PIN, LOW);            //Write LOW (0V) on the DIGITAL PIN (realy activated) - FAN_O_PIN ON
        FAN_O_stat = 0;                          //Update the value of the boolean variable to follow up the status
      }
      Serial.println("======================================== WeMos OUTPUT FAN LED's relay reset | End");
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
