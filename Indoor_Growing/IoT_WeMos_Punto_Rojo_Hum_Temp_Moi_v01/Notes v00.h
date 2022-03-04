// ############################## IoT Project to Pub/Sub measurements via MQTT Broker CloudMQTT 

/* ############################## Version Index
IoT_WeMos_Punto_Rojo_Hum_Temp_Ctrl_v00
  => Initial version
IoT_WeMos_Punto_Rojo_Hum_Temp_Ctrl_v01 
  => Security version (Larika almost deletes all the code, so just in case...).
Larika DID something, I had to reconfigure completly the Arduino IDE and VS Code. I don't know
what she did, but I have to take more care about it.
IoT_WeMos_Punto_Rojo_Hum_Temp_Ctrl_v02
  => Added functionality to clear the system's alarm memory and sensor's failure memory
*/

/* ############################## Project's Description
The device (WeMos D1 R1 chipset with ESP8266 onborad) will report:
- Temperature (°C)
- Humidity (%)
- Moisture (%)
- PH of Soil (#) - Future Use
- Open/Close Door (On/Off) - Future Use
Along other measurements, every 10 sec via CloudMQTT Broker (using mqtt protocol)
At the same time, WeMos will accept commands form MQTT Broker (Publish/subscribe)  */

/* ############################## Bill of Materials
WeMos D1 R1         1 => Chipset with ESP8266 onboard
DHT22 Sensor        1 => Temperature and Humidity sensor
Moisture Sensor     1 => Used to measure the Moisture of the soil
PH Sensor           1 => Used to measure the PH of the soil - Future Use
Open/Close Sensor   1 => Used to measure the Door's status - Future Use
Relay               4 => Relay to control the LEDs and FANs
DB9 Female          2 => Used for wiring WeMos to SENSORs and RELAYs
DB9 Male            2 => Used for wiring SENSORs and RELAYs to WeMos
UTP Cable           2 => Used for wiring */

/* ############################## Libraires used on the Project
#include <ESP8266WiFi.h>    => On Arduino IDE: Tools=>Manage Libraries=>Adafruit ESP8266
#include <PubSubClient.h>   => On Arduino IDE: Tools=>Manage Libraries=>MQTTPubSubClient
#include <ArduinoJson.h>    => On Arduino IDE: Tools=>Manage Libraries=>ArduinoJson
#include <DHT.h>            => On Arduino IDE: Tools=>Manage Libraries=>DHT sensor library */

/* ############################## Relay's LOGIC
All relays will be set using the Normal Open logigc. This means that if there is a lack of energy
the LEDs and FAN will be automatically turned off.
So, the initail status of all relays will be NO

LED_1   =>  Relay controling the central LED of the Indoor Setup  
LED_4   =>  Relay controling the periferial LEDs of the Indor Seput
FAN_I   =>  Relay controling the FAN for INPUT air flow into the Indoor Setup
FAN_O   =>  Relay controling the FAN for OUTPUT air flow of the Indoor Setup

This two LED's relays will allow three different levels of illumination inside the Indoor Setup
LOW => LED_1 relay activated: Only the central LED (one in total) will be turned ON
MID => LED_4 realy activated: Only the periferial LEDS (four in total) will be turned ON
HIG => LED_1 && LED_4 relays activated: All LEDs will be turned ON (five in total)

On the other hand, the two FAN's relay will allow to completly control the INPUT and OUTPUT air flow:
IN      => FAN_I relay activated: Only INPUT FAN turned ON
OU      => FAN_O relay activated: Only OUTPUT FAN turned ON
IN-OU   => FAN_I && FAN_O realys activated: Total air cirulation (IN and OUT)

If digitalWrite(PIN, HIGH)  => Writes 3V to PIN => Relay Deactivated => Original NO/NC status.
If digitalWrite(PIN, LOW)   => Writes 0V to PIN => Relay Activated => Inverted NO/NC status
*/

/* ############################## Wiring
The wiring will be done using 2 (two) DB9 Serial connectors and UTP LAN cables (8 wires at total).
So, viewing the Female DB9 Serial Connector intalled On the Board of WemosD1R1...

-------------------
\  5  4  3  2  1 /
 \  9  8  7  6  /
  --------------
PIN     UTP Cable colour
1       Blue
6       White/Blue
2       Brown
7       White/Brown
4       Orange
8       White/Orange
5       Green
9       White/Green

SENSOR's Female DB9 Serial Connector pinout distribution (Updated 20210626):
1   =>  +3.3V                                           => +3.3 Vcc
6   =>  GND                                             => GND
2   =>  DHT22 Data                                      => D8 (GPIO00)
7   =>  MOIST Data                                      => A0 (Analog Input)
4   =>  PH Data (future use)                            => D9 (GPIO02)            - Future use - Not connected
8   =>  Door Data (future use)                          => D10 (GPIO15)           - Future use - Not connected
5   =>  +5.0V                                           => +5.0 Vcc               - Future use - Not connected
9   =>  GND                                             => GND                    - Future use - Not connected
3   =>  Not Connected (UTP cable has only 8 wires)      => Not connected          - Not connected

RELAY's Female DB9 Serial Connector pinout distribution:
1   =>  +3.3V                                           => +3.3 Vcc
6   =>  GND                                             => GND
2   =>  Control Relay_1 (LED_1)                         => D4 (GPIO04)
7   =>  Control Relay_2 (LED_4)                         => D5 (GPIO14)
4   =>  Control Relay_3 (FAN_I)                         => D6 (GPIO12)
8   =>  Control Relay_4 (FAN_O)                         => D7 (GPIO13)
5   =>  +5.0V                                           => +5.0 Vcc               - Future use - Not connected
9   =>  GND                                             => GND                    - Future use - Not connected
3   =>  Not Connected (UTP cable has only 8 wires)      => Not connected          - Not connected
*/

/* ############################## Calibrating Sensors
DHT22   =>  It is not necessary to calibrate this sensor (already calbirated in factory)
MOI     =>  Even that in some sites I found that it is recommended to feed the sensor with 5v, the current project considers 3.3v.
            The capacitive sensor needs to be calibrated with values of air (considered "dry") and water. This values
            will be "reference" to the function map(), as below:
            moi_raw = map(moi_unr, AirValue, WaterValue, 0, 100);
            moi_raw will be the value of the soil's moisture as a percentage, published on MQTT.
            As 20210629, cloudy day here in Salvador and very high ambient humidity (70%) I've measured:
            AirValue = 664 (set the value to 680 in order to adjust the humidity of the air)
            Water Value = 328 (set the value to 310 in order to adjust how much wet is the soil)
*/  

/* ############################## Cloud MQTT - MQTT Broker
Will be used the CloudMQTT Broker: https://www.cloudmqtt.com/ using a free "Cute Cat" plan.
Note: Plan Cute Cat is no longer available and is replaced by $5 Humble Hedgehog. For more information please read our announcement.
Consider to change MQTT Broker (even it is still working, I can't edit/change anything)
User: alvarova26@hotmail.com
Pass: Soyyoal26;
Instance: WeMos
Parameters defined on the Broker side
Server:                         m16.cloudmqtt.com
Region:                         amazon-web-services::us-east-1
User:                           oepdtofo
Password:                       hcJjL0FbT40U
Port:                           17879
SSL Port:                       27879
Websockets Port (TLS only):     37879
Connection limit:               5 */

/* ############################## MQTT Panel - Android App
System Status------------------------------------------------ Layout Decorator
  Cuerrent Measurements-------------------------------------- Layout Decorator
    Temp.---------------------------------------------------- Gauge
      Topic: WeMosReadings
      Payload: $.TEMMEM
    Humidity------------------------------------------------- Gauge
      Topic: WeMosReadings
      Payload: $.HUMMEM
    Moisture------------------------------------------------- Gauge
      Topic: WeMosReadings
      Payload: $.MOIMEM
  
contunar aca
*/

/* ############################## Note: Update header of PubSubClient.h
MQTT_MAX_PACKET_SIZE defined in the library header MUST BE INCREASED to allow larger payloads in client.publish() method
C:\Users\Alvaro\Documents\Arduino\libraries\PubSubClient\src => PubSubClient.h */

/* ############################## Note: WeMos D1 R1 is different to R2
WEMOS D1 R1 is different to D1 R2 => There are some PINOUT differneces.
Check out: https://tasmota.github.io/docs/devices/Wemos-D1-R1-%26-R2/
Pinout map: https://www.google.com/search?q=wemos+d1+r1+pinout&client=firefox-b-d&sxsrf=ALeKk00jsstp66rOYZiH3N_W7hX0ctlsFA:1624199572176&tbm=isch&source=iu&ictx=1&fir=jzXSHfwEPzOfHM%252C4r1yq4sX81jpWM%252C_&vet=1&usg=AI4_-kT9wKVX0ORAHmIIyZ1qneYdeOUBuA&sa=X&ved=2ahUKEwj8sKz0tqbxAhXEr5UCHUgeDEcQ9QF6BAgDEAE#imgrc=jzXSHfwEPzOfHM
Take a look at the schematics for more information. */

/* ############################## Note: PINOUT correspondence Arduino/WeMos/ESP8266
//Pin       Function			ESP8266
//D0        RXD				    GPIO3
//D1		TXD					GPIO1
//D2		IO					GPI016
//D3/D15	IO, SCL				GPI05
//D4/D14	IO, SDA				GPIO4
//D5/D13    IO, SCK				GPIO14
//D6/D12	IO, MISO			GPIO12
//D7/D11	IO, MOSI			GPIO13
//D8		IO, Pull-Up		    GPIO0
//D9		IO, Pull-Up, LED	GPIO2
//D10		IO, Pull-Up, SS		GPIO15
//A0		Analog input		A0
//5V		5V					-
//3V3		3.3V				3.3V
//G			Ground				GND
//RST		Reset				RST
######### Be aware that there are only 9 GPIOxx (D2 trough D10) availables for using!!!!!!
One of them is broken and not working at all => D2=GPIO16 */ 

/* Testing GPIOxx of WeMos as INPUT from DHT22 (20210622 23:00)
#define D2    16    //GPIO16    => Not working as INPUT from DHT22
#define D3    5     //GPIO05    => OK as INPUT from DHT22
#define D4    4     //GPIO04    => OK as INPUT from DHT22
#define D5    14    //GPIO14    => OK as INPUT from DHT22
#define D6    12    //GPIO12    => OK as INPUT from DHT22
#define D7    13    //GPIO13    => OK as INPUT from DHT22
#define D8    0     //GPIO00    => OK as INPUT from DHT22
#define D9    2     //GPIO02    => OK as INPUT from DHT22
#define D10   15    //GPIO15    => OK as INPUT from DHT22 */

/* Testing A0 (Analogial Input) of WeMos as INPUT from MOI (20210622 23:00)
#define A0    A0    //ANALOG    => OK as INPUT from MOI */

/* Testing GPIOxx of WeMos as OUTPUT to control the RELAY (20210622 23:00)
#define D2    16    //GPIO16    => Not working as OUTPUT to control RELAY
#define D3    5     //GPIO05    => OK as OUTPUT to control RELAY
#define D4    4     //GPIO04    => OK as OUTPUT to control RELAY
#define D5    14    //GPIO14    => OK as OUTPUT to control RELAY
#define D6    12    //GPIO12    => OK as OUTPUT to control RELAY
#define D7    13    //GPIO13    => OK as OUTPUT to control RELAY
#define D8    0     //GPIO00    => OK as OUTPUT to control RELAY
#define D9    2     //GPIO02    => OK as OUTPUT to control RELAY
#define D10   15    //GPIO15    => OK as OUTPUT to control RELAY */

