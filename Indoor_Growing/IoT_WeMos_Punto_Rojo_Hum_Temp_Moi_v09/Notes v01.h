// ############################## IoT Project to Pub/Sub measurements via MQTT Broker CloudMQTT from WeMos D1 R1

/* ############################## Version Index
IoT_WeMos_Punto_Rojo_Hum_Temp_Moi_v00
  => Initial version
IoT_WeMos_Punto_Rojo_Hum_Temp_Moi_v01 
  => Security version (Larika almost deletes all the code, so just in case...).
Larika DID something, I had to reconfigure completly the Arduino IDE and VS Code. I don't know
what she did, but I have to take more care about it.
IoT_WeMos_Punto_Rojo_Hum_Temp_Moi_v02
  => Added functionality to clear the system's alarm memory and sensor's failure memory
IoT_WeMos_Punto_Rojo_Hum_Temp_Moi_v03
  => Correction in the tem/hum/moi_logic functions
IoT_WeMos_Punto_Rojo_Hum_Temp_Moi_v04
  => New approach in order to reduce the heat inside the IGS
IoT_WeMos_Punto_Rojo_Hum_Temp_Moi_v05
  => New functionality to alternate illumination's status
IoT_WeMos_Punto_Rojo_Hum_Temp_Moi_v06
  => HW/SW problem detecte - Some change in order to address the problem (lib/code/etc.).
IoT_WeMos_Punto_Rojo_Hum_Temp_Moi_v07
  => Code developed to use the version 6.x.x of ArduinoJson library.
IoT_WeMos_Punto_Rojo_Hum_Temp_Moi_v08
  => Code improved to keep the running time (with Chrono library and the millis() function).
  => Actually, just for running time  millis() is used. Chrono could be used in future applications.
  => Improving the code to allow the cental LED turn ON and OFF automatcially (refer to code updated 20220630)
IoT_WeMos_Punto_Rojo_Hum_Temp_Moi_v09
  => Improving the code to allow ON/OFF alternation and rotation of LEDs (refer to code updated 20220703)
  => Imroveing strategy with new functions (check_illum(), check_onoff(), check_rot(), etc).
*/

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

/* ############################## Bill of Materials
WeMos D1 R1         1 => Chipset with ESP8266 onboard
DHT22 Sensor        1 => Temperature and Humidity sensor
Moisture Sensor     1 => Used to measure the Moisture of the soil
PH Sensor           1 => Used to measure the PH of the soil - Future Use
Open/Close Sensor   1 => Used to measure the Door's status - Future Use
Relay               4 => Relay to control the LEDs and FANs
DB9 Female          2 => Used for wiring WeMos to SENSORs and RELAYs
DB9 Male            2 => Used for wiring SENSORs and RELAYs to WeMos
UTP Cable           2 => Used for wiring.
*/

/* ############################## Arduino IDE Configuration for WeMos ESP8266
Some tutorial for IDE configuration...
https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/
https://www.instructables.com/Steps-to-Setup-Arduino-IDE-for-NODEMCU-ESP8266-WiF/
https://blogmasterwalkershop.com.br/embarcados/wemos/wemos-d1-configurando-a-ide-do-arduino

Configurations:

Files=>Preferences: http://arduino.esp8266.com/stable/package_esp8266com_index.json
Tools=>Board=>Boards Manager... look up for "esp8266 by ESP8266 Community" => Install.
Tools=>Board=>ESP8266 Boards (3.0.1)... look up for LOLIN(WeMos) D1 R1.
Tools=>Manage Libraries... look up for "PubSubClient by Nick O'Leary".
  * Version as today(20210706): v2.8.0.
Tools=>Manage Libraries... look up for "ArduinoJson by Benoit Blanchon".
  * Must be very carefull chosing the version of the library. The current version of the code
    uses v6.x.x - has been updated since v07.
    Older version of this code use the older version of the library - changes completely!!!
    Be very carefull.
Tools=>Manage Libraries... look up for "DHT sensor library by Adafruit".
  * Version as today(20210706): v1.4.2.
    This library needs another one: Adafruit Unified Sensor (install it as well)
Tools=>Manage Libraries... look up for "Chrono by Thomas O Freder...".
  * Version as today(20210706): v1.1.3.

Note: ESP8266WiFi.h is installed along many others libraries when the esp8266 board is installed.
C:\Users\Alvaro\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\3.0.1\libraries\ESP8266WiFi

The Arduino IDE stores the manually installed boards in (many many libraries):
C:\Users\Alvaro\AppData\Local\Arduino15\packages\

The Arduino IDE stores the manually installed libraries in:
C:\Users\Alvaro\Documents\Arduino\libraries\
*/

/* ############################## Libraires used on the Project
#include <ESP8266WiFi.h>    => Installed along many others when the manually install the Board ESP8266
#include <PubSubClient.h>   => On Arduino IDE: Tools=>Manage Libraries=>PubSubClient by Nick O'Leary
#include <ArduinoJson.h>    => On Arduino IDE: Tools=>Manage Libraries=>ArduinoJson by Benoit Blanchon
#include <DHT.h>            => On Arduino IDE: Tools=>Manage Libraries=>DHT sensor library by Adafruit.
*/

/* ############################## Relay's LOGIC
All relays will be set using the Normal Open logigc. This means that if there is a lack of energy
the LEDs and FANs will be automatically turned off.
So, the initail status of all relays will be NO

LED_A   =>  Relay controling the central LED of the Indoor Setup
 ______
|.    .|
|  x   |
|.____.|

LED_B   =>  Relay controling 2 periferial LEDs of the Indoor Seput
 ______
|x    .|
|  .   |
|.____x|

LED_C   =>  Relay controling the remaining 2 periferial LEDs of the Indoor Seput
 ______
|.    x|
|  .   |
|x____.|

FAN_A   =>  Relay controling the FANs for INPUT and OUTPUT air flow of the Indoor Setup

This three LED's relays will allow seven different configurations of illumination:
 ______  ______  ______  ______  ______  ______  ______ 
|.    .||x    .||.    x||x    .||.    x||x    x||x    x|
|  x   ||  .   ||  .   ||  x   ||  x   ||  .   ||  x   |
|.____.||.____x||x____.||.____x||x____.||x____x||x____x|
  VLOW    LOWA    LOWB    MIDA    MIDB    HIGH    VHIG
* An additional configuration has been added: ON/OFF operation - to allow the LEDs to be turned
on and off autmatically.

And five different levels of illumination:
Very Low  =>  LED_A's relay activated
Low (x2)  =>  LED_B's or LED_C's relay activated
Mid (x2)  =>  LED_A's and (LED_B's or LED_C's) relays activated
High      =>  LED_B's and LED_C's relays activated
Very Hig  =>  LED_A's and LED_B's and LED_C's relays activated
* ONOFF   =>  LED_A's relay alternating the status ON/OFF

On the other hand, the only one remaining relay will control the FANs for air circulation
Circulation   => FAN_A's realys activated

If digitalWrite(PIN, HIGH)  => Writes 3V to PIN => Relay Deactivated => Original NO/NC status.
If digitalWrite(PIN, LOW)   => Writes 0V to PIN => Relay Activated => Inverted NO/NC status
*/

/* ############################## Function mapping
void set_illumination(char illc)
    0 <=> "x" => OFF Illumination: All LEDs OFF                         (5 LEDs OFF)
    1 <=> "a" => VLOW Illumination: Only central LED ON                 (1 LEDs ON in total)
    2 <=> "b" => LOWA Illumination: Only \ LEDs ON                      (2 LEDs ON in total)
    3 <=> "c" => LOWB Illumination: Only / LEDs ON                      (2 LEDs ON in total)
    4 <=> "d" => MIDA Illumination: Central and \ LEDs ON               (3 LEDs ON in total)
    5 <=> "e" => MIDB Illumination: Central and / LEDs ON               (3 LEDs ON in total)
    6 <=> "f" => HIGH Illumination: Only \ and / LEDs ON                (4 LEDs ON in total)
    7 <=> "g" => VHIG Illumination: All LEDs ON                         (5 LEDs ON in total)

void set_rotation(char rotc)
    1 <=> '1' =>  Rotation period set to 01 min => 6 cycles of 10 sec each = 60 seconds 
    2 <=> '2' =>  Rotation period set to 05 min => 30 cycles of 10 sec each = 300 seconds 
    3 <=> '3' =>  Rotation period set to 10 min => 60 cycles of 10 sec each = 600 seconds 
    4 <=> '4' =>  Rotation period set to 15 min => 90 cycles of 10 sec each = 900 seconds 
    5 <=> '5' =>  Rotation period set to 20 min => 120 cycles of 10 sec each = 1200 seconds 
    6 <=> '6' =>  Rotation period set to 30 min => 180 cycles of 10 sec each = 1800 seconds 

void set_onoff(char onoffc)
    0 <=> '0' =>  ON/OFF mode disabled (set to work permanently) 
    1 <=> '1' =>  ON/OFF period set to 04 hs => 1440 cycles of 10 sec each = 14400 seconds 
    2 <=> '2' =>  ON/OFF period set to 08 hs => 2880 cycles of 10 sec each = 28800 seconds 
    3 <=> '3' =>  ON/OFF period set to 12 hs => 4320 cycles of 10 sec each = 43200 seconds 
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
2   =>  Control Relay_1 (LED_A)                         => D4 (GPIO04)
7   =>  Control Relay_2 (LED_B)                         => D5 (GPIO14)
4   =>  Control Relay_3 (LED_C)                         => D6 (GPIO12)
8   =>  Control Relay_4 (FAN_A)                         => D7 (GPIO13)
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
Refer to MQTT Panel App on the cellphone
*/

/* ############################## Note: Update header of PubSubClient.h
MQTT_MAX_PACKET_SIZE defined in the library header MUST BE INCREASED (from 256 to 1024) to allow larger payloads in client.publish() method
C:\Users\Alvaro\Documents\Arduino\libraries\PubSubClient\src => PubSubClient.h
*/

/* ############################## Note: WeMos D1 R1 is different to R2
WEMOS D1 R1 is different to D1 R2 => There are some PINOUT differneces.
Check out: https://tasmota.github.io/docs/devices/Wemos-D1-R1-%26-R2/
Pinout map: https://www.google.com/search?q=wemos+d1+r1+pinout&client=firefox-b-d&sxsrf=ALeKk00jsstp66rOYZiH3N_W7hX0ctlsFA:1624199572176&tbm=isch&source=iu&ictx=1&fir=jzXSHfwEPzOfHM%252C4r1yq4sX81jpWM%252C_&vet=1&usg=AI4_-kT9wKVX0ORAHmIIyZ1qneYdeOUBuA&sa=X&ved=2ahUKEwj8sKz0tqbxAhXEr5UCHUgeDEcQ9QF6BAgDEAE#imgrc=jzXSHfwEPzOfHM
Take a look at the schematics for more information.
*/

/* ############################## Note: PINOUT correspondence Arduino/WeMos/ESP8266
//Pin       Function			    ESP8266
//D0        RXD				        GPIO3
//D1		    TXD					      GPIO1
//D2		    IO					      GPI016
//D3/D15	  IO, SCL				    GPI05
//D4/D14	  IO, SDA				    GPIO4
//D5/D13    IO, SCK				    GPIO14
//D6/D12	  IO, MISO			    GPIO12
//D7/D11	  IO, MOSI		  	  GPIO13
//D8		    IO, Pull-Up		    GPIO0
//D9		    IO, Pull-Up, LED	GPIO2
//D10		    IO, Pull-Up, SS		GPIO15
//A0		    Analog input		  A0
//5V		    5V					      -
//3V3		    3.3V				      3.3V
//G			    Ground				    GND
//RST		    Reset				      RST
######### Be aware that there are only 9 GPIOxx (D2 trough D10) availables for using!!!!!!
One of them is broken and not working at all => D2=GPIO16
*/ 

/* Testing GPIOxx of WeMos as INPUT from DHT22 (20210622 23:00)
#define D2    16    //GPIO16    => Not working as INPUT from DHT22
#define D3    5     //GPIO05    => OK as INPUT from DHT22
#define D4    4     //GPIO04    => OK as INPUT from DHT22
#define D5    14    //GPIO14    => OK as INPUT from DHT22
#define D6    12    //GPIO12    => OK as INPUT from DHT22
#define D7    13    //GPIO13    => OK as INPUT from DHT22
#define D8    0     //GPIO00    => OK as INPUT from DHT22
#define D9    2     //GPIO02    => OK as INPUT from DHT22
#define D10   15    //GPIO15    => OK as INPUT from DHT22
*/

/* Testing A0 (Analogial Input) of WeMos as INPUT from MOI (20210622 23:00)
#define A0    A0    //ANALOG    => OK as INPUT from MOI
*/

/* Testing GPIOxx of WeMos as OUTPUT to control the RELAY (20210622 23:00)
#define D2    16    //GPIO16    => Not working as OUTPUT to control RELAY
#define D3    5     //GPIO05    => OK as OUTPUT to control RELAY
#define D4    4     //GPIO04    => OK as OUTPUT to control RELAY
#define D5    14    //GPIO14    => OK as OUTPUT to control RELAY
#define D6    12    //GPIO12    => OK as OUTPUT to control RELAY
#define D7    13    //GPIO13    => OK as OUTPUT to control RELAY
#define D8    0     //GPIO00    => OK as OUTPUT to control RELAY
#define D9    2     //GPIO02    => OK as OUTPUT to control RELAY
#define D10   15    //GPIO15    => OK as OUTPUT to control RELAY
*/

/* ############################## Reference values: Tem & Hum & Moi

https://www.cannaconnection.com/blog/18563-ideal-humidity-levels-for-growing

Temperature:
Seedlings:          23-27°C
Vegetative:         22-28°C
Flowering:          20-26°C
Late Flowering:     18-24°C 
Today (20210629) in Salvador it is around 27°C. So, it will be set as pre-defined temperature in the code.

Humidity:
Seedlings:          65-80%
Vegetative:         55-70%
Flowering:          40-50%
Late Flowering:     30-40% 
Today (20210629) in Salvador it is around 75%. So, it will be set as pre-defined humidity in the code.

Moisture:
Seedlings:          65-80%
Vegetative:         55-70%
Flowering:          40-50%
Late Flowering:     30-40% 
So, will be set 70% as pre-defined moisture in the code.
*/

/* ############################## Reference values: LEDs & Power

https://www.greenbudguru.com/what-size-led-grow-light-do-i-need
https://www.ledgrowlightsdepot.com/blogs/blog/16326275-how-many-led-watts-are-required-per-square-foot-of-grow-space

Recommended wattage assuming 1.0 sq ft/plant
Plants  Sq Ft   Watts
1       1 	    30 to 40 watts
2 	    2 	    60 to 80 watts
4 	    4 	    120 to 140 watts
6 	    6 	    180 to 200 watts

 Recommended wattage assuming 2.0 sq ft/plant
Plants  Sq Ft   Watts
1 	    2 	    60 to 80 watts
2 	    4 	    120 to 140 watts
4 	    8 	    240 to 300 watts

Metric equivalences:
1 f     =>  3048 cm
1 SqFt  =>  0.092903 m2
The Indoor Growing Setup has 36,4 cm x 36,7 cm x 80,0 cm
So... it  would be about 36,4 x 36,7 = 0,1336 m2 => 1,44 SqFt
Finally, rounding... the IGS has aprox. 1,5 SqFt for just one plant.
That give us a range of about 50 to 70 watts.

The LEDs I bought are aprox. 28w each (there are five in total).
With this calculation, the optimal configuration would be 2-3 LEDs turned on at a given time.
*/