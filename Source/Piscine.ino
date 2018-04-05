#include <xPL.h>
#include <xPL_Message.h>
#include <xPL_utils.h>

#include <avr/wdt.h>        // Watchdog
#include <SPI.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h>
#include "xPL.h"
#include "DHT.h"            // DTH21 Temperature and Humidity.
#include <LiquidCrystal.h>  // LCD
#include "EmonLib.h"        // Power current/sensor

//------------------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------------------
int relai1Pin = 39;   // Relai connected to digital - Pompe a eau On/Off
int relai2Pin = 41;   // Relai connected to digital - Pompe a eau Auto/Manuel
int relai3Pin = 43;   // Relai connected to digital - pompe a chaelur
int relai4Pin = 45;   // Relai connected to digital - sauna
int relai5Pin = 47;   // Relai connected to digital - regulateur Ph/Redox 220V
int relai6Pin = 49;   // Relai connected to digital - Surpresseur (Pour l activer une fois de temps en temps pour le garder en ordre de marche et faire circuler l'eau
int relai7Pin = 48;   // Relai connected to digital - to be used
int relai8Pin = 53;   // Relai connected to digital - to be used


int potar1Pin = A8;   // Potar tarage PH
int potar2Pin = A9;   // Potar tarage Redox

int sonde1Pin = A10;  // Sonde PH
int sonde2Pin = A11;  // Sonde Redox

uint8_t pin_ph = sonde1Pin;         // Pin pH Probe
uint8_t pin_redox = sonde2Pin;      // Pin Redox Probe

uint8_t pin_dth = 30;               // Pin sensor DTH21 Temperature and Humidity.

uint8_t pin_power_sensor = A12;            // Pin to power sensor.
unsigned int pin_power_sensor1_num = 12;
unsigned int pin_power_sensor2_num = 13;
unsigned int pin_power_sensor3_num = 14;

double Irms1 = 0;
double Irms2 = 0;
double Irms3 = 0;

uint8_t pin_potentiometer_ph = potar1Pin;     // Pin to the potentiometer used to calibrate pH
uint8_t pin_potentiometer_redox = potar2Pin; // Pin to potentiometer used to define redox desired range

float ph_sensor_value = 0.0;          // value read in Volt (0 to 5)
float ph_value_float = 0.0;           // pH value from 0.0 to 14.0 in float
char ph_value_char[5];                // pH value from 0 to 14 in char
float ph_offset = 0.0;                // Offset to apply on ph measured (Tarage)
// float ph_pente_correction = 0,0;      // To implement: Correction sur le pente to apply on ph measured (Tarage)

float redox_sensor_value = 0.0;       // value read in Volt (0 to 5)
float redox_value_float = 0.0;        // redox value from -2000 to 2000 mV in float
char redox_value_char[5];             // redox value from -2000 to 2000 in char
float redox_offset = 0.0;             // Offset to apply on redox measured (Tarage)
// float redox_pente_correction = 0,0;      // To implement: Correction sur le pente to apply on redox measured (Tarage)

float temperature_float = 0.0;
char temperature_char[5];
float humidity_float = 0.0;
char humidity_char[5];

float power_eau_value_float = 0.0;        // Power consumption in Watt
char power_eau_value_char[5];

float power_chauffage_value_float = 0.0;        // Power consumption in Watt
char power_chauffage_value_char[5];

float power_sauna_value_float = 0.0;        // Power consumption in Watt
char power_sauna_value_char[5];

bool filtration_bool = 0;             // Filtration state. 0 is off, 1 is on
bool chauffage_bool = 0;             // Chauffage state. 0 is off, 1 is on
bool sauna_bool = 0;             // Chauffage state. 0 is off, 1 is on

int affichagePowerRoll = 1;       // Permet d'afficher la puissance a tour de role sur une meme ligne du LCD

int counter_filtration = 0;           // Count minutes of filtration on 24h // To display to numbers of hours of working on 24h. The filtration time should be Temperature of the water divised by two. (Hours=Temp/2)
float counter_filtration_float = 0.0;
char counter_filtration_char[5];

unsigned long lastReadingTime = 0;
int count_time_30s = 0;               // used to trigger action every 30s (15*2s)
int count_time_30min = 0;             // used to trigger action every 30min (60*30s)
int count_time_24h = 0;               // used to trigger action every 24h (2880*30)

byte mac[] = { 0x90, 0xA2, 0xDA, 0x10, 0x09, 0xE6 };  // Production MAC address
IPAddress IP(192, 168, 4, 40); // Adresse IP donné au Shield Ethernet to be used if not DHCP
IPAddress broadcast(192, 168, 4, 255);

EthernetUDP Udp;
xPL xpl;

// LiquidCrystal lcd( 43, 42, 45, 44, 47, 46);
// https://www.arduino.cc/en/Reference/LiquidCrystalConstructor
// LiquidCrystal(rs, enable, d4, d5, d6, d7) 
LiquidCrystal lcd( 26, 27, 24, 25, 22, 23);

DHT dht(pin_dth, DHT21);

EnergyMonitor emon1;                  // Create an instance for power current Pompe a Eau
EnergyMonitor emon2;                  // Create an instance for power current Pompe a Chaleur
EnergyMonitor emon3;                  // Create an instance for power current Sauna

//------------------------------------------------------------------------------------------

void setup()
{
  // Watchdog part
  MCUSR &= ~_BV(WDRF);            // Clear the reset bit
  WDTCSR |= _BV(WDCE) | _BV(WDE); // Disable the WDT
  WDTCSR = 0;

  Serial.begin(115200);
  Serial.println(F("Starting"));

  lcd.begin(20, 4);                  // Init LCD screen, 4 lignes by 20 chars
  lcd.clear();

  lcd.setCursor (0, 1);
  lcd.print("DHCP (10s)");
  //Lancement connexion Ethernet avec l'adresse MAC et l'adresse IP
  Ethernet.begin( mac );
  // Ethernet.begin( mac, IP );

  lcd.setCursor (0, 2);
  lcd.print( IP );
  delay(1000);
  printMac(mac);
  printIP();      // Show IP in serial monitor

  Udp.begin(xpl.udp_port);

  xpl.SendExternal = &SendUdPMessage;                             // pointer to the send callback
  xpl.AfterParseAction = &AfterParseAction;                       // pointer to a post parsing action callback
  xpl.SetSource_P(PSTR("xpl"), PSTR("arduino"), PSTR("piscine")); // parameters for hearbeat message

  dht.begin();                                            // Start tempature and humidity sensor
  
  emon1.current( pin_power_sensor1_num, 29);               // Power Current: input pin, calibration.
  // Calibration à 29 par tatonnement (voir fichier excel) car je ne connais pas le nombre de tour, ni la resistance burden
  // pour appliquer le calcul http://www.homautomation.org/2013/09/17/current-monitoring-with-non-invasive-sensor-and-arduino/
  emon2.current( pin_power_sensor2_num, 29);
  emon3.current( pin_power_sensor3_num, 29);

  lcd.clear();
  lcd.setCursor (0, 0); lcd.print("T:");      lcd.setCursor (10, 0);    lcd.print("F:");      lcd.setCursor (12, 0); lcd.print("...(24h)");
  lcd.setCursor (0, 1); lcd.print("Ph:");     lcd.setCursor (10, 1);    lcd.print("Rx:");
  // lcd.setCursor (0, 2); lcd.print("Eau:");    lcd.setCursor (10, 2);    lcd.print("Cha:");
  lcd.setCursor (0, 3); lcd.print("Mode:");

  delay(2000);

  pinMode(relai1Pin, OUTPUT);
  pinMode(relai2Pin, OUTPUT);
  pinMode(relai3Pin, OUTPUT);
  pinMode(relai4Pin, OUTPUT);
  pinMode(relai5Pin, OUTPUT);
  pinMode(relai6Pin, OUTPUT);
  pinMode(relai7Pin, OUTPUT);
  pinMode(relai8Pin, OUTPUT);
  digitalWrite( relai1Pin, 0);
  digitalWrite( relai2Pin, 0);
  digitalWrite( relai3Pin, 0);
  digitalWrite( relai4Pin, 0);
  digitalWrite( relai5Pin, 0);
  digitalWrite( relai6Pin, 0);
  digitalWrite( relai7Pin, 0);
  digitalWrite( relai8Pin, 0);

  lcd.setCursor (5, 3);
  lcd.print("               "); // 15
  lcd.setCursor (5, 3);
  if ( digitalRead(relai1Pin) ) lcd.print("On-"); else lcd.print("Off-");
  if ( digitalRead(relai2Pin) ) lcd.print("Auto"); else lcd.print("Manuel");

  wdt_enable(WDTO_4S); //enable 4s watchdog
}

void loop()
{
  xpl.Process();  // heartbeat management

  // Parser part. Read input XPL message
  if (Udp.parsePacket())
  {
    char xPLMessageBuff[XPL_MESSAGE_BUFFER_MAX];
    Udp.read(xPLMessageBuff, XPL_MESSAGE_BUFFER_MAX); // read the packet into packetBufffer
    xpl.ParseInputMessage(xPLMessageBuff);              // parse message
  }

  // Protect if millis return to 0 (every 50 days)
  if (millis() - lastReadingTime < 0)
  {
    lastReadingTime = millis();
  }
  //-------------------------------------------------------------------------------------
  // Show datas on LCD every 2 seconds
  if ((millis() - lastReadingTime) >= 2000)
  {
      lcd.clear();
      lcd.setCursor (0, 0); lcd.print("T:");      lcd.setCursor (10, 0);    lcd.print("F:");      lcd.setCursor (12, 0); lcd.print("...(24h)");
      lcd.setCursor (0, 1); lcd.print("Ph:");     lcd.setCursor (10, 1);    lcd.print("Rx:");
      // lcd.setCursor (0, 2); lcd.print("Eau:");    lcd.setCursor (10, 2);    lcd.print("Cha:");
      lcd.setCursor (0, 3); lcd.print("Mode:");

      lcd.setCursor (5, 3);
      lcd.print("               "); // 15
      lcd.setCursor (5, 3);
      if ( digitalRead(relai1Pin) ) lcd.print("On-"); else lcd.print("Off-");
      if ( digitalRead(relai2Pin) ) lcd.print("Auto"); else lcd.print("Manuel");
    
    // pH Part
    // See excel file witdh value measured to do the calibration
    // Initial code was with theoritical formulas, I use direct calibration from Ph/Redox values and Arduino raw value. Linear modelisation between 2 points.
    // ph_value_float = 3.0/187.0*(analogRead(pin_ph)+(analogRead(pin_potentiometer_ph)-513.0)/10.0)-2.305;
    // ph_value_float = 3.0 / 187.0 * analogRead(pin_ph) + ph_offset - 2.305;
    
    ph_sensor_value = analogRead(pin_ph) * 5000.0 / 1023.0 / 1000.0; // form 0.0 to 5.0 V
    ph_value_float = 0.9 * ph_value_float + 0.1 * ( (0.0178 * ph_sensor_value * 200.0) - 1.889 + ph_offset );     // formula to calcul pH from sensor value
    
    lcd.setCursor (3, 1);
    lcd.print("       ");   // Clean lcd old digits : 7
    lcd.setCursor (3, 1);
    lcd.print(ph_value_float, 2);

    // Redox Part
    // See excel file witdh value measured to do the calibration
    // Initial code was with theoritical formulas, I use direct calibration from Ph/Redox values and Arduino raw value. Linear modelisation between 2 points.
    // redox_value_float = 1.507 * (analogRead(pin_redox)+(analogRead(pin_potentiometer_redox)-513.0)/10.0) -339.937;
    // redox_value_float = 1.507 * analogRead(pin_redox) + redox_offset - 339.937;
    redox_sensor_value = analogRead(pin_redox) * 5000.0 / 1023.0 / 1000.0;   // form 0.0 to 5.0 V
    redox_value_float = 0.9 * redox_value_float + 0.1 * ( ((2.5 - redox_sensor_value) / 1.037) * 1000.0 +redox_offset );     // from -2000 to 2000 mV
    
    lcd.setCursor (13, 1);
    lcd.print("       ");  // Clean lcd old digits : 7
    lcd.setCursor (13, 1);
    lcd.print(redox_value_float, 0); lcd.print("mV");

    // DHT Temp and humidity Part
    temperature_float = dht.readTemperature();
    humidity_float = dht.readHumidity();
    lcd.setCursor (2, 0);
    lcd.print("        "); // Clean lcd old digits: 8
    lcd.setCursor (2, 0);
    lcd.print(temperature_float, 1); lcd.print("C");


    // Power sensor pompe a eau
    Irms1 = emon1.calcIrms(1480);  // Calculate Power current (Irms only). 1480 is the number of sample used to compute a value. More info on https://openenergymonitor.org/emon/buildingblocks/explanation-of-the-phase-correction-algorithm
    // power_eau_value_float = Irms1 * 232.0; // Formula for one phase
    power_eau_value_float = Irms1 * 380.0 * 0.8 * 1.7320 ; // P = UI cos(phi) racine(3) http://www.volta-electricite.info/articles.php?lng=fr&pg=2517
    
    // lcd.setCursor (4, 2);
    // lcd.print("      ");  // Clean lcd old digits: 6
    // lcd.setCursor (4, 2);
    // lcd.print(power_eau_value_float, 0); lcd.print("W");

    // Power state Pompe A Eau. If Power is more than 300W -> on: filtration in progress
    if (power_eau_value_float > 300)  filtration_bool = 1;  else filtration_bool = 0;

    // Power sensor pompe a chaleur
    Irms2 = emon2.calcIrms(1480);  // Calculate Power current (Irms only). 1480 is the number of sample used to compute a value. More info on https://openenergymonitor.org/emon/buildingblocks/explanation-of-the-phase-correction-algorithm
    // power_chauffage_value_float = Irms2 * 232.0;
    power_chauffage_value_float = Irms2 * 380.0 * 0.8 * 1.7320 ; // P = UI cos(phi) racine(3) http://www.volta-electricite.info/articles.php?lng=fr&pg=2517
    // lcd.setCursor (14, 2);
    // lcd.print("      ");  // Clean lcd old digits: 6
    // lcd.setCursor (14, 2);
    // lcd.print(power_chauffage_value_float, 0); lcd.print("W");

    // Power state Pompe A Eau Power is more than 300W on: Chauffage in progress
    if (power_chauffage_value_float > 300) chauffage_bool = 1; else chauffage_bool = 0;

    // Power sensor Sauna
    Irms3 = emon3.calcIrms(1480);  // Calculate Power current (Irms only). 1480 is the number of sample used to compute a value. More info on https://openenergymonitor.org/emon/buildingblocks/explanation-of-the-phase-correction-algorithm
    // power_sauna_value_float = Irms3 * 232.0;
    power_sauna_value_float = Irms3 * 380.0 * 0.8 * 1.7320 ; // P = UI cos(phi) racine(3) http://www.volta-electricite.info/articles.php?lng=fr&pg=2517
    //lcd.setCursor (4, 2);
    //lcd.print("      ");  // Clean lcd old digits: 6
    //lcd.setCursor (4, 2);
    //lcd.print(power_eau_value_float, 0); lcd.print("W");

    // Power state Sauna. If Power is more than 300W -> on: sauna in progress
    if (power_sauna_value_float > 300)  sauna_bool = 1;  else sauna_bool = 0;

    lcd.setCursor (0, 2);
    lcd.print("                    ");  // Clean lcd old digits: 20
    lcd.setCursor (0, 2);
    switch (affichagePowerRoll) {
      case 1:
        lcd.print("P. a Eau: "); lcd.print(power_eau_value_float, 0); lcd.print("W");
        break;
      case 2:
        lcd.print("P. a chaleur: "); lcd.print(power_chauffage_value_float, 0); lcd.print("W");
        break;
      case 3:
        lcd.print("Sauna: "); lcd.print(power_sauna_value_float, 0); lcd.print("W");
        break;
    }
    affichagePowerRoll++;
    if (affichagePowerRoll > 3) affichagePowerRoll = 1;

    count_time_30s++; // Count 15 cycles for sending XPL every 30s
    lastReadingTime = millis();
  }
  //----------------------------------------------------------------------------
  // Send datas as xPL Message every 30 seconds
  if (count_time_30s == 15)
  {
    // pH Part
    dtostrf(ph_value_float , 3, 2, ph_value_char);                          // float to char format: XX.XX
    print_sensor_value("pH", analogRead(pin_ph), ph_value_float);           // debug print
    send_xpl_message("ph", ph_value_char);                                  // send xpl message

    // Redox Part
    dtostrf(redox_value_float, 5, 0, redox_value_char);                     // float to char with decimal resolution format: XXXX
    print_sensor_value("Redox", analogRead(pin_redox), redox_value_float);  // debug print
    send_xpl_message("redox", redox_value_char);                            // send xpl message

    // DHT Temp and humidity Part
    //Send temperature to XPL
    dtostrf(temperature_float , 3, 2, temperature_char);
    print_sensor_value("temp", temperature_float, temperature_float);
    send_xpl_message("temp", temperature_char);                             //send xpl message

    //Send humidity to XPL
    dtostrf(humidity_float , 3, 2, humidity_char);
    print_sensor_value("humidity", humidity_float, humidity_float);
    send_xpl_message("humidity", humidity_char);                            //send xpl message

    // Power sensor Pompe Eau
    dtostrf(power_eau_value_float , 4, 0, power_eau_value_char);
    print_sensor_value("PompeEauW", power_eau_value_float, power_eau_value_float);
    send_xpl_message("PompeEauW", power_eau_value_char);                          //send xpl message

    // Power state
    if (filtration_bool == 1) send_xpl_message("pompeEau", "1"); else send_xpl_message("pompeEau", "0"); // Power is more than 300W, filtration in progress

    // Power sensor Pompe a Chaleur
    dtostrf(power_chauffage_value_float , 4, 0, power_chauffage_value_char);
    print_sensor_value("pompeChaleurW", power_chauffage_value_float, power_chauffage_value_float);
    send_xpl_message("pompeChaleurW", power_chauffage_value_char);                          //send xpl message

    // Power state
    if (chauffage_bool == 1) send_xpl_message("pompeChaleur", "1"); else send_xpl_message("pompeChaleur", "0");  // Power is more than 300W, filtration in progress

    // Power sensor Sauna
    dtostrf(power_sauna_value_float , 4, 0, power_sauna_value_char);
    print_sensor_value("SaunaW", power_sauna_value_float, power_sauna_value_float);
    send_xpl_message("saunaW", power_sauna_value_char);                          //send xpl message

    // Power state
    if (sauna_bool == 1) send_xpl_message("sauna", "1"); else send_xpl_message("sauna", "0"); // Power is more than 300W, filtration in progress


    if (count_time_30min % 2 == 0)    // every 1min, used to count minutes of active filtration on 24h
    {
      if (filtration_bool == 1)  counter_filtration++;     // if filtration in progress

      print_sensor_value("timer", counter_filtration * 1.0, counter_filtration * 1.0);
      counter_filtration_float = counter_filtration * 1.0;
      send_xpl_message("timer", dtostrf(counter_filtration_float, 5, 0, counter_filtration_char) );
      Serial.println(counter_filtration);
      count_time_24h++;
    }

    Serial.println("");

    count_time_30s = 0;
    count_time_30min++;
  }
  //----------------------------------------------------------------------------
  if (count_time_30min == 60)     // every 30min (60*30s)
  {
    /*
        if ((redox_value_float > redox_max) && (filtration_bool == 1))    // if to much clorine and filtration in progress
        {
          digitalWrite(pin_relay_justsalt, 1);              // Set the relay to stop Clorine production
        }
        if ((redox_value_float < redox_min) && (filtration_bool == 1))
        {
          digitalWrite(pin_relay_justsalt, 0);              // Set the relay to start Clorine production
        }
    */
    count_time_30min = 0;
  }
  //----------------------------------------------------------------------------


  if (count_time_24h == 2880)     // every 24h (1440*1min)
  {
    Serial.println(F("Every 24h"));
    Serial.println(count_time_24h);
    lcd.setCursor (12, 0);
    // lcd.print("        "); // 8
    lcd.setCursor (12, 0);
    lcd.print(counter_filtration / 60);                 // Affiche la partie heures
    lcd.print("h");                                     // affiche h
    if (counter_filtration % 60 < 10) lcd.print("0");   // Affiche un zero si la partie minute n'a qu'1 digit et pas deux
    lcd.print(counter_filtration % 60);                 // Affiche la partie minutes

    counter_filtration = 0;
    count_time_24h = 0;
  }
  //----------------------------------------------------------------------------

  wdt_reset();  //Reset the Watchdog timer
}

// Send UDP Message
void SendUdPMessage(char *buffer)
{
  Udp.beginPacket(broadcast, xpl.udp_port);
  Udp.write(buffer);
  Udp.endPacket();
}

// Print MAC Address
void printMac (const byte *buf)
{
  Serial.print(F("MAC: "));
  for (byte i = 0; i < 6; ++i)
  {
    if (buf[i] >= 0 && buf[i] <= 16)
      Serial.print(F("0"));
    Serial.print( buf[i], HEX );
    if (i < 5)
      Serial.print(F(":"));
  }
  Serial.println("");
}

// Print IP address
void printIP()
{
  // print your local IP address:
  Serial.print(F("My IP address: "));
  for (byte thisByte = 0; thisByte < 4; thisByte++)
  {
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(F("."));
  }
  Serial.println();
}

// Print debug info into serial monitor
void print_sensor_value(char* name, int sensor_value, float value)
{
  //print the results to the serial monitor for debug:
  Serial.print(name);
  Serial.print(F(" sensor: "));
  Serial.print(sensor_value);
  Serial.print(F(" output: "));
  Serial.println(value);
}

// Send XPL Message
void send_xpl_message(char* type, char* current)
{
  xPL_Message msg;

  msg.type = XPL_TRIG;
  msg.hop = 1;
  msg.SetSource( "xpl", "arduino", "piscine");
  msg.SetTarget_P(PSTR("*"));

  msg.SetSchema_P(PSTR("sensor"), PSTR("basic"));
  msg.AddCommand_P(PSTR("device"), PSTR("piscine"));
  msg.AddCommand("type", type);
  msg.AddCommand("current", current);
  xpl.SendMessage(&msg);
}

// Parse input XPL messages
void AfterParseAction(xPL_Message * message)
{
  int relaiValue = 0;
  int pin = 0;
  int chgPin = 0;

  Serial.println( F("AfterParse") );
  if (xpl.TargetIsMe(message))
  {
    Serial.println( F("ForMe") );
    if (message->IsSchema("control", "basic"))
    {

      if ( (strcmp(message->command[0].name, "type") == 0)    /* && (strcmp(message->command[0].value, "piscine" ) == 0) */ )
      {
        if ( (strcmp(message->command[1].name, "current") == 0) /* && (strcmp(message->command[2].name, "data") == 0) */ )
        {
          if ( strcmp(message->command[0].value, "relai1"   ) == 0 ) {
            pin = relai1Pin;
            chgPin = 1;
          }
          if ( strcmp(message->command[0].value, "relai2"   ) == 0 ) {
            pin = relai2Pin;
            chgPin = 1;
          }
          if ( strcmp(message->command[0].value, "relai3"   ) == 0 ) {
            pin = relai3Pin;
            chgPin = 1;
          }
          if ( strcmp(message->command[0].value, "relai4"   ) == 0 ) {
            pin = relai4Pin;
            chgPin = 1;
          }
          if ( strcmp(message->command[0].value, "relai5"   ) == 0 ) {
            pin = relai5Pin;
            chgPin = 1;
          }
          if ( strcmp(message->command[0].value, "relai6"   ) == 0 ) {
            pin = relai6Pin;
            chgPin = 1;
          }
          if ( strcmp(message->command[0].value, "relai7"   ) == 0 ) {
            pin = relai7Pin;
            chgPin = 1;
          }
          if ( strcmp(message->command[0].value, "relai8"   ) == 0 ) {
            pin = relai8Pin;
            chgPin = 1;
          }
          if ( strcmp(message->command[1].value, "on"       ) == 0 ) {
            relaiValue = 1;
          }
          if ( strcmp(message->command[1].value, "off"      ) == 0 ) {
            relaiValue = 0;
          }
          digitalWrite( pin, relaiValue);
          //Serial.println(message->command[1].value, message->command[2].value) );
          send_xpl_message( message->command[0].value, message->command[1].value);

          // ph_offset
          if ( strcmp(message->command[0].value, "phOffset"   ) == 0 ) {
            Serial.print( F("phOffset: ") );
            Serial.println(message->command[1].value);
            ph_offset = atof( message->command[1].value ) / 100;
            Serial.print( F("phOffset: ") );
            Serial.println(ph_offset);
          }

          // redox_offset
          if ( strcmp(message->command[0].value, "redoxOffset"   ) == 0 ) {
            Serial.print( F("redoxOffset: ") );
            Serial.println(message->command[1].value);
            redox_offset = atof( message->command[1].value );
            Serial.print( F("redoxOffset: ") );
            Serial.println(redox_offset);
          }


          lcd.setCursor (5, 3);
          lcd.print("               "); // 15
          lcd.setCursor (5, 3);
          if ( digitalRead(relai1Pin) )
          {
            lcd.print("On-");
            if ( digitalRead(relai2Pin) ) lcd.print("Auto"); else lcd.print("Manuel");
            if ( digitalRead(relai3Pin) ) lcd.print("-Chauf");
          }
          else lcd.print(" Off");
        }
      }
    }
  }

}


