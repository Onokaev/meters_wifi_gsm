#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>

//gsm
#define pass(void)
#define TX_Pin 25
#define RX_Pin 26
SoftwareSerial SerialAT(RX_Pin, TX_Pin);
TinyGsm modem(SerialAT);
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
TinyGsmClient client1(modem);
PubSubClient mqtt1(client1);


// Your GPRS credentials, if any
const char apn[] = "safaricom.com";   //safaricom apn
const char gprsUser[] = "";
const char gprsPass[] = "";


//wifi
const char *ssid = "Connection Ting";
const char *password = "0717817569";
const char *mqtt_server = "mqtt.eclipse.org";
WiFiClient esp_Client;
PubSubClient mqtt(esp_Client);

//mqtt variables
const char* topicK = "KPLC/14286265203";
uint32_t lastReconnectAttempt = 0;
long last_Msg = 0;
char Msg[50];
int value = 0;

const int BAUD_RATE = 9600;   //serial speed of sim800l and monitor

TaskHandle_t Task1;
TaskHandle_t Task2;

//lcd display
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

const int relayPin = 16;
const int ledPin = 4;
const int ledPin1 = 2;
const int currentPin = 13;
const int ac_power = 14;

//current measurement
int mVperAmp = 185; // use 100 for 20A Module and 66 for 30A Module
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
const int consumption_Voltage = 240;
double power_consumed = 0;
double total_power = 0;
double kilowatt_hour_consumed = 0;
double current_units = 0;
double hours = 0;



void setup()                     //when it boots, we read the current_units balance from eeprom
{
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(currentPin,INPUT);
  pinMode(ac_power, INPUT);
  pinMode(relayPin, OUTPUT);
  Serial.begin(BAUD_RATE);
  SerialAT.begin(BAUD_RATE);
  digitalWrite(relayPin, HIGH);
  
  //relays work with reverse logic, set them high to turn them off
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Initializing");
  
  xTaskCreatePinnedToCore(
    Task1Code,       //the function to implement this task
    "Task1",         //name of task
    100000,            //stack size in words
    NULL,            //Task input parameter
    1,               //Priority of task
    &Task1,          //Task handle
    0);              //Core where task should run

    delay(500);

  xTaskCreatePinnedToCore(
    Task2Code,
    "Task2",
    100000,
    NULL,
    1,
    &Task2,
    1);
    delay(500);
}



void Task1Code(void *parameter)
{
  Serial.print("Task1 running on core: ");
  Serial.println(xPortGetCoreID());
  double consumption_time = 0;
  int ac_power_available = digitalRead(ac_power);
  
  for (;;)
  {
      consumption_time = millis();
      Voltage = getVPP();
      VRMS = (Voltage/2.000) *0.707;     //get RMS value of sampled voltage
      AmpsRMS = (VRMS * 1000)/mVperAmp;   //convert to current using sensor sensitivity
      power_consumed = AmpsRMS * consumption_Voltage;
      hours = (millis() - consumption_time)/3600;
      kilowatt_hour_consumed = (power_consumed * hours)/1000;
      current_units = current_units - kilowatt_hour_consumed;    //subtract power consumed from units available
      delay(500);
     // lcd.clear();

      //if ac power is lost, save current_units to eeprom                                 //////////////////////////////////////////////////IMPORTANT
      if (current_units < 0.00)
      {
         digitalWrite(relayPin, LOW);
         lcd.clear();
         lcd.print("Units used up");
         delay(500);
         lcd.clear();
         lcd.print(current_units);
         delay(500);
      }
      else
      {
        pass();
      }
        
      //Serial.print(AmpsRMS);
      //lcd.setCursor(0,1);
     // lcd.print(AmpsRMS, 3);
      //Serial.println(" Amps RMS");  
      delay(500);
  }
}


void Task2Code(void *parameters)
{
  //uint32_t lastReconnectattempt = 0;
  Serial.print("Task2 running on core: ");
  Serial.println(xPortGetCoreID());
  //SerialGSM.begin(BAUD_RATE, SERIAL_8N1, RX_Pin, TX_Pin);
//  setup_WiFi();
//  mqtt.setServer(mqtt_server, 1883);  //port 1883
//  mqtt.setCallback(callback);
  Serial.println("Wait...");
  Serial.println("Initializing modem...");

 //modem init is causing issues. after init, the modem connects to network after a long time
 // modem.init();
  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);

  Serial.print("Waiting for network...");
  //while(!modem.isNetworkConnected());   //loop until network connected
  if (!modem.waitForNetwork()) {
    Serial.println(" fail");
    delay(10000);
    //continue;
    //return;
  }
  else
  {
    Serial.println(" success");
  }
  //Serial.println(" success");

  if (modem.isNetworkConnected()) {
    Serial.println("Network connected");
  }
  else
  {
    pass();
  }
//
//#if TINY_GSM_USE_GPRS
//  // GPRS connection parameters are usually set after network registration
//    Serial.print(F("Connecting to "));
//    Serial.print(apn);
//    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
//      Serial.println(" fail");
//      delay(10000);
//      return;
//    }
//    Serial.println(" success");
//
//  if (modem.isGprsConnected()) {
//    Serial.println("GPRS connected");
//  }
//#endif

  // mqtt1 Broker setup
  mqtt1.setServer(mqtt_server, 1883);
  mqtt1.setCallback(mqtt1Callback);
  
  long int consumption_time = millis();
  current_units = 0.5;
  lcd.clear();
  for (;;)
  {
//    if (!mqtt.connected())
//    {
//      reconnect();
//    }
//    mqtt.loop();
//
//    long now = millis();
//
//    //convert value to char array
//    char string_units[10];
//    dtostrf(current_units, 1, 2, string_units);
//
//    if (now - last_Msg > 5000)
//    {
//      last_Msg = now;
//      mqtt.publish("KPLC/14286265203", string_units);
//    }
//    else
//    {
//      pass();
//    }

    ///GSM
  if (!mqtt1.connected()) 
  {
    Serial.println("=== mqtt1 NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqtt1Connect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
  }
  else
  {
    pass();
  }

  mqtt1.loop();
  long now = millis();

  //convert value to char array
  double current_units = 34;
  char string_units[10];
  dtostrf(current_units, 1, 2, string_units);

    if (now - last_Msg > 5000)
    {
      last_Msg = now;
      mqtt1.publish("KPLC/14286265203", string_units);
    }
    else
    {
      pass();
    }
  }
}



void loop()
{
  
}


//WIFI functions WIFI functions  WIFI functions WIFI functions
//WIFI functions
//WIFI functions
void setup_WiFi()
{
  delay(10);
  //we first connect to a wifi network
  Serial.println();
  Serial.print("Connecting to: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("ip address:");
  Serial.println(WiFi.localIP());
}

//callback for WiFi
void callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic:");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i=0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if(String(topic) == "KPLC/14286265203")
  {
    Serial.print(messageTemp);
  }
  else
  {
    pass();
  }
  
}


void reconnect()
{
  //loop until we are reconnected
  while(!mqtt.connected())
  {
    Serial.print("Attempting Mqtt connection...");

    if(mqtt.connect("ESP32Client"))
    {
      Serial.print("Connected");
      //subscribe to topic
      mqtt.subscribe("KPLC/14286265203");
    }
    else
    {
      Serial.print("Failed, rc = ");
      Serial.print(mqtt.state());
      Serial.print("Trying again in 5seconds");
      delay(5000);
    }
  }
}


//Gsm functions
void mqtt1Callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.write(payload, length);
  Serial.println();

  String messageTemp;
  for (int i=0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();

  // Only proceed if incoming message's topic matches
  if (String(topic) == topicK) 
  {
    Serial.print(messageTemp);
  }
}

boolean mqtt1Connect() {
  Serial.print("Connecting to ");
  Serial.print(mqtt_server);

  // Connect to mqtt1 Broker
  boolean status = mqtt1.connect("GsmClientTest");

  // Or, if you want to authenticate mqtt1:
  //boolean status = mqtt1.connect("GsmClientName", "mqtt1_user", "mqtt1_pass");
  if (status == false) {
    Serial.println(" fail");
    return false;
  }
  Serial.println(" success");
  mqtt1.publish(topicK, "GsmClientTest started");
  mqtt1.subscribe(topicK);
  return mqtt1.connected();
}

void flushBuffer()
{
  while(SerialAT.available())
  {
    SerialAT.readString();   //flushes AT serial buffer
  }
}


//set up GSM
void setupGSM()
{
  //lcd.clear();
  //lcd.print("Setting up GSM");
  SerialAT.println("AT");
  delay(5);
  flushBuffer();
  //lcd.print("Done");
}


float getVPP()
{
  float result;
  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  int minValue = 4096;          // store min value here, vlue for 12bit ADC
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(currentPin);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the maximum sensor value*/
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * 3.3)/4096.0;
      
   return result;
}

void Update_Serial()
{
      delay(500);
      while(Serial.available())
      {
        SerialAT.write(Serial.read());
      }
    
      while(SerialAT.available())
      {
        Serial.write(SerialAT.read());
        
      }
 
}
