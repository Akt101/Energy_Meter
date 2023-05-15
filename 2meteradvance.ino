
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <string.h>

ModbusMaster meter1;
SoftwareSerial meterSerial(4, 5);                          //Rx, Tx
WiFiManager wm;
WiFiClient client;
HTTPClient https;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800);
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

String host = "energy-meter-";
const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = "admin@123";

String Device_Id = "api_key=Smdtec123h4w389e7ll&id=";
String serverName = "https://nbri.in/mongo_file/post_data.php" ; //http://smartdwelliot.in/mongo_file/post_data.php
char mtr1[20];
char mtr2[20];
char mtr3[20];
char mtr4[20];
char mtr5[20];
char devName[25];

void takeReading();
void networkStatus();
String serialize(ModbusMaster , String );
void read_reg(ModbusMaster );
float uniData(ModbusMaster , int , int );
void getReadingReg(ModbusMaster , int , int );
void preTransmission();
void postTransmission();
void DateAndTime_online();
void device_Initialize();
void add_Custom_Param();
String getParam(String);
void saveParamCallback();
void spiffs_check();
void spiffs_write();
void spiffs_read();
void checkButton();
void autocon();
//====================================================================Defining Libraries===========================================================================


#define MAX485_TX_ENABLE  16                               // DE & RE
#define WIFI_PIN 0
WiFiManagerParameter custom_html;
WiFiManagerParameter field1;
WiFiManagerParameter field2;
WiFiManagerParameter field3;
WiFiManagerParameter field4;
WiFiManagerParameter field5;
WiFiManagerParameter field6;

//====================================================================Defining variables===========================================================================
char time_online[] = "00000000000000";
unsigned long currentmillis = 0;
long lastReconnectAttempt = 0;
float readings[44];                     // Array to Store readings
long int event_2 = 10000;
unsigned long int prev_2 = 0;
int wifiState = 0, lastWifiState = 0, csWifi = 0;
int result = 0;                         // Starting Address for MODBUS


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);                                                           //Serial Begin
  meterSerial.begin(9600);
  pinMode(MAX485_TX_ENABLE, OUTPUT);
  pinMode(WIFI_PIN, INPUT);
  digitalWrite(MAX485_TX_ENABLE, 0);
  delay(1000);
  Serial.println("---START---");
  spiffs_check();
 // spiffs_read();
  checkButton();
  device_Initialize();
  delay(1000);
  
  MDNS.begin(host+devName);
  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and password '%s'\n", host, update_path, update_username, update_password);

  Serial.println("---END---");
  timeClient.update();
}

void loop() {
  // put your main code here, to run repeatedly:
  httpServer.handleClient();
  MDNS.update();
  currentmillis = millis();
  wm.process();
  delay(10);
  takeReading();
  networkStatus();
}


void takeReading() {
  timeClient.update();
  if (currentmillis - prev_2 >= event_2) {
    std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);
    client->setInsecure();
    String liveData = "";

    //Meter-1
    https.begin(*client, serverName);
    // Specify content-type header
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");    
    liveData = serialize(meter1, mtr1);
    int httpResponseCode = https.POST(liveData);   // change liveData to storeDataArray
    Serial.println(liveData);
    Serial.print("1st HTTP Response code: ");
    Serial.println(httpResponseCode);
    //Free resources
    https.end();

    prev_2 = currentmillis;
  }
}

void networkStatus() {    // Network Info

  wifiState = WiFi.status();
  if (wifiState != lastWifiState) {
    if (wifiState == 3) {
      Serial.println("Wifi is back!");
      csWifi =  1;
    }
    else {
      csWifi = 0;
    }
  }
  lastWifiState = wifiState;
}

String serialize(ModbusMaster Meter_Id, String ID) {                                                                           //Take Readings
  DateAndTime_online();
  read_reg(Meter_Id);

  String dataReading = "";

  dataReading += Device_Id ;
  dataReading += ID ;

  dataReading += "&d1=";
  dataReading += readings[1];

  dataReading += "&d2=";
  dataReading += readings[2];

  dataReading += "&d3=";
  dataReading += readings[3];

  dataReading += "&d4=";
  dataReading += readings[4];

  dataReading += "&d5=";
  dataReading += readings[5];

  dataReading += "&d6=";
  dataReading += readings[6];

  dataReading += "&d7=";
  dataReading += readings[7];

  dataReading += "&d8=";
  dataReading += readings[8];

  dataReading += "&d9=";
  dataReading += readings[9];

  dataReading += "&d10=";
  dataReading += readings[10];

  dataReading += "&d11=";
  dataReading += readings[11];

  dataReading += "&d12=";
  dataReading += readings[12];

  dataReading += "&d13=";
  dataReading += readings[13];

  dataReading += "&d14=";
  dataReading += readings[14];

  dataReading += "&d15=";
  dataReading += readings[15];

  dataReading += "&d16=";
  dataReading += readings[16];

  dataReading += "&d17=";
  dataReading += readings[17];

  dataReading += "&d18=";
  dataReading += readings[18];

  dataReading += "&d19=";
  dataReading += readings[19];

  dataReading += "&d20=";
  dataReading += readings[20];

  dataReading += "&d21=";
  dataReading += readings[21];

  dataReading += "&d22=";
  dataReading += readings[22];

  dataReading += "&d23=";
  dataReading += readings[23];

  dataReading += "&d24=";
  dataReading += readings[24];

  dataReading += "&d25=";
  dataReading += readings[25];

  dataReading += "&d26=";
  dataReading += readings[26];

  dataReading += "&d27=";
  dataReading += readings[27];

  dataReading += "&d28=";
  dataReading += readings[28];

  dataReading += "&d29=";
  dataReading += readings[29];

  dataReading += "&d30=";
  dataReading += readings[30];

  dataReading += "&d31=";
  dataReading += readings[31];

  dataReading += "&d32=";
  dataReading += readings[32];

  dataReading += "&d33=";
  dataReading += readings[33];

  dataReading += "&d34=";
  dataReading += readings[34];

  dataReading += "&d35=";
  dataReading += readings[35];

  dataReading += "&d36=";
  dataReading += readings[36];

  dataReading += "&d37=";
  dataReading += readings[37];

  dataReading += "&d38=";
  dataReading += readings[38];  

  dataReading += "&d39=";
  dataReading += readings[39]; 

  dataReading += "&d40=";
  dataReading += readings[40];

  dataReading += "&d41=";
  dataReading += readings[41];

  dataReading += "&d42=";
  dataReading += readings[42];

  dataReading += "&d43=";
  dataReading += readings[43];

  dataReading += "&d44=";
  dataReading += readings[44];

  dataReading += "&d45=";
  dataReading += readings[45];

  dataReading += "&time=";
  dataReading += time_online;

  return dataReading;
}

void read_reg(ModbusMaster Meter_Id_A) {
  // First batch reading 52 Registers....
  uint8_t  result_a;
  uint8_t  result_b;
  
  
  //getReadingReg(Meter_Id_A, 0, 52);     // (Reg No Starting From, Reg QTY to Read)
    result_a = Meter_Id_A.readInputRegisters(1, 52);
    
    Serial.print("Error code A: ");
    Serial.println(result_a);

  if (result_a == Meter_Id_A.ku8MBSuccess) {
    
    float a = uniData(Meter_Id_A, 0, 1);

    Serial.print("A Data: ");Serial.println(a);
    
    /*readings[2] = uniData(Meter_Id_A, 2, 3);
    readings[3] = uniData(Meter_Id_A, 4, 5);
    readings[4] = uniData(Meter_Id_A, 6, 7);
    readings[5] = uniData(Meter_Id_A, 8, 9);
    readings[6] = uniData(Meter_Id_A, 10, 11);
    readings[7] = uniData(Meter_Id_A, 12, 13);
    readings[8] = uniData(Meter_Id_A, 14, 15);
    readings[9] = uniData(Meter_Id_A, 16, 17);
    readings[10] = uniData(Meter_Id_A, 18, 19);
    readings[11] = uniData(Meter_Id_A, 20, 21);
    readings[12] = uniData(Meter_Id_A, 22, 23);
    readings[13] = uniData(Meter_Id_A, 24, 25);
    readings[14] = uniData(Meter_Id_A, 26, 27);
    readings[15] = uniData(Meter_Id_A, 28, 29);
    readings[16] = uniData(Meter_Id_A, 30, 31);
    readings[17] = uniData(Meter_Id_A, 32, 33);
    readings[18] = uniData(Meter_Id_A, 34, 35);
    readings[19] = uniData(Meter_Id_A, 36, 37);
    readings[20] = uniData(Meter_Id_A, 38, 39);
    readings[21] = uniData(Meter_Id_A, 40, 41);// same
    
    readings[27] = uniData(Meter_Id_A, 42, 43);// mfm376-pf1   em4 
    readings[28] = uniData(Meter_Id_A, 44, 45);//mfm376-pf2
    readings[29] = uniData(Meter_Id_A, 46, 47);//mfm376-pf3
    readings[22] = uniData(Meter_Id_A, 48, 49);//mfm376-pf avg
    readings[23] = uniData(Meter_Id_A, 50, 51);//mfm376-freq
    */
    Serial.println("First Read Complete");
  }
  delay(200);



  result_b = Meter_Id_A.readHoldingRegisters(0, 50);
  Serial.print("Error code B: "); Serial.println(result_b);
  if (result_b == Meter_Id_A.ku8MBSuccess) {
  
    float b = uniData(Meter_Id_A, 0, 1);

    Serial.print("B Data: ");Serial.println(b);
    Serial.print("Final: ");
    float final1 = (Meter_Id_A.getResponseBuffer(0x00)/100.0f);
    Serial.println(final1);
  }
  /*
  // Second batch reading 50 Registers....
 // getReadingReg(Meter_Id_A, 52, 14);     // (Reg No Starting From, Reg QTY to Read)
    result_b = Meter_Id_A.readInputRegisters(52, 14);
    Serial.print("Error code B: ");
    Serial.println(result_b);
  if (result_b == Meter_Id_A.ku8MBSuccess) {
    readings[24] = uniData(Meter_Id_A, 0, 1);       //mfm376-kw
    readings[25] = uniData(Meter_Id_A, 2, 3);       //mfm376-kvar
    readings[26] = uniData(Meter_Id_A, 4, 5);       //mfm376-kva
    readings[33] = uniData(Meter_Id_A, 6, 7);     //mfm376-kwh
    readings[34] = uniData(Meter_Id_A, 8, 9);     //mfm376-kvarh
    readings[35] = uniData(Meter_Id_A, 10, 11);     //mfm376-kvah
    Serial.println("Second Read Complete"); 
  }
  */
}

/*
float method1(ModbusMaster Meter_Id_B, int a, int b){



  
}*/


float uniData(ModbusMaster Meter_Id_B, int a, int b) {
  float opa ;
  union
  {
    uint32_t x;
    float f;
  } u;
  u.x = (((unsigned long)Meter_Id_B.getResponseBuffer(b) << 16) | Meter_Id_B.getResponseBuffer(a));

  opa = u.f ;
  return opa;
}


void preTransmission() {
  digitalWrite(MAX485_TX_ENABLE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_TX_ENABLE, 0);
}

void DateAndTime_online() {
  const unsigned  int time2020 = 1577836800;
  const unsigned  int time2036 = 2085436800;

  unsigned long int epochTime = timeClient.getEpochTime();
  //Get a time structure
  //  Serial.print("Epoch:  "); Serial.println(epochTime);
  if (epochTime > time2020 && epochTime < time2036) {       // Time Condition for 2036
    //  Serial.print("Time NTP: ");
    struct tm *ptm = gmtime ((time_t *)&epochTime);
    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon + 1;
    int currentYear = ptm->tm_year + 1900;
    time_online[0] = ((currentYear) / 1000) % 10 + '0';         //To get 1st digit from year()
    time_online[1] = ((currentYear) / 100) % 10 + '0';          //To get 2nd digit from year()
    time_online[2] = ((currentYear) / 10) % 10 + '0';          //To get 3rd digit from year()
    time_online[3] = (currentYear) % 10 + '0';                 //To get 4th digit from year()
    time_online[4] = (currentMonth) / 10 + '0';                //To get 1st digit from month()
    time_online[5] = (currentMonth) % 10 + '0';                 //To get 2nd digit from month()
    time_online[6] = (monthDay) / 10 + '0';                   //To get 1st digit from day()
    time_online[7] = (monthDay) % 10 + '0';                    //To get 2nd digit from day()
    time_online[8] = (timeClient.getHours()) / 10 + '0';
    time_online[9] = (timeClient.getHours()) % 10 + '0';
    time_online[10] = (timeClient.getMinutes()) / 10 + '0';
    time_online[11] = (timeClient.getMinutes()) % 10 + '0';
    time_online[12] = (timeClient.getSeconds()) / 10 + '0';
    time_online[13] = (timeClient.getSeconds()) % 10 + '0';
  }
}

void device_Initialize() {

  // WiFi Manager **
  
  // wm.resetSettings();

  // WiFi Manager **
  autocon();
  timeClient.begin();
  timeClient.setTimeOffset(19800);

  meter1.begin(1, meterSerial);                                                    // Modbus slave ID 1
  meter1.preTransmission(preTransmission);
  meter1.postTransmission(postTransmission);

  Serial.println("Device Initialized");

}

void add_Custom_Param() {
  // add a custom input field
  int customFieldLength = 20;
  new (&custom_html) WiFiManagerParameter ("<h2>Configuration</h2>"); // only custom html
  new (&field1) WiFiManagerParameter("customfieldid1", "Device Name", devName, customFieldLength, "placeholder=\"Site Location\"");
  new (&field2) WiFiManagerParameter("customfieldid2", "Meter-1", mtr1, customFieldLength, "placeholder=\"\"");
  new (&field3) WiFiManagerParameter("customfieldid3", "Meter-2", mtr2, customFieldLength, "placeholder=\"\"");
  new (&field4) WiFiManagerParameter("customfieldid4", "Meter-3", mtr3, customFieldLength, "placeholder=\"\"");
  new (&field5) WiFiManagerParameter("customfieldid5", "Meter-4", mtr4, customFieldLength, "placeholder=\"\"");
  new (&field6) WiFiManagerParameter("customfieldid6", "Meter-5", mtr5, customFieldLength, "placeholder=\"\"");
  wm.addParameter(&custom_html);
  wm.addParameter(&field1);
  wm.addParameter(&field2);
  wm.addParameter(&field3);
  wm.addParameter(&field4);
  wm.addParameter(&field5);
  wm.addParameter(&field6);

  wm.setSaveParamsCallback(saveParamCallback);
  std::vector<const char *> menu = {"wifi", "info", "param", "sep", "restart", "exit"};
  wm.setMenu(menu);

  wm.setCustomHeadElement("<h1>Smartdwell Technologies</h1>");
  //wm.setCustomHeadElement("<style>html{filter: invert(100%); -webkit-filter: invert(100%);} </style>");

  wm.setClass("invert");
  wm.setConnectTimeout(100); // how long to try to connect for before continuing
  wm.setConfigPortalTimeout(180);
  wm.setConfigPortalBlocking(true);
  //wm.setCleanConnect(true); // disconnect before connect, clean connect
  wm.setAPClientCheck(true);

}

void autocon(){
    bool res;
  res = wm.autoConnect("New_Device_Setup", "738738738");
  if (!res) {
    Serial.println("Failed to connect or hit timeout");
  }
  else {
    Serial.println("Auto connect WiFi");
  }
}

String getParam(String name) {
  //read parameter from server, for customhmtl input
  String value;
  if (wm.server->hasArg(name)) {
    value = wm.server->arg(name);
  }
  return value;
}

void saveParamCallback() {
  Serial.println("[CALLBACK] saveParamCallback fired");
  spiffs_write();
  // After writing call read function to get updated values
  spiffs_read();
}

void spiffs_check() {
  if (SPIFFS.begin()) {
    Serial.println("Mounted SPIFFS");
    if (SPIFFS.exists("/config.json")) {
      Serial.println("Config file exist");
    }
    else {
      Serial.println("Config file dosent Exist");
      Serial.println("Creating Config File");
      SPIFFS.format();
      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println("file creation failed again.");
      }
      else
      {
        Serial.println("File created successfully");
        configFile.close();  //Close file
      }
    }
  }
}

void spiffs_write() {

  const size_t capacity = JSON_OBJECT_SIZE(10);
  DynamicJsonDocument doc(capacity);

  doc["id1"]  = field1.getValue();
  doc["id2"]  = field2.getValue();
  doc["id3"]  = field3.getValue();
  doc["id4"]  = field4.getValue();
  doc["id5"]  = field5.getValue();
  doc["id6"]  = field6.getValue();
  

  if (SPIFFS.begin()) {
    Serial.println("Mounted SPIFFS");
    if (SPIFFS.exists("/config.json")) {
      Serial.println("Config file exist");
      File configFile = SPIFFS.open("/config.json", "w");
      if (configFile) {
        Serial.println("Opened Config file");
        serializeJson(doc, configFile);
        serializeJson(doc, Serial);
        configFile.close();
        Serial.println("Closed config file");
        Serial.println("Data Updated sucessfully");
      }
      else {
        Serial.println("Config file not Opened");
      }
    }
    else {
      Serial.println("Config file dosent Exist");
    }
  }
  else {
    Serial.println("SPIFFS Mount Failed");
  }
}

void spiffs_read() {
  Serial.println("Mounting SPIFFS for READ...");

  if (SPIFFS.begin()) {
    Serial.println("Mounted LittleFS..");
    if (SPIFFS.exists("/config.json")) {
      Serial.println("Config file found..");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("Opened Config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        Serial.print("SPIFFS Data:  "); Serial.println(buf.get());

        DynamicJsonDocument doc1(400);
        DeserializationError error = deserializeJson(doc1, buf.get());
        // Test if parsing succeeds.
        if (error) {
          Serial.print(F("DeserializeJson() failed: "));
          Serial.println(error.f_str());
          return;
        } else {
          Serial.println("Deserialized sucessfully");
        }
        strlcpy(devName, doc1["id1"], sizeof(devName));
        strlcpy(mtr1, doc1["id2"], sizeof(mtr1));
        strlcpy(mtr2, doc1["id3"], sizeof(mtr2));
        strlcpy(mtr3, doc1["id4"], sizeof(mtr3));
        strlcpy(mtr4, doc1["id5"], sizeof(mtr4));
        strlcpy(mtr5, doc1["id6"], sizeof(mtr5));

        configFile.close();
        Serial.print(devName);Serial.print("  "); Serial.print(mtr1);Serial.print("  ");Serial.print(mtr2);Serial.print("  ");Serial.print(mtr3);Serial.print("  ");Serial.print(mtr4);Serial.print("  ");Serial.println(mtr5);
      }
    } else {
      Serial.println("Config json dosent exist");
    }
  } else {
    Serial.println("Failed to mount LittleFS");
  }
}

void checkButton(){
  add_Custom_Param();
  // check for button press
  if ( digitalRead(WIFI_PIN) == LOW ) {
    // poor mans debounce/press-hold, code not ideal for production
    delay(50);
    if( digitalRead(WIFI_PIN) == LOW ){
      Serial.println("Button Pressed");
      // still holding button for 3000 ms, reset settings, code not ideaa for production
      delay(3000); // reset delay hold
      if( digitalRead(WIFI_PIN) == LOW ){
        Serial.println("Button Held");

       // start portal w delay
      Serial.println("Starting config portal");
      
      if (!wm.startConfigPortal(devName,"738738738")) {
        Serial.println("failed to connect or hit timeout");
        delay(3000);
        // ESP.restart();
      } else {
        //if you get here you have connected to the WiFi
        Serial.println("connected...yeey :)");
      }

      }
    }
  }
}
