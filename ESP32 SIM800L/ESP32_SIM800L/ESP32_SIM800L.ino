const char apn[]      = "dialogbb"; 
const char gprsUser[] = ""; 
const char gprsPass[] = ""; 
const char simPIN[]   = ""; 
const char server[] = "childgps.000webhostapp.com"; // domain name
const char resource[] = "/Index.php?lat=7.7769&lon=81.6042";  // resource path
const int  port = 80;                             


// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

 
#define SerialMon Serial
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      
#define TINY_GSM_RX_BUFFER   1024  

#define I2C_SDA_2            18
#define I2C_SCL_2            19

#include <Wire.h>
#include <TinyGsmClient.h>
#include <TinyGPS++.h>
#include "HardwareSerial.h"

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif


TwoWire I2CPower = TwoWire(0);

TinyGsmClient clientx(modem,0);
TinyGsmClient clienty(modem,1);

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

TinyGPSPlus gps;
double latitude , longitude;
HardwareSerial SerialGPS(1);

bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37);
  } else {
    I2CPower.write(0x35); 
  }
  return I2CPower.endTransmission() == 0;
}

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLEDevice.h"

static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID    charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

String beaconID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
String newValue = "false";
String fallDetected = "false";
String fallDelay = "false";
const int DeviceID = 15512;
const String password = "1234";
unsigned long lastMillis;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    pClient->connect(myDevice);  
    Serial.println(" - Connected to server");

    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } 
  } 
}; 

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 14,27);
  
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
  
  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));
 
  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  
  SerialMon.println("Initializing modem...");
  modem.restart();
 
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork(240000L)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }

  SerialMon.println(" OK");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");
}

void loop() {

  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("connected to the BLE Server.");
    } else {
      Serial.println("failed to connect to the server");
    }
    doConnect = false;
  }

  if (connected) {
    
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("Fall Detected : ");
   
    String fall = value.c_str();
    Serial.println(fall);
    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
      if(fall == "true"){
        fallDetected = "true";
        fallDelay = "true";
      }
  }else if(doScan){
    BLEDevice::getScan()->start(0);  
  }

    SerialMon.print("Connecting to ");
    SerialMon.print(server);
    if (!clientx.connect(server, port)) {
      SerialMon.println(" fail");
    }
    else {
      SerialMon.println(" OK");

      while (Serial2.available() >0) {
          gps.encode(Serial2.read());
        }

      latitude = gps.location.lat();
      longitude = gps.location.lng();
     
      // Making an HTTP Get request
    String httpRequestData = "/Index.php?lat="+String(latitude,6)+"&lon="+String(longitude,6)+"&fall="+fallDetected+"&deviceID="+DeviceID+"&password="+password+"&beaconID="+beaconID;
    
      SerialMon.println("Performing HTTP GET request...");
      clientx.print(String("GET ") + httpRequestData + " HTTP/1.1\r\n");
      clientx.print(String("Host: ") + server + "\r\n");
      clientx.print("Connection: close\r\n\r\n");
      clientx.println();

      unsigned long timeout = millis();
      while (clientx.connected() && millis() - timeout < 10000L) {
        
        while (clientx.available()) {
          char c = clientx.read();
          SerialMon.print(c);
          if(c == 'T'){
            fallDetected = "false";
          }
          timeout = millis();
        }
        
      }

      if(fallDelay == "true"){
        delay(3000);
      }

    if (millis() - lastMillis >= 5*60*1000UL)
      {
        lastMillis = millis();  //get ready for the next iteration
        SerialMon.println("Performing HTTP GET request to SQL...");
        clienty.connect(server, port);
        String httpRequestHistory = "/sendHistory.php?lat="+String(latitude,6)+"&lon="+String(longitude,6)+"&deviceID="+DeviceID;
        clienty.print(String("GET ") + httpRequestHistory + " HTTP/1.1\r\n");
        clienty.print(String("Host: ") + server + "\r\n");
        clienty.print("Connection: close\r\n\r\n");
        clienty.println();

      }

    } 
  
}
