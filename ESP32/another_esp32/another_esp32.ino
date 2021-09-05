#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic* pCharacteristic = NULL;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
boolean fall = false; 
boolean trigger1=false; 
boolean trigger2=false; 
boolean trigger3=false; 
byte trigger1count=0; 
byte trigger2count=0; 
byte trigger3count=0; 
int angleChange=0;

void mpu_read(){
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission(false);
 Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
 AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
 AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 }

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  
  Wire.write(0);     
  Wire.endTransmission(true);
  Serial.println("Wrote to IMU");
  
  BLEDevice::init("Child GPS Tracker");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("false");
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); 
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  mpu_read();

  ax = (AcX-2050)/16384.00;
  ay = (AcY-77)/16384.00;
  az = (AcZ-1947)/16384.00;
  gx = (GyX+270)/131.07;
  gy = (GyY-351)/131.07;
  gz = (GyZ+136)/131.07;
  
  float Raw_Amp = pow(pow(ax,2)+pow(ay,2)+pow(az,2),0.5);
  int Amp = Raw_Amp * 10;  
  Serial.println(Amp);
  if (Amp<=2 && trigger2==false){ //if AM breaks lower threshold
    trigger1=true;
    Serial.println("TRIGGER 1 ACTIVATED");
    }
    
  if (trigger1==true){
    trigger1count++;
    if (Amp>=12){ //if AM breaks upper threshold
      trigger2=true;
      Serial.println("TRIGGER 2 ACTIVATED");
      trigger1=false; trigger1count=0;
      }
  }
      
  if (trigger2==true){
    trigger2count++;
    angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5); 
    Serial.println("AngleChange : ");
    Serial.println(angleChange);
    if (angleChange>=300 && angleChange<=500){ //if orientation changes
      trigger3=true; trigger2=false; trigger2count=0;
      Serial.println("AngleChange : ");
      Serial.println(angleChange);
      Serial.println("TRIGGER 3 ACTIVATED");
        }
    }
  if (trigger3==true){
      trigger3count++;
      if (trigger3count>=10){ 
        angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5);
        //delay(10);
        Serial.println(angleChange); 
        if ((angleChange>=0) && (angleChange<=150)){ //Check if orientation remains same
            fall=true; trigger3=false; trigger3count=0;
            Serial.println(angleChange);
              }
        else{ 
            trigger3=false; trigger3count=0;
            Serial.println("TRIGGER 3 DEACTIVATED");
        }
      }
    }
  if (fall==true){ 
    Serial.println("FALL DETECTED"); 
    pCharacteristic->setValue("true");
    fall=false;
    }
  if (trigger2count>=6){ 
    trigger2=false; trigger2count=0;
    Serial.println("TRIGGER 2 DECACTIVATED");
    }
  if (trigger1count>=6){ 
    trigger1=false; trigger1count=0;
    Serial.println("TRIGGER 1 DECACTIVATED");
    }
    delay(100);
    
}
