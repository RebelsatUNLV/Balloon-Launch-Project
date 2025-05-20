// CubeSat v1 ASB Code Revision 1.0
// =================================
// ©Copyright 2023 EdgeFlyte, LLC.
// All Rights Reserved. 

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include "SparkFun_ENS160.h"
// #include <Adafruit_BMP085.h> // OLD LIB
// #include <sps30.h> // REMOVED SPS30
// #include <DFRobot_SCD4X.h> // REMOVED SCD4X
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>


Adafruit_SHT31 sht31 = Adafruit_SHT31();
SparkFun_ENS160 ens; 
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
// struct sps30_measurement m; // REMOVED SPS30
// DFRobot_SCD4X SCD4X(&Wire, SCD4X_I2C_ADDR); // REMOVED SCD4X


// ASB Variables
#define LED     2
bool ist[128];


struct baseSens {
  bool sht3x, bmp180, ens160; // REMOVED sps30, scd40
};
baseSens bs;

struct baseData {
  float t1, t2, p1, rh, pa1;
  uint32_t aqi, tvoc, eco2;
  // uint16_t co2; // REMOVED CO2 (from SCD4X)
  int ensFlags;
  
};
baseData bd;


void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(1, INPUT);
  UCSR0B &= ~_BV(TXEN0);
  
  bootLED();
  
  scani2c();
  // Check base sensors
  for(int i=0; i<128; i++){
    if(ist[i]){
      Serial.print("DTC 0x");
      if(i < 16) Serial.print("0");
      Serial.println(i, HEX);
    }
  }
  if(ist[0x77]) bs.bmp180 = true;
  if(ist[0x44]) bs.sht3x = true;
  if(ist[0x53]) bs.ens160 = true;
  

  // sensirion_i2c_init(); // REMOVED (was for SPS30)

  // REMOVED SPS30 Initialization Block
  // if(sps30_probe() != 0) {
  //   Serial.print("SPS sensor probing failed\n");
  //   delay(500);
  //   bs.sps30 = false;
  // }
  // uint16_t ret = sps30_set_fan_auto_cleaning_interval_days(4);
  // if (ret) {
  //   Serial.print("error setting the auto-clean interval: ");
  //   Serial.println(ret);
  // }
  // ret = sps30_start_measurement();
  // if (ret < 0) {
  //   Serial.print("error starting measurement\n");
  // }

  if (bmp.begin()) {
    flashLED();
  }
  if(ens.begin() ){
    flashLED();
  }
  if(sht31.begin()){
    flashLED();
  }
  // REMOVED SCD4X Initialization and Configuration Block
  // if(SCD4X.begin() ){
  //   flashLED();
  // }
  // delay(2000); // This delay was part of the SCD4X init sequence
  // SCD4X.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);
  // SCD4X.setTempComp(4.0);
  // //  float temp = 0;
  // //  temp = SCD4X.getTempComp();
  // //  Serial.print("The current temperature compensation value : ");
  // //  Serial.print(temp);
  // //  Serial.println(" C");
  // // SCD4X.setSensorAltitude(bmp.readAltitude());
  // SCD4X.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);
  
  if( ens.setOperatingMode(SFE_ENS160_RESET)){ // This block remains
    Serial.println("Ready.");
    delay(100);
    ens.setOperatingMode(SFE_ENS160_STANDARD);
    delay(100);

    int ensStatus = ens.getFlags();
  //  Serial.print("Gas Sensor Status Flag (0 - Standard, 1 - Warm up, 2 - Initial Start Up): ");
    Serial.println(ensStatus);
  }
}

uint32_t lowPollRate, highPollRate, txPacketTime = 0;

void loop() {
  if(lowPollRate + 1000 <= millis()){
    getSHT3x();
    getENS160();
    // pollsps(); // REMOVED
    // getSCD4x(); // REMOVED
    lowPollRate = millis();
  }

  if(highPollRate + 500 <= millis()){
    getBMP180();
    
    highPollRate = millis();
  } 

  if(txPacketTime + 1000 <= millis()){
    txMainPkt();
    txPacketTime = millis();
  }
}


void bootLED(){
  for(int i=0; i<10; i++){
    digitalWrite(LED, 1);
    delay(50);
    digitalWrite(LED, 0);
    delay(50);
  }
}


void flashLED(){
  digitalWrite(LED, 1);
  delay(100);
  digitalWrite(LED, 0);
  delay(500);
  
}



void scani2c(){
  for(byte a = 1; a < 127; a++ ){
    Wire.beginTransmission(a);
    byte e = Wire.endTransmission();
    if (e == 0) ist[a] = true;
    else  ist[a] = false;
  }  
}


// This txMainPkt function is from your provided code for this modification request.
// It only sends T1, T2, RH, P1.
void txMainPkt(){
  UCSR0B |= _BV(TXEN0);
  digitalWrite(LED, 1);

  char bu[150];
  //                  | T1 | T2 | RH | P1 |
  sprintf(bu, "$ASB,0,%05d,%05d,%04d,%05d*",
          (int)((bd.t1+40)*100), (int)((bd.t2+40)*100), (int)(bd.rh*10),
          (int)(bd.p1/10));
  Serial.println(bu);
  delay(100);
  UCSR0B &= ~bit (TXEN0);
  // UCSR0B &= _BV(TXEN0);
  // digitalWrite(1, 0);
  // pinMode(1, INPUT);
  digitalWrite(LED, 0);
}


// Main Sens Functions
void getSHT3x(){
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  if (! isnan(t)) bd.t1 = t;
  if (! isnan(h)) bd.rh = h;
}

void getBMP180(){
  sensors_event_t event;
  bmp.getEvent(&event);

  if(event.pressure)
  {
    float temperature;
    bmp.getTemperature(&temperature);
    bd.t2 = temperature;

    bd.p1 = event.pressure;
    // Note: bd.pa1 is in struct baseData but not calculated/updated here in this version of getBMP180
  }
}

void getENS160(){

  if( ens.checkDataStatus() ){
//    Serial.print("Air Quality Index (1-5) : ");
//    Serial.println(ens.getAQI());
    bd.aqi = ens.getAQI();

//    Serial.print("Total Volatile Organic Compounds: ");
//    Serial.print(ens.getTVOC());
//    Serial.println("ppb");
    bd.tvoc = ens.getTVOC();

//    Serial.print("CO2 concentration: ");
//    Serial.print(ens.getECO2());
//    Serial.println("ppm");
    bd.eco2 = ens.getECO2();

//    Serial.print("Gas Sensor Status Flag (0 - Standard, 1 - Warm up, 2 - Initial Start Up): ");
//    Serial.println(ens.getFlags());
    bd.ensFlags = ens.getFlags();

//    Serial.println();
  }
}


// REMOVED pollsps() function
// void pollsps(){
// //  if(!bs.sps30) return;
//   ...
// }

// REMOVED getSCD4x() function
// void getSCD4x(){
//   if(SCD4X.getDataReadyStatus()) {
//     ...
//   }
// }