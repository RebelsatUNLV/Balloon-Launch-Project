// CubeSat v1 ASB Code Revision 1.0
// =================================
// ©Copyright 2023 EdgeFlyte, LLC.
// All Rights Reserved. 

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include "SparkFun_ENS160.h"
#include <Adafruit_BMP085.h>
#include <sps30.h>
#include <DFRobot_SCD4X.h>
//barometer libary dependencies
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#define ALTITUDE 1011.9	

//barometer macros begin
// ======== CALIBRATION SETTINGS (MODIFY THESE) ========
// Enter your known altitude in meters
#define KNOWN_ALTITUDE 743.0  // Replace with your actual altitude

// Enter your local sea level pressure in hPa (check a weather service)
// Standard is 1013.25 hPa, but using local value improves accuracy
#define LOCAL_SEA_LEVEL_PRESSURE 1011.6  // Replace with local value

// Pressure offset in hPa (adjust based on comparison with reference)
#define PRESSURE_OFFSET 0.0  // Fine-tune as needed after testing
// =====================================================
//baromete macros end


Adafruit_SHT31 sht31 = Adafruit_SHT31();
SparkFun_ENS160 ens; 
Adafruit_BMP085 bmp;
struct sps30_measurement m;
DFRobot_SCD4X SCD4X(&Wire, SCD4X_I2C_ADDR);


// ASB Variables
#define LED     2
bool ist[128];


struct baseSens {
  bool sht3x, bmp180, ens160, sps30, scd40;
};
baseSens bs;

struct baseData {
  float t1, temperature, pressureKPa, rh, altitude;
  uint32_t aqi, tvoc, eco2;
  uint16_t co2;
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
     // if(i < 16) Serial.print("0");
      Serial.println(i, HEX);
    }
  }
  if(ist[0x77]) bs.bmp180 = true;
  if(ist[0x44]) bs.sht3x = true;
  if(ist[0x53]) bs.ens160 = true;
  

  sensirion_i2c_init();

  if(sps30_probe() != 0) {
    Serial.print("SPS sensor probing failed\n");
    delay(500);
    bs.sps30 = false;
  }
  uint16_t ret = sps30_set_fan_auto_cleaning_interval_days(4);
  if (ret) {
    Serial.print("error setting the auto-clean interval: ");
    Serial.println(ret);
  }
  ret = sps30_start_measurement();
  if (ret < 0) {
    Serial.print("error starting measurement\n");
  }

  if (bmp.begin()) {
    flashLED();
  }
  if(ens.begin() ){
    flashLED();
  }
  if(sht31.begin()){
    flashLED();
  }
  if(SCD4X.begin() ){
    flashLED();
  }
  delay(2000);
  SCD4X.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);
  SCD4X.setTempComp(4.0);
//  float temp = 0;
//  temp = SCD4X.getTempComp();
//  Serial.print("The current temperature compensation value : ");
//  Serial.print(temp);
//  Serial.println(" C");
  SCD4X.setSensorAltitude(bmp.readAltitude(ALTITUDE*100));
  SCD4X.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);
  


  
  
  if( ens.setOperatingMode(SFE_ENS160_RESET)){
    Serial.println("Ready.");
    delay(100);
    ens.setOperatingMode(SFE_ENS160_STANDARD);
    delay(100);

    int ensStatus = ens.getFlags();
  //  Serial.print("Gas Sensor Status Flag (0 - Standard, 1 - Warm up, 2 - Initial Start Up): ");
    Serial.println(ensStatus);
  }
  //barometer
   while (!Serial) delay(10); // Wait for serial port to open
  
  Serial.println("Pressure Sensor with Compile-Time Calibration");
  Serial.println("");
  //bread board testing
  // Initialize I2C
  //Wire.setSDA(0);  // GPIO 0 for SDA
  //Wire.setSCL(1);  // GPIO 1 for SCL
  //Wire.begin();
  
  /* Initialize the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
   // Serial.print("Ooops, no BMP085/180 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //end barometer
}

uint32_t lowPollRate, highPollRate, txPacketTime = 0;

void loop() {
  if(lowPollRate + 1000 <= millis()){
    getSHT3x();
    getENS160();
    pollsps();
    getSCD4x();
    lowPollRate = millis();
  }

  if(highPollRate + 500 <= millis()){
    getBMP180();
    
    highPollRate = millis();
  } 

  if(txPacketTime + 1000 <= millis()){
    txMainPkt();
    txPM2Pkt();
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



void txMainPkt(){
  UCSR0B |= _BV(TXEN0);
  digitalWrite(LED, 1);

  char bu[150];
  //                  | T1 | T2 | RH | P1 | PA |AQI |TVOC|ECO2|CO2
  sprintf(bu, "$ASB,0,%05d,%05d,%04d,%05d,%05d,%01d,%06d,%05d,%05d,*",
          (int)((bd.t1+40)*100), (int)((bd.temperature)), (int)(bd.rh*10),
          (int)(bd.pressureKPa), (int)(bd.altitude),
          (int)bd.aqi, (int)bd.tvoc, (int)bd.eco2, (int)bd.co2);
  Serial.println(bu);
  delay(100);
  UCSR0B &= ~bit (TXEN0);
  // UCSR0B &= _BV(TXEN0);
  // digitalWrite(1, 0);
  // pinMode(1, INPUT);
  digitalWrite(LED, 0);
}

void txPM2Pkt(){
  digitalWrite(LED, 1);
  UCSR0B |= _BV(TXEN0);
  char bu[150];
  //                  |PM 1.0 - 10.0     |NC 0.5 - 10.0           |TYP Part Size
  sprintf(bu, "$ASB,1,%05d,%05d,%05d,%05d,%06d,%06d,%06d,%06d,%06d,%03d,*",
          (int)(m.mc_1p0 * 10),(int)(m.mc_2p5 * 10),(int)(m.mc_4p0 * 10),(int)(m.mc_10p0 * 10),
          (int)((m.nc_0p5)*10), (int)((m.nc_1p0  - m.nc_0p5)*10), (int)((m.nc_2p5  - m.nc_1p0)*10), 
          (int)((m.nc_4p0  - m.nc_2p5)*10), (int)((m.nc_10p0 - m.nc_4p0)*10),
          (int)(m.typical_particle_size * 100)
          );
  Serial.println(bu);
  delay(100);
  // UCSR0B &= _BV(TXEN0);
  UCSR0B &= ~bit (TXEN0);
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


    //barometer begin
    /* Get a new sensor event */ 
  sensors_event_t event;
  //bmp.getEvent(&event);
 
  /* Display the results with calibration applied */
  if (event.pressure)
  {
    // Apply calibration offset to pressure
    float calibratedPressure = event.pressure + PRESSURE_OFFSET;
    
    // Convert to kPa for display
    float pressureKPa = calibratedPressure / 10.0;
    
    // Display raw and calibrated pressure
   // Serial.print("Raw Pressure:      ");
  //  Serial.print(event.pressure / 10.0);
   // Serial.println(" kPa");
    
   // Serial.print("Calibrated Pressure: ");
   // Serial.print(pressureKPa);
  //  Serial.println(" kPa");
    
    /* Get the current temperature */
    float temperature;
    //bmp.getTemperature(&temperature);
   // Serial.print("Temperature:        ");
   // Serial.print(temperature);
   // Serial.println(" C");

    /* Calculate altitude using the calibrated pressure and local sea level reference */
    float altitude = 44330.0 * (1.0 - pow(calibratedPressure / LOCAL_SEA_LEVEL_PRESSURE, 0.190294957184));
    
   // Serial.print("Altitude:           "); 
   // Serial.print(altitude); 
   // Serial.println(" m");
    
    // Calculate sea level pressure using the known altitude
    // This is useful for verifying your calibration
    float calculatedSeaLevel = calibratedPressure / pow(1.0 - (KNOWN_ALTITUDE / 44330.0), 5.255);
    Serial.print("Calculated Sea Level: ");
    Serial.print(calculatedSeaLevel/10);
   Serial.println(" kPa");
    
   // Serial.println("");
  bd.temperature = temperature;
  bd.pressureKPa = calculatedSeaLevel;
  bd.altitude = altitude;

  }
  else
  {
    Serial.println("Sensor error");
  }
  delay(1000);
  //barometer end
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





void pollsps(){
//  if(!bs.sps30) return;
  
  
  uint16_t data_ready;
  int16_t ret;

  ret = sps30_read_data_ready(&data_ready);
  if (ret < 0) {
    Serial.print("#error reading data-ready flag: ");
    Serial.println(ret);
    return;
  } else if (!data_ready){
    Serial.print("#data not ready, no new measurement available\n");
    return;
  }
  ret = sps30_read_measurement(&m);
  if (ret < 0) {
    Serial.print("#error reading measurement\n");
    return;
  } else {

//    Serial.print("PM  1.0: ");
//    Serial.println(m.mc_1p0);
//    Serial.print("PM  2.5: ");
//    Serial.println(m.mc_2p5);
//    Serial.print("PM  4.0: ");
//    Serial.println(m.mc_4p0);
//    Serial.print("PM 10.0: ");
//    Serial.println(m.mc_10p0);
//
//    Serial.print("Typical partical size: ");
//    Serial.println(m.typical_particle_size);
//
//    Serial.print(m.nc_0p5);
//    Serial.print(" ");
//    Serial.print(m.nc_1p0  - m.nc_0p5);
//    Serial.print(" ");
//    Serial.print(m.nc_2p5  - m.nc_1p0);
//    Serial.print(" ");
//    Serial.print(m.nc_4p0  - m.nc_2p5);
//    Serial.print(" ");
//    Serial.print(m.nc_10p0 - m.nc_4p0);
//    Serial.println();
  }
}

void getSCD4x(){
  if(SCD4X.getDataReadyStatus()) {
    DFRobot_SCD4X::sSensorMeasurement_t data;
    SCD4X.readMeasurement(&data);
//
//    Serial.print("Carbon dioxide concentration : ");
//    Serial.print(data.CO2ppm);
//    Serial.println(" ppm");
    bd.co2 = data.CO2ppm;
//
//    Serial.print("Environment temperature : ");
//    Serial.print(data.temp);
//    Serial.println(" C");
//
//    Serial.print("Relative humidity : ");
//    Serial.print(data.humidity);
//    Serial.println(" RH");

//    Serial.println();
  }
}
