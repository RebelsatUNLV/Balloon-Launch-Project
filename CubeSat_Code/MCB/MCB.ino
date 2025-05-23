// CubeSat v1 MCB Code Revision 1.0
// =================================
// ©Copyright 2023 EdgeFlyte, LLC.
// All Rights Reserved.
// Working code
#include "Arduino.h"
#include <assert.h>
#include <SPI.h>
#include <Wire.h>
#include "SD.h"
#include "RF24.h"
#include "FastIMU.h"
#include "Adafruit_SHT31.h"
#include <Adafruit_GPS.h>
#include "Madgwick.h"

// Global variables for system state and hardware control
File dlog;
File root;
double btLETime = 0;
bool sdInit = false;
bool sendPKTtoBLE = true;
uint8_t csID = 0xA3; // Set to CS ID
// NRF radio module configuration 
#define CHANNEL_START 0
#define CHANNEL_STOP 127
RF24 radio(11, 15);
#define PACKET_SIZE 32
#define SD_CS 17
#define BUZZER 3
#define GP9 9
#define GP10 10
#define GP21 21
#define GP22 22
#define GP26 26
#define GP27 27
#define L1 6
#define L2 7
#define L3 8

#define LOCAL_SEA_LEVEL_PRESSURE 1011.6
void flashRed()
{
    digitalWrite(L2, 1);
    delay(10);
    digitalWrite(L2, 0);
}

// Makes LED flash green
void flashGreen()
{
    digitalWrite(L1, 1);
    delay(10);
    digitalWrite(L1, 0);
}

// MPU 9250 (IMU) configuration and data structures
#define IMU_ADDRESS 0x68
#define PERFORM_CALIBRATION
MPU9250 IMU;
calData calib = {0};
AccelData accelData;
GyroData gyroData;
MagData magData;
Madgwick filter;

// SHT 3x (digital temperature and humidity sensor)
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// GPS
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false

// State of each sensor
struct sensorStates
{
    bool sht3x, nrf24, mpu9250, sdmmc, gps = false;
    int err = 0;
};
sensorStates sState;

// Data from each sensor
struct sensorData
{
    float ax, ay, az;
    float accelMagnitude;
    float gx, gy, gz;
    float mx, my, mz;
    float t1, t2, rh;
    double lat, lng;
    float alt, hdop;
    int sats;
};
sensorData sData;

// System info 
struct sysInfo
{
    int xvCH = 11;
    uint8_t address[6] = {45, 46, 45, 46, 45};
    int bms_err;
    bool bms_ch1, bms_ch2, bms_ch3, bms_ch4;
    float batV, batPct, sol1V, sol2V;
    bool rbf, sd;
};
sysInfo sys;

// Atmospheric data readings
struct atmosphericData
{
    float t1, t2, rh, p1, pa1;
    int aqi, tvoc, eco2, co2;
    float pm10, pm25, pm40, pm100;
    float nc05, nc10, nc25, nc40, nc100;
    float typ;
};
atmosphericData ad;

String temp;

// CubeSat Comms Protocol
// Initializes all hardware and sensors
#define SBUS_ADDR 0x00
bool cad[CHANNEL_STOP - CHANNEL_START + 1];
void setup()
{
    delay(5000);
    pinMode(BUZZER, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SD_CS, OUTPUT);
    pinMode(2, INPUT);
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(L3, OUTPUT);
    digitalWrite(L1, 0);
    digitalWrite(L2, 0);
    digitalWrite(L3, 0);
    startupTone();
    Serial.begin(115200); // computer output
    Serial1.begin(9600);  // Internal output
    Serial2.setRX(5);
    Serial2.setTX(4);
    Serial2.begin(9600);
    Wire.setSDA(12);
    Wire.setSCL(13);
    Wire.begin();
    SPI.setRX(16);
    SPI.setTX(19);
    SPI.setSCK(18);
    SPI.begin();
    delay(1000);
    if (SD.begin(SD_CS))
    {
        sState.sdmmc = true;
        Serial.println("SD Card Init Success!");
        Serial2.println("SD Card Init Success!");
        sdInit = true;
        root = SD.open("/");
        printDirectory(root, 0);
    }
    // NRF 24L01
    if (!radio.begin(&SPI))
    {
        Serial2.println("NRF24 Init Failure.");
    }
    radio.setChannel(62);
    radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.
    radio.setPayloadSize(32);
    radio.openWritingPipe(sys.address);
    radio.openReadingPipe(1, sys.address);
    radio.setDataRate(RF24_1MBPS);
    radio.stopListening();
    //
    radio.printDetails();
    radio.printPrettyDetails();
    // Init GPS
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    initMPU9250();
    // Need to begin/initialize the Madgwick filter for it to filter and update values, initial beta setting of 0.1
    // The beta determines how much the filter trusts accel./magnet. data vs. gyroscope, so initially there won't be 
    // much trust between the sensors, but then beta will reupdate after initialization and calibration
    filter.begin(0.1f); 
    initSHTx();
    delay(2000);
    digitalWrite(LED_BUILTIN, 1);
    tone(BUZZER, 500);
    delay(1000);
    noTone(BUZZER);
}

double mpuTime, xvTime, orientationTxTime, asbPktTime;
static unsigned long lastPrintTime = 0;
String t2;
void loop()
{
    // Print all sensor data every second
    if (millis() - lastPrintTime >= 1000)
    {
        lastPrintTime = millis();
        // Print comprehensive sensor report
        Serial.println("\n=== FULL SENSOR REPORT ===");
        // IMU Data
        Serial.print("Accel (m/s²): ");
        Serial.print(sData.ax);
        Serial.print(", ");
        Serial.print(sData.ay);
        Serial.print(", ");
        Serial.print(sData.az);
        Serial.print(", ");
        Serial.println(sData.accelMagnitude);
        Serial.print("Gyro (deg/s): ");
        Serial.print(sData.gx);
        Serial.print(", ");
        Serial.print(sData.gy);
        Serial.print(", ");
        Serial.println(sData.gz);
        Serial.print("Mag (μT): ");
        Serial.print(sData.mx);
        Serial.print(", ");
        Serial.print(sData.my);
        Serial.print(", ");
        Serial.println(sData.mz);
        // Environmental
        Serial.print("Temp/Humidity: ");
        Serial.print(sData.t1);
        Serial.print("°C, ");
        Serial.print(sData.rh);
        Serial.println("%");
        // Atmospheric
        Serial.print("ASB Temp: ");
        Serial.print(ad.t1);
        Serial.print("°C, ");
        Serial.print(ad.t2);
        Serial.println("°C");
        Serial.print("Pressure: ");
        Serial.print(ad.p1);
        Serial.print("hPa, Alt: ");
        Serial.print(ad.pa1);
        Serial.println("m");
        // Battery System
        Serial.print("Battery: ");
        Serial.print(sys.batV);
        Serial.print("V (");
        Serial.print(sys.batPct);
        Serial.print("%), Solar: ");
        Serial.print(sys.sol1V);
        Serial.print("V, ");
        Serial.print(sys.sol2V);
        Serial.println("V");
        // GPS
        Serial.print("GPS: ");
        Serial.print(GPS.latitudeDegrees, 6);
        Serial.print(", ");
        Serial.print(GPS.longitudeDegrees, 6);
        Serial.print(", Alt: ");
        Serial.print(GPS.altitude);
        Serial.print("m, Sats: ");
        Serial.println(GPS.satellites);
        Serial.println("========================");
    }
    if (digitalRead(2) == 0)
    {
        Serial2.println("Detected RBF Tag. Suspending System...");
        tone(BUZZER, 1250, 100);
        delay(100);
        noTone(BUZZER);
        delay(100);
        tone(BUZZER, 1250, 100);
        delay(100);
        noTone(BUZZER);
        while (digitalRead(2) == 0)
        {
            digitalWrite(L3, 1);
            delay(2000);
            digitalWrite(L3, 0);
            delay(2000);
        }
    }
    if (millis() - 10 >= mpuTime)
    {
        readMPU9250();
        mpuTime = millis();
    }
    if (millis() - 500 >= xvTime)
    {
        tx_packet();
        xvTime = millis();
    }
    if (millis() - 2000 >= asbPktTime)
    {
        txASBPacket();
        asbPktTime = millis();
    }
    if (millis() - 100 >= orientationTxTime)
    {
        txOrientationPkt();
        orientationTxTime = millis();
    }
    // Write data to file
    if (millis() - 1000 >= btLETime)
    {

        // txBLE();
        // usbDebug();
        readSHTx();
        logData(); // ADD THIS LINE
        btLETime = millis();
    }
    if (millis() - 1000 >= btLETime)
    {
        // txBLE();
        // usbDebug();
        // scani2c();
        readSHTx();
        btLETime = millis();
    }
    char c = GPS.read();
    if (GPSECHO)
        if (c)
            Serial.print(c);
    if (GPS.newNMEAreceived())
    {
        // Serial.print(GPS.lastNMEA());
        if (!GPS.parse(GPS.lastNMEA()))
            return;
    }
    // End GPS
    if (Serial2.available() > 0)
    {
        String rs = Serial2.readStringUntil('\n');
        String t;
        if (rs.startsWith("$BMS"))
        {
            t = rs.substring(5, 6); // ERR
            sys.bms_ch1 = t.toInt();
            t = rs.substring(7, 8); // LDO1
            sys.bms_ch1 = t.equals("1");
            t = rs.substring(9, 10); // LDO2
            sys.bms_ch2 = t.equals("1");
            t = rs.substring(11, 12); // LDO3
            sys.bms_ch3 = t.equals("1");
            t = rs.substring(13, 14); // LDO4
            sys.bms_ch4 = t.equals("1");
            t = rs.substring(15, 19); // BatV
            sys.batV = t.toFloat();
            t = rs.substring(20, 26); // BatPCT
            sys.batPct = t.toFloat();
            t = rs.substring(27, 31); // Sol 1 V
            sys.sol1V = t.toFloat();
            t = rs.substring(32, 36); // Sol 2 V
            sys.sol2V = t.toFloat();
        }
        if (rs.startsWith("$ASB,0"))
        {
            // Temperature 1
            t2 = rs;
            t = rs.substring(7, 12);
            int i = t.toInt();
            ad.t1 = ((float)i / 100) - 40;
            // Temperature 2
            t = rs.substring(13, 18);
            i = t.toInt();
            ad.t2 = ((float)i / 100) - 40;
            // sData.t2 = ad.t2; // <<<<<<<<<<<< FIX: Assign ad.t2 to sData.t2
            // Humidity
            t = rs.substring(20, 24); // AD RH (corrected position)
            i = t.toInt();
            ad.rh = ((float)i / 10);
            // Pressure (FIXED)
            t = rs.substring(25, 30); // Correct substring for pressure
            i = t.toInt();
            ad.p1 = (float)i * 10; // Proper kPa conversion
            // Pressure Altitude (FIXED)
            // t = rs.substring(26, 31); // Now unique substring
            ad.pa1 = 44330.0 * (1.0 - pow(ad.p1 / LOCAL_SEA_LEVEL_PRESSURE, 0.190294957184));
        }
    }
}

// This function initializes and calibrates the MPU9250 (IMU)
void initMPU9250()
{
    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0)
    {
        Serial2.print("Error initializing IMU: ");
        Serial2.println(err);
        return;
    }
    sState.mpu9250 = true;
#ifdef PERFORM_CALIBRATION
    tone(BUZZER, 1200, 100);
    delay(100);
    noTone(BUZZER);
    delay(100);
    tone(BUZZER, 1200, 100);
    delay(100);
    noTone(BUZZER);
    delay(100);
    tone(BUZZER, 1200, 100);
    delay(100);
    noTone(BUZZER);
    delay(100);
    Serial2.println("FastIMU calibration & data example");
    if (IMU.hasMagnetometer())
    {
        delay(1000);
        Serial2.println("Move IMU in figure 8 pattern until done.");
        delay(3000);
        IMU.calibrateMag(&calib);
        Serial2.println("Magnetic calibration done!");
    }
    tone(BUZZER, 1400);
    Serial2.println("Keep IMU level.");
    delay(2000);
    IMU.calibrateAccelGyro(&calib);
    Serial2.println("Calibration done!");
    // Print calibration results
    Serial2.println("Accel biases X/Y/Z: ");
    Serial2.print(calib.accelBias[0]);
    Serial2.print(", ");
    Serial2.print(calib.accelBias[1]);
    Serial2.print(", ");
    Serial2.println(calib.accelBias[2]);
    Serial2.println("Gyro biases X/Y/Z: ");
    Serial2.print(calib.gyroBias[0]);
    Serial2.print(", ");
    Serial2.print(calib.gyroBias[1]);
    Serial2.print(", ");
    Serial2.println(calib.gyroBias[2]);
    if (IMU.hasMagnetometer())
    {
        Serial2.println("Mag biases X/Y/Z: ");
        Serial2.print(calib.magBias[0]);
        Serial2.print(", ");
        Serial2.print(calib.magBias[1]);
        Serial2.print(", ");
        Serial2.println(calib.magBias[2]);
        Serial2.println("Mag Scale X/Y/Z: ");
        Serial2.print(calib.magScale[0]);
        Serial2.print(", ");
        Serial2.print(calib.magScale[1]);
        Serial2.print(", ");
        Serial2.println(calib.magScale[2]);
    }
    noTone(BUZZER);
    // Reinitialize IMU with updated calibration data
    IMU.init(calib, IMU_ADDRESS);
#endif
}

// This function logs data from all the sensors to a CSV file on the SD card
void logData()
{
    if (!sdInit)
        return;
    dlog = SD.open("/datalog.csv", FILE_WRITE);
    if (dlog)
    {
        // Write header if file is empty
        if (dlog.size() == 0)
        {
            dlog.println("Timestamp,Lat,Lon,Alt (GPS),Temp1 (C),Temp2 (C),RH (%),Pressure (hPa),AccelX (m/s²),AccelY (m/s²),AccelZ (m/s²), Alt (Bar)");
        }
        // Calculate timestamp
        unsigned long totalSeconds = millis() / 1000;
        int seconds = totalSeconds % 60;
        int totalMinutes = totalSeconds / 60;
        int minutes = totalMinutes % 60;
        int hours = totalMinutes / 60;
        // Write timestamp
        if(hours < 10) dlog.print("0");
        dlog.print(hours);
        dlog.print(":");
        if(minutes < 10) dlog.print("0");
        dlog.print(minutes);
        dlog.print(":");
        if(seconds < 10) dlog.print("0");
        dlog.print(seconds);
        dlog.print(",");
        // Check GPS fix before logging
        dlog.print(GPS.latitudeDegrees, 6);
        dlog.print(",");
        dlog.print(GPS.longitudeDegrees, 6);
        dlog.print(",");
        dlog.print(GPS.altitude);
        // Continue with other data
        dlog.print(",");
        dlog.print(ad.t1); // Temperature (°C)
        dlog.print(",");
        dlog.print(ad.t2,2);
        dlog.print(",");
        dlog.print(sData.rh); // Humidity (%)
        dlog.print(",");
        dlog.print(ad.p1, 2); // Pressure (kPa with 2 decimals)
        // IMU (Accelerometer, gyroscope, magnetometer) 
        dlog.print(",");
        dlog.print(sData.t1); // Temperature (°C)
        dlog.print(",");
        dlog.print(ad.t2,2);
        dlog.print(",");
        dlog.print(sData.rh); // Humidity (%)
        dlog.print(",");
        dlog.print(ad.p1, 2); // Pressure (kPa with 2 decimals)
        dlog.print(",");
        dlog.print(accelData.accelX);
        dlog.print(",");
        dlog.print(accelData.accelY);
        dlog.print(",");
        dlog.print(accelData.accelZ);
        dlog.print(",");
        dlog.println(ad.pa1);
        dlog.close();
    }
}

// This function reads the IMU data
void readMPU9250()
{
    IMU.update();
    IMU.getAccel(&accelData);
    sData.ax = accelData.accelX;
    sData.ay = accelData.accelY;
    sData.az = accelData.accelZ;
    sData.accelMagnitude = sqrt(sData.ax * sData.ax + sData.ay * sData.ay + sData.az * sData.az);
    IMU.getGyro(&gyroData);
    sData.gx = gyroData.gyroX;
    sData.gy = gyroData.gyroY;
    sData.gz = gyroData.gyroZ;
    if (IMU.hasMagnetometer())
    {
        IMU.getMag(&magData);
        sData.mx = magData.magX;
        sData.my = magData.magY;
        sData.mz = magData.magZ;
        // The following lines with filter.update cannot be commented out or skipped if the magnetometer is not detected, 
        // if they are then the filter will not update, the data won't update, and the values won't reflect true orientation
        filter.update(gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ, 
        accelData.accelX, accelData.accelY, accelData.accelZ, 
        magData.magX, magData.magY, magData.magZ);
    } else {
        // filter.update is not the same as filter.updateIMU, filter.updateIMU is specifically designed for when 
        // "magnetometer measurement invalid (avoids NaN in magnetometer normalisation)"
        filter.updateIMU(gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ, 
        accelData.accelX, accelData.accelY, accelData.accelZ);
    }
}

// This function initializes the temperature and humidity sensor
void initSHTx()
{
    if (!sht31.begin(0x44))
    {
        Serial2.println("Couldn't find SHT31");
        return;
    }
    sState.sht3x = true;
}

// This function reads the temperature and humidity sensor
void readSHTx()
{
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    if (!isnan(t))
    {
        sData.t1 = t;
    }
    else
    {
        sState.err++;
        Serial2.println("Failed to read temperature");
    }
    if (!isnan(h))
    {
        sData.rh = h;
    }
    else
    {
        sState.err++;
        Serial2.println("Failed to read humidity");
    }
}

// This function scans I2C bus and updates i2cStats with detected addresses 
bool i2cStats[128];
void scani2c()
{
    for (byte a = 1; a < 127; a++)
    {
        Wire.beginTransmission(a);
        byte e = Wire.endTransmission();
        if (e == 0)
            i2cStats[a] = true;
        else
            i2cStats[a] = false;
    }
}

// This function plays a startup tone sequence via the buzzer
void startupTone()
{
    tone(BUZZER, 650, 250);
    delay(250);
    tone(BUZZER, 750, 250);
    delay(250);
    tone(BUZZER, 1000, 750);
    delay(750);
    tone(BUZZER, 850, 100);
    delay(100);
    tone(BUZZER, 1000, 100);
    delay(100);
    tone(BUZZER, 850, 100);
    delay(100);
    tone(BUZZER, 1000, 100);
    delay(100);
    tone(BUZZER, 850, 100);
    delay(100);
    tone(BUZZER, 1000, 100);
    delay(100);
    tone(BUZZER, 850, 100);
    delay(100);
    tone(BUZZER, 1000, 100);
    delay(100);
    noTone(BUZZER);
    digitalWrite(BUZZER, 0);
}

// This function sends system and sensor data to the BLE module over Serial2
void txBLE()
{
    if (!sendPKTtoBLE)
        return;
    Serial2.print("%CSTM,M,");
    Serial2.print(csID, HEX);
    Serial2.print(","); // GPS Data
    Serial2.print(GPS.latitudeDegrees, 6);
    Serial2.print(",");
    Serial2.print(GPS.longitudeDegrees, 6);
    Serial2.print(",");
    Serial2.print(GPS.altitude, 2);
    Serial2.print(",");
    Serial2.print(GPS.satellites);
    Serial2.print(",");
    Serial2.print(GPS.speed);
    Serial2.print(",");
    Serial2.print(GPS.angle);
    Serial2.print(","); // MCB Atmospherics
    Serial2.print(sData.t1, 2);
    Serial2.print(",");
    Serial2.print(sData.t2, 2);
    Serial2.print(",");
    Serial2.print(sData.rh, 2);
    Serial2.print(","); // IMU Data
    Serial2.print(sData.ax, 2);
    Serial2.print(",");
    Serial2.print(sData.ay, 2);
    Serial2.print(",");
    Serial2.print(sData.az, 2);
    Serial2.print(",");
    Serial2.print(sData.accelMagnitude, 2);
    Serial2.print(",");
    Serial2.print(sData.gx, 2);
    Serial2.print(",");
    Serial2.print(sData.gy, 2);
    Serial2.print(",");
    Serial2.print(sData.gz, 2);
    Serial2.print(",");
    Serial2.print(sData.mx, 2);
    Serial2.print(",");
    Serial2.print(sData.my, 2);
    Serial2.print(",");
    Serial2.print(sData.mz, 2);
    Serial2.print(","); // BMS Data
    Serial2.print(sys.bms_ch1);
    Serial2.print(",");
    Serial2.print(sys.bms_ch2);
    Serial2.print(",");
    Serial2.print(sys.bms_ch3);
    Serial2.print(",");
    Serial2.print(sys.bms_ch4);
    Serial2.print(",");
    Serial2.print(sys.batV, 2);
    Serial2.print(",");
    Serial2.print(sys.batPct, 0);
    Serial2.print(",");
    Serial2.print(sys.sol1V, 1);
    Serial2.print(",");
    Serial2.print(sys.sol2V, 1);
    Serial2.print(",");
    Serial2.print(sys.bms_err);
    Serial2.println("");
}

// Function for USB debugging to allow communication of devices over a USB connection
void usbDebug()
{
    // return;
    Serial.print("%CSTM,M,");
    Serial.print(csID, HEX);
    Serial.print(","); // GPS Data
    Serial.print(GPS.latitudeDegrees, 6);
    Serial.print(",");
    Serial.print(GPS.longitudeDegrees, 6);
    Serial.print(",");
    Serial.print(GPS.altitude, 2);
    Serial.print(",");
    Serial.print(GPS.satellites);
    Serial.print(",");
    Serial.print(GPS.speed);
    Serial.print(",");
    Serial.print(GPS.angle);
    Serial.print(","); // MCB Atmospherics
    Serial.print(sData.t1, 2);
    Serial.print(",");
    Serial.print(sData.t2, 2);
    Serial.print(",");
    Serial.print(sData.rh, 2);
    Serial.print(","); // IMU Data
    Serial.print(sData.ax, 2);
    Serial.print(",");
    Serial.print(sData.ay, 2);
    Serial.print(",");
    Serial.print(sData.az, 2);
    Serial.print(",");
    Serial.print(sData.accelMagnitude, 2);
    Serial.print(",");
    Serial.print(sData.gx, 2);
    Serial.print(",");
    Serial.print(sData.gy, 2);
    Serial.print(",");
    Serial.print(sData.gz, 2);
    Serial.print(",");
    Serial.print(sData.mx, 2);
    Serial.print(",");
    Serial.print(sData.my, 2);
    Serial.print(",");
    Serial.print(sData.mz, 2);
    Serial.print(","); // BMS Data
    Serial.print(sys.bms_ch1);
    Serial.print(",");
    Serial.print(sys.bms_ch2);
    Serial.print(",");
    Serial.print(sys.bms_ch3);
    Serial.print(",");
    Serial.print(sys.bms_ch4);
    Serial.print(",");
    Serial.print(sys.batV, 2);
    Serial.print(",");
    Serial.print(sys.batPct, 0);
    Serial.print(",");
    Serial.print(sys.sol1V, 1);
    Serial.print(",");
    Serial.print(sys.sol2V, 1);
    Serial.print(",");
    Serial.print(sys.bms_err);
    Serial.println("");
}


void nrf24Scan()
{
}

// This function prints the file directory for the SD card
void printDirectory(File dir, int numTabs)
{
    File entry = dir.openNextFile();
    if (!entry)
    {
        // no more files
        return;
    }
    for (uint8_t i = 0; i < numTabs; i++)
    {
        Serial2.print('\t');
    }
    Serial2.print(entry.name());
    if (entry.isDirectory())
    {
        Serial2.println("/");
        printDirectory(entry, numTabs + 1);
    }
    else
    {
        // files have sizes, directories do not
        Serial2.print("\t\t");
        Serial2.print(entry.size(), DEC);
        time_t cr = entry.getCreationTime();
        time_t lw = entry.getLastWrite();
        struct tm *tmstruct = localtime(&cr);
        Serial2.printf("\tCREATION: %d-%02d-%02d %02d:%02d:%02d", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
        tmstruct = localtime(&lw);
        Serial2.printf("\tLAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
    }
    entry.close();
}

// This function transmits the GPS, TPU, and BMS data to the ASB over radio
char packet[PACKET_SIZE];
void tx_packet()
{
    for (int i = 0; i < PACKET_SIZE; i++)
        packet[i] = '0'; // Clear Packet
    uint8_t report = 0;
    // Put Data into 32 Byte Packet Chunks
    // GPS Main Packet
    sprintf(packet, "#A,%02X,%.5f,%.5f,",
            csID, GPS.latitudeDegrees, GPS.longitudeDegrees);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    // GPS Aux Packet
    sprintf(packet, "#B,%02X,%.1f,%d,%.1f,%.1f,",
            csID, GPS.altitude, GPS.satellites, GPS.speed, GPS.angle);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    // TPU Main Packet
    sprintf(packet, "#C,%02X,%.2f,%.2f,%.2f,",
            csID, sData.t1, sData.t2, sData.rh);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    // BMS Main Packet
    sprintf(packet, "#G,%02X,%1d,%1d,%1d,%1d,%2.1f,%03.0f,",
            csID, sys.bms_ch1, sys.bms_ch2, sys.bms_ch3, sys.bms_ch4, sys.batV, sys.batPct);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    // BMS Aux Packet
    sprintf(packet, "#H,%02X,%2.1f,%2.1f,%d,",
            csID, sys.sol1V, sys.sol2V, sys.bms_err);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
}

// This function transmits the IMU orientation data to the ASB over radio
void txOrientationPkt()
{
    uint8_t report = 0;
    // ACCEL Main Packet
    sprintf(packet, "#D,%02X,%03.2f,%03.2f,%03.2f, %03.2f",
            csID, sData.ax, sData.ay, sData.az, sData.accelMagnitude);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    // GRYO Main Packet
    sprintf(packet, "#E,%02X,%03.2f,%03.2f,%03.2f,",
            csID, sData.gx, sData.gy, sData.gz);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    // MAG Main Packet
    sprintf(packet, "#F,%02X,%03.2f,%03.2f,%03.2f,",
            csID, sData.mx, sData.my, sData.mz);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    // Make sure the filter initializes, begins, and is updating or the following values are meaningless
    sprintf(packet, "#I,%02X,%03.2f,%03.2f,%03.2f,%03.2f,",
            csID, filter.getQuatW(), filter.getQuatX(), filter.getQuatY(), filter.getQuatZ());
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
}

// This function transmits the atmospheric data to the ASB over radio
void txASBPacket()
{
    sprintf(packet, "#J,%02X,%02.1f,%02.1f,%03.1f,%04.1f,",
            csID, ad.t1, ad.t2, ad.rh, ad.p1);
    int report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    sprintf(packet, "#K,%02X,%05.0f,%1d,%06d,%05d,",
            csID, ad.pa1, ad.aqi, ad.tvoc, ad.eco2);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    sprintf(packet, "#L,%02X,%05d,%02.2f,",
            csID, ad.co2, ad.typ);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    sprintf(packet, "#M,%02X,%04.1f,%04.1f,%04.1f,",
            csID, ad.pm10, ad.pm25, ad.pm40);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    sprintf(packet, "#N,%02X,%04.1f,%05.1f,%05.1f,",
            csID, ad.pm100, ad.nc05, ad.nc10);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
    sprintf(packet, "#O,%02X,%05.1f,%05.1f,%05.1f,*",
            csID, ad.nc25, ad.nc40, ad.nc100);
    report = radio.write(&packet, PACKET_SIZE);
    (report) ? flashGreen() : flashRed();
}