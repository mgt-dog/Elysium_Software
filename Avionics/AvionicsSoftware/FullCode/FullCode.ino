// Elysium Avionics Software
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <Arduino_LSM6DS3.h>


void sensorSample();
void filter();
void sendTelemetry();
void calcApogee() {
  // calculate the apogee based on current velocity, altitude, and elevation angle
  // based on the fourth order  runge-kutta method for approximation
  m=22.389 //Kilograms
  A_r=0.019 //m^2
  p= //kg/m^3
}

//UART 
  //GPS 
    #define gpsSerial Serial0
    SFE_UBLOX_GNSS_SERIAL sam10q; 
    const int gpsRX = 44;
    const int gpsTX = 43;
 
//I2C 
  //LPS
    Adafruit_LPS22 lps;
  const int I2CSDA = 4;
  const int I2CSCL = 5;

//SPI 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // I2C Setup
    Wire.begin(I2CSDA, I2CSCL);

  //GPS Setup
    //Standard 8bit no parity 1 stop bit (SERIAL_8N1)
    gpsSerial.begin(115200, SERIAL_8N1, gpsRX, gpsTX); 
    if (!sam10q.begin(gpsSerial)) //Connect to the u-blox module using mySerial (defined above)
      {Serial.println(F("u-blox GNSS not detected. Retrying..."));}
  //LSP Setup
    if (!lps.begin_I2C()) 
      {Serial.println("Failed to find LPS22 chip");}
    lps.setDataRate(LPS22_RATE_25_HZ);
  // LSM Setup
    if (!IMU.begin()) 
    {Serial.println("Failed to initialize IMU!");}
}

void loop() {
  // put your main code here, to run repeatedly:

}

void sensorSample(){
  //GPS
    if(sam10q.getPVT() == true){
      float latitude = sam10q.getLatitude();
      float longitude = sam10q.getLongitude();
      float altitude = sam10q.getAltitudeMSL();
    }
  //LPS
    sensors_event_t temp;
    sensors_event_t pres;
    lps.getEvent(&pres, &temp);// get pressure
    float temperature = temp.temperature;
    float pressure = pres.pressure;
  //LSM
    float gx,gy,gz;
    if(IMU.gyroscopeAvailable()){
      IMU.readGyroscope(gx,gy,gz);
    }
    float ax,ay,az;
    if(IMU.accelerationAvailable()){
      IMU.readAcceleration(ax,ay,az);
    }

}
