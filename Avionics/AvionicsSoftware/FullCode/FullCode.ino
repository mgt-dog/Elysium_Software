// Elysium Avionics Software
#include <SparkFun_u-blox_GNSS_v3.h>

void sensorSample();
void filter();
void sendTelemetry();
void calcAltitude();

#define gpsSerial Serial0
SFE_UBLOX_GNSS_SERIAL sam10q; 
const int gpsRX = 44;
const int gpsTX = 43;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //GPS Setup
    //Standard 8bit no parity 1 stop bit (SERIAL_8N1)
    gpsSerial.begin(115200, SERIAL_8N1, gpsRX, gpsTX); 
    if (sam10q.begin(gpsSerial) == false) //Connect to the u-blox module using mySerial (defined above)
    {Serial.println(F("u-blox GNSS not detected. Retrying..."));}

}

void loop() {
  // put your main code here, to run repeatedly:

}

void sensorSample(){
  //GPS
  if(sam10q.getPVT() == true)
  float latitude = sam10q.getLatitude();
  float longitude = sam10q.getLongitude();
  float altitude = sam10q.getAltitudeMSL();

}
