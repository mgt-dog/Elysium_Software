// Elysium Avionics Software
#include <SparkFun_u-blox_GNSS_v3.h>

void sensorSample();
void filter();
void sendTelemetry();
void calcAltitude();

#define gpsSerial Serial0
const int gpsRX = 44;
const int gpsTX = 43;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  gpsSerial.begin(115200);

  gpsSerial.setRx(gpsRX);
  gpsSerial.setTx(gpsTX);



}

void loop() {
  // put your main code here, to run repeatedly:

}

void sensorSample(){

}
