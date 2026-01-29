// Elysium Avionics Software
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <Arduino_LSM6DS3.h>
#include <math.h>

// function Declerations
  void sensorSample();
  void filter();
  void sendTelemetry();
  float calcApogee();
  float calcVelocity(float h, float v_x, float v_y, float p, float area, float C_d);
  float  calcAcceleration(float velocity_x, float velocity_y, float p, float area);
  void stateMachine(float tAcel, float Alti, float velY);

//Global Variables
  float altitude;
  float velocityX, velocityY, velocityZ;
  float accelerationX, accelerationY, accelerationZ;
  float temperature, pressure;

  enum fstate{LAUNCHPAD, ASCENT, COAST, APOGEE, DESCENT, LANDED};
  fstate STATE;
//UART 
  //GPS 
    #define gpsSerial Serial0
    SFE_UBLOX_GNSS_SERIAL sam10q; 
    const int gpsRX = 44;
    const int gpsTX = 43;
 
//I2C 
  //LPS
    Adafruit_LPS25 lps;
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
      {Serial.println("Failed to find LPS25 chip");}
    lps.setDataRate(LPS25_RATE_25_HZ);
  // LSM Setup
    if (!IMU.begin()) 
    {Serial.println("Failed to initialize IMU!");}
}

void loop() {
  // put your main code here, to run repeatedly:

}
  void stateMachine(float tAcel, float Alti, float velY){
    static float lastTime;
    switch(STATE){
      case LAUNCHPAD:
       if((tAcel > 19.6 && Alti>10.0f)||(Alti>30.0f))
          { STATE = ASCENT; }
        break;

      case ASCENT:
        if(tAcel< 0) //include time t>3.96s later for burnout time of motor
          { STATE = COAST; }
        break;

      case COAST: 
        if (tAcel < 0 && velY < 0)
          { STATE = DESCENT; 
            lastTime = millis();
          } //instert automatic closing of airbreaks right here
        break;

      case DESCENT:

        if(millis()-lastTime > 30) //figure out a better way other than delay T-T
        { 
          if(velY<0.5 && velY>-0.5){
             delay(15);
             if(velY<0.5 && velY>-0.5{
              STATE = LANDED;
             }
          }
        }
      break;

      case LANDED:
        break;

    }
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
     temperature = temp.temperature;
     pressure = pres.pressure;
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

float calcApogee() {
  // calculate the apogee based on current velocity, altitude, and elevation angle
  // based on the fourth order  runge-kutta method for approximation
  // initiate all variables
  const float t_h = 0.1;
  const float m = 22.389; //Kilograms
  const float A_r = 0.019; //m^2
  const float C_d = 0.375585;
  float y_p = altitude;
  float v_x = pow(pow(velocityX,2)+pow(velocityY,2),1/2);
  float v_y = velocityZ;
  float p = 1.225; //kg/m^3
  float apogee;
  float k_1;
  float k_2;
  float k_3;
  float k_4;
  while (v_y > 0) {
    k_1 = calcVelocity(t_h, v_x, v_y, p, A_r, C_d);
    k_2 = calcVelocity(t_h/2, v_x, v_y, p, A_r, C_d);
    k_3 = calcVelocity(t_h/2, v_x, v_y, p, A_r, C_d);
    k_4 = calcVelocity(t_h, v_x, v_y, p, A_r, C_d);
    y_p = y_p + t_h/6*(k_1+2*k_2+2*k_3+k_4);
  }
  apogee = y_p;
  Serial.printf("Predicted apogee is %f", apogee);
  return apogee;
}

float calcVelocity(float h, float v_x, float v_y, float p, float area, float C_d) {
  float theta = atan(v_y/v_x);
  float a_d = -1*sin(theta)*0.5*p*C_d*(pow(v_x,2)+pow(v_y,2));
  float a_g = -1*9.80665;
  float a = a_d + a_g;
  v_y -= a*h;
  
  return 0;
}