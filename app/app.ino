
#include <Wire.h>
#include <TimerOne.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

// Initial time
long int ti;
volatile bool intFlag=false;

unsigned long interval=3000;
unsigned long previousMillis=0;

TinyGPS gps;//Declaramos el objeto gps
SoftwareSerial serialgps(3,4);//Declaramos el pin 4 Tx y 3 Rx
//Declaramos la variables para la obtención de datos
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long chars;
unsigned short sentences, failed_checksum;

// Counter
long int cpt=0;

// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data){
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index=0;
  while (Wire.available())
  Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data){
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void setup() {
    Serial.begin(19200);//Iniciamos el puerto serie
    serialgps.begin(9600);//Iniciamos el puerto serie del gps

    Wire.begin();

    // Set accelerometers low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS,29,0x06);
    // Set gyroscope low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS,26,0x06);

    // Configure gyroscope range
    I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
    // Configure accelerometers range
    I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);

    // Request continuous magnetometer measurements in 16 bits
    I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

    pinMode(13, OUTPUT);
    Timer1.initialize(10000); // initialize timer1, and set a 1/2 second period
    Timer1.attachInterrupt(callback); // attaches callback() as a timer overflow interrupt

    // Store initial time
    ti=millis();
}

void callback(){
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

void loop() {
  unsigned long currentMillis = millis();
  if ((unsigned long)(currentMillis - previousMillis) >= interval) {
    IMU();
    GPS();
    previousMillis = millis();
  }
}

static void IMU(){{
  while (!intFlag);
  intFlag=false;

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);

  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];

  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];

  uint8_t ST1;
  do {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }

  while (!(ST1&0x01));

  // Read magnetometer data
  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // Magnetometer
  int16_t mx=-(Mag[3]<<8 | Mag[2]);
  int16_t my=-(Mag[1]<<8 | Mag[0]);
  int16_t mz=-(Mag[5]<<8 | Mag[4]);

  Serial.println("{\"type\": \"metrics\", \"data\": {\
    \"accelerometer\": {\"x\":"+String(ax)+", \"y\":"+String(ay)+", \"z\":"+String(az)+"},\
    \"gyroscope\": {\"x\":"+String(gx)+", \"y\":"+String(gy)+", \"z\":"+String(gz)+"},\
    \"magnetometer\": {\"x\":"+String(mx)+", \"y\":"+String(my)+", \"z\":"+String(mz)+"}\
  }}#");
}}


static void GPS(){
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;){
    while (serialgps.available()){
      char c = serialgps.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if(gps.encode(c)){

        bool newData = false;
        unsigned long chars;
        unsigned short sentences, failed;

        // For one second we parse GPS data and report some key values
        for (unsigned long start = millis(); millis() - start < 1000;){
          while (serialgps.available()){
            char c = serialgps.read();
            // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
            if (gps.encode(c)) {
              newData = true;
            }
          }
        }

        if (newData){
          float latitude, longitude;
          gps.f_get_position(&latitude, &longitude);
          gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
          gps.stats(&chars, &sentences, &failed_checksum);

          Serial.println("{\"type\": \"gps\", \"data\": {\
            \"latitude\":"+String(latitude, 7)+",\
            \"longitude\":"+String(longitude, 7)+",\
            \"f_altitude\":"+String(gps.f_altitude())+",\
            \"f_course\": "+String(gps.f_course())+",\
            \"f_speed_kmph\": "+String(gps.f_speed_kmph())+",\
            \"satellites\": "+String(gps.satellites())+"\
          }}#");
        }
      }
    }
  }
}
