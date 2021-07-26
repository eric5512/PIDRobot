#include <Wire.h>

#define    MPU9250_ADDRESS            0x68

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G         0x00  
#define    ACC_FULL_SCALE_4_G         0x08
#define    ACC_FULL_SCALE_8_G         0x10
#define    ACC_FULL_SCALE_16_G        0x18

#define    FOW   3
#define    BACK  2     
#define    EN    10



const float setpoint = 0;
float error = 0;
float pError = 0;
const double RtoD = 57.2957;
const double AGtoA = 0.007758;
float calGy;
unsigned long Ti;
unsigned long tiempo;
uint8_t Gyr[2];
uint8_t Acc[2];
float Gx;
float Az;
float angleG=0;
float angleA=1;
float angle=0;
const float Kp=35;
const float Ki=2;
const float Kd=0.5;
float P=0;
float I=0;
float D=0;
float pid=0;

void forward(int n){
  digitalWrite(BACK, LOW);
  digitalWrite(FOW, HIGH);
  analogWrite(EN, n);
}


void backward(int n){
  digitalWrite(FOW, LOW);
  digitalWrite(BACK, HIGH);
  analogWrite(EN, n);
}


void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

float offset(){
  float off=0;
  for(int n=0; n<500;n++){
  I2Cread(MPU9250_ADDRESS, 67, 2, Gyr);
  Gx= Gyr[0]<<8 | Gyr[1];
  off+=Gx;
  }
  off/=500;
  return off;
}


void setup()
{

  Wire.begin();
  
  pinMode(13,OUTPUT);
  
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
  
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);

  delay(100);
  calGy=offset();

  while(angleA !=0){
   I2Cread(MPU9250_ADDRESS, 63, 2, Acc);
   Az= Acc[0]<<8 | Acc[1];
   angleA=asin(Az/8230);
   angleA*=RtoD;
  }
  digitalWrite(13, HIGH);
}



void loop()
{
 Ti=millis();
 for(int i=0;i<50;i++){
  I2Cread(MPU9250_ADDRESS, 67, 2, Gyr);
  Gx= Gyr[0]<<8 | Gyr[1];
  Gx-=calGy;
  if((Gx>-150 && Gx<150) || (Gx<-500000 || Gx>50000)){
   continue;
  }
  angleG=angleG+(float)Gx;
 }

 tiempo = millis()-Ti;
 angleG=angleG*(tiempo)*0.001;
 angleG*=0.02;
 angleG*=AGtoA;

 if (angleG>100){
  angleG = 0;
 }
 
 angle+=angleG;

 if (angle > 90){
  angle = 0;
 }

 error = setpoint - angle;
 P = error * Kp;
 I+= error * Ki;
 D = (pError - error) * Kd;
 pError = error;
 pid=P+I+D;
 
 if(pid>=255){
  pid=254;
 }
 
 if(pid<=-255){
  pid=-254;
 }
 
 
 if(pid>0){
  backward(0);
  forward(pid);
 }
 
 else{
  forward(0);
  backward(-pid);
 }
}



