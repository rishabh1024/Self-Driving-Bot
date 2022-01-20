 // MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int left1= 2, left2= 4, right1= 7, right2= 8, enable1=3, enable2= 5, ir3= 12,ir4= 11,ir1=10,ir2=13;

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  pinMode(ir3,INPUT);
  pinMode(ir4,INPUT);
  pinMode(ir1,INPUT);
  pinMode(ir2,INPUT);
  pinMode(left1,OUTPUT);
  pinMode(left2,OUTPUT);
  pinMode(right1,OUTPUT);
  pinMode(right2,OUTPUT);
  pinMode(enable1,OUTPUT);
  pinMode(enable2,OUTPUT);
  digitalWrite(enable1,HIGH);
  digitalWrite(enable2,HIGH);
}

int err=30;

void forward()
{
   analogWrite(enable1,122);
   analogWrite(enable2,100);
   digitalWrite(left2,HIGH);
   digitalWrite(right2,HIGH); //Forward
   digitalWrite(left1,LOW);
   digitalWrite(right1,LOW);
}

void rright()
{
  //analogWrite(enable1,100);
  //analogWrite(enable2,130);
  digitalWrite(left2,HIGH);
  digitalWrite(right2,LOW); //Onthespotright
  digitalWrite(left1,LOW);
  digitalWrite(right1,LOW);
  
}

void rleft()
{
  //analogWrite(enable1,130);
  //analogWrite(enable2,100);
  digitalWrite(left2,LOW);
  digitalWrite(right2,HIGH); //Onthespotleft
  digitalWrite(left1,LOW);
  digitalWrite(right1,LOW);
  
}

void Stop()
{
  digitalWrite(left2,LOW);
  digitalWrite(right2,LOW); //Stop
  digitalWrite(left1,LOW);
  digitalWrite(right1,LOW);
}

void loop(){
  
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
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(1000);
  if((GyZ<400)&&(GyZ>-100))
  {
    start:
    Serial.println("Forward");
    forward();
  }
  else if(GyZ<-50)
  {
    Serial.println("Left");
    rleft();
    delay(5);
    goto start;
  }
  else if(GyZ>600)
  {
    Serial.println("Right");
    rright();
    delay(5);
    goto start;
  }

    
  
}
