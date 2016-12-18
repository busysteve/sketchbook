// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
int led = 13;
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void setup(){
  pinMode(led, OUTPUT); 
  pinMode(11, OUTPUT); 
  pinMode(10, OUTPUT); 
  pinMode(6, OUTPUT); 
  pinMode(5, OUTPUT); 
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop(){
  readChip();
  
  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(30);               // wait for a second
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  
  int center = 1500.00;
  
  int accy = getFloat( AcY );
  
  if( accy > center + 100.00 )
  {
     analogWrite( 10, 0 );
     analogWrite( 11, 255 );
     
     analogWrite( 5, 0 );
     analogWrite( 6, 255 );
  }
  else if( accy < center - 100.00 )
  {
     analogWrite( 11, 0 );
     analogWrite( 10, 255 );

     analogWrite( 6, 0 );
     analogWrite( 5, 255 );
  }
  else if( 0 && accy > center + 100.00 )
  {
     analogWrite( 10, 0 );
     analogWrite( 11, 190 );
     
     analogWrite( 5, 0 );
     analogWrite( 6, 190 );
  }
  else if( 0 && accy < center - 100.00 )
  {
     analogWrite( 11, 0 );
     analogWrite( 10, 190 );

     analogWrite( 6, 0 );
     analogWrite( 5, 190 );
  }
  //serialOut();
  //Serial.println( getAX() );
  
  delay(10);
}

float getFloat( int val )
{
  return val;
}

void readChip()
{
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
  
}

void serialOut()
{
  Serial.print("AcX = "); Serial.print( getFloat(AcX));
  Serial.print(" | AcY = "); Serial.print(getFloat(AcY));
  Serial.print(" | AcZ = "); Serial.print(getFloat(AcZ));
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(getFloat(GyX));
  Serial.print(" | GyY = "); Serial.print(getFloat(GyY));
  Serial.print(" | GyZ = "); Serial.println(getFloat(GyZ));
}
